/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <drivers/gpio.h>
#include <soc.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <settings/settings.h>

#include <nrfx_gpiote.h>
#include <devicetree.h>
#include <assert.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define GPIO_FEM_TX 28 // Connected to FEM TX PIN
#define GPIO_FEM_RX 29 // Connected to FEM RX PIN
#define GPIO_FEM_NO_PINS 2

static volatile uint32_t event_count = 0;

static const nrfx_gpiote_pin_t mpsl_fem_pin_no[GPIO_FEM_NO_PINS] = {
	GPIO_FEM_TX, GPIO_FEM_RX
};

static void monitor_gpiote_irq_handler(nrfx_gpiote_pin_t pin,
				       nrf_gpiote_polarity_t action)
{
	event_count++;
}

/* Initialize and start timer and interupt handling */
static void monitor_init(void)
{
	nrfx_err_t err = nrfx_gpiote_init(0);
	assert(err == NRFX_ERROR_INVALID_STATE);

	nrfx_gpiote_in_config_t gpiote_in_config =
		NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);

	for (int i = 0; i < GPIO_FEM_NO_PINS; i++) {
		int err = nrfx_gpiote_in_init(mpsl_fem_pin_no[i],
					  &gpiote_in_config,
					  monitor_gpiote_irq_handler);
		assert(err == NRFX_SUCCESS);
	}
}

static void monitor_start(void)
{
	/* Clear event log */
	event_count = 0;

	/* Enable interupts */
	for (int i = 0; i < GPIO_FEM_NO_PINS; i++) {
		nrfx_gpiote_in_event_enable(mpsl_fem_pin_no[i], true);
	}
}

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

	printk("Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);
}

#ifdef CONFIG_BT_LBS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Security changed: %s level %u\n", addr, level);
	} else {
		printk("Security failed: %s level %u err %d\n", addr, level,
		       err);
	}
}
#endif

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_LBS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_LBS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static void pairing_confirm(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	bt_conn_auth_pairing_confirm(conn);

	printk("Pairing confirmed: %s\n", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing failed conn: %s, reason %d\n", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
	.pairing_confirm = pairing_confirm,
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
#endif

void main(void)
{
	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(gpiote)),
		    DT_IRQ(DT_NODELABEL(gpiote), priority), nrfx_isr,
		    nrfx_gpiote_irq_handler, 0);

	printk("Starting Bluetooth Peripheral LBS example\n");

	monitor_init();

	bt_conn_cb_register(&conn_callbacks);
	if (IS_ENABLED(CONFIG_BT_LBS_SECURITY_ENABLED)) {
		bt_conn_auth_cb_register(&conn_auth_callbacks);
	}

	int err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	monitor_start();

	printk("Advertising successfully started\n");

	for (;;) {
		k_sleep(K_MSEC(2000));
		printk("event_count: %d\n", event_count);
	}
}
