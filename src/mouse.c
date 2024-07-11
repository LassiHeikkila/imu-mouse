/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

// Extended by Lassi Heikkilä

#include <assert.h>
#include <errno.h>
#include <stddef.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>

#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include <bluetooth/services/hids.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/dis.h>

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "battery.h"
#include "buttons.h"
#include "mouse.h"

LOG_MODULE_REGISTER(mouse);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define BASE_USB_HID_SPEC_VERSION 0x0101

/* Number of pixels by which the cursor is moved when a button is pushed. */
#define MOVEMENT_SPEED 50
/* Number of input reports in this application. */
#define INPUT_REPORT_COUNT 3
/* Length of Mouse Input Report containing button data. */
#define INPUT_REP_BUTTONS_LEN 3
/* Length of Mouse Input Report containing movement data. */
#define INPUT_REP_MOVEMENT_LEN 3
/* Length of Mouse Input Report containing media player data. */
#define INPUT_REP_MEDIA_PLAYER_LEN 1
/* Index of Mouse Input Report containing button data. */
#define INPUT_REP_BUTTONS_INDEX 0
/* Index of Mouse Input Report containing movement data. */
#define INPUT_REP_MOVEMENT_INDEX 1
/* Index of Mouse Input Report containing media player data. */
#define INPUT_REP_MPLAYER_INDEX 2
/* Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_REF_BUTTONS_ID 1
/* Id of reference to Mouse Input Report containing movement data. */
#define INPUT_REP_REF_MOVEMENT_ID 2
/* Id of reference to Mouse Input Report containing media player data. */
#define INPUT_REP_REF_MPLAYER_ID 3

// HIDs queue sizes
#define MOVEMENT_QUEUE_SIZE 10
#define CLICK_QUEUE_SIZE 10

/* Key used to move cursor left */
#define KEY_LEFT_MASK DK_BTN1_MSK
/* Key used to move cursor up */
#define KEY_UP_MASK DK_BTN2_MSK
/* Key used to move cursor right */
#define KEY_RIGHT_MASK DK_BTN3_MSK
/* Key used to move cursor down */
#define KEY_DOWN_MASK DK_BTN4_MSK

/* Key used to accept or reject passkey value */
#define KEY_PAIRING_ACCEPT BUTTON_LEFT
#define KEY_PAIRING_REJECT BUTTON_MIDDLE

// bit indices for boot mode button report
#define LEFT_BUTTON_BOOT_MODE_IDX 0
#define MIDDLE_BUTTON_BOOT_MODE_IDX 1
#define RIGHT_BUTTON_BOOT_MODE_IDX 2

// array indices for regular mode button report
#define LEFT_BUTTON_IDX 0
#define MIDDLE_BUTTON_IDX 1
#define RIGHT_BUTTON_IDX 2

/* HIDS instance. */
BT_HIDS_DEF(hids_obj, INPUT_REP_BUTTONS_LEN, INPUT_REP_MOVEMENT_LEN,
            INPUT_REP_MEDIA_PLAYER_LEN);

static struct k_work movement_work;
struct mouse_pos
{
    int16_t x_val;
    int16_t y_val;
};

static struct k_work click_work;
struct mouse_event
{
    int key;
    bool state;
};

// Mouse movement queue
K_MSGQ_DEFINE(movement_queue, sizeof(struct mouse_pos), MOVEMENT_QUEUE_SIZE, 4);

// Mouse event queue
K_MSGQ_DEFINE(mouse_event_queue, sizeof(struct mouse_event), CLICK_QUEUE_SIZE,
              4);

#if CONFIG_BT_DIRECTED_ADVERTISING
/* Bonded address queue. */
K_MSGQ_DEFINE(bonds_queue, sizeof(bt_addr_le_t), CONFIG_BT_MAX_PAIRED, 4);
#endif

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
                  (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
                  (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static struct conn_mode
{
    struct bt_conn *conn;
    bool in_boot_mode;
} conn_mode[CONFIG_BT_HIDS_MAX_CLIENT_COUNT];

static struct k_work adv_work;

static struct k_work pairing_work;
struct pairing_data_mitm
{
    struct bt_conn *conn;
    unsigned int passkey;
};

K_MSGQ_DEFINE(mitm_queue, sizeof(struct pairing_data_mitm),
              CONFIG_BT_HIDS_MAX_CLIENT_COUNT, 4);

#if CONFIG_BT_DIRECTED_ADVERTISING
static void bond_find(const struct bt_bond_info *info, void *user_data) {
    int err;

    /* Filter already connected peers. */
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (conn_mode[i].conn) {
            const bt_addr_le_t *dst = bt_conn_get_dst(conn_mode[i].conn);

            if (!bt_addr_le_cmp(&info->addr, dst)) {
                return;
            }
        }
    }

    err = k_msgq_put(&bonds_queue, (void *)&info->addr, K_NO_WAIT);
    if (err) {
        LOG_ERR("No space in the queue for the bond.");
    }
}
#endif

static void advertising_continue(void) {
    struct bt_le_adv_param adv_param;

#if CONFIG_BT_DIRECTED_ADVERTISING
    bt_addr_le_t addr;

    if (!k_msgq_get(&bonds_queue, &addr, K_NO_WAIT)) {
        char addr_buf[BT_ADDR_LE_STR_LEN];

        adv_param = *BT_LE_ADV_CONN_DIR(&addr);
        adv_param.options |= BT_LE_ADV_OPT_DIR_ADDR_RPA;

        int err = bt_le_adv_start(&adv_param, NULL, 0, NULL, 0);

        if (err) {
            LOG_ERR("Directed advertising failed to start");
            return;
        }

        bt_addr_le_to_str(&addr, addr_buf, BT_ADDR_LE_STR_LEN);
        LOG_INF("Direct advertising to %s started", addr_buf);
    } else
#endif
    {
        int err;

        adv_param = *BT_LE_ADV_CONN;
        // adv_param.options |= BT_LE_ADV_OPT_ONE_TIME;
        err =
            bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
        if (err) {
            LOG_ERR("Advertising failed to start (err %d)", err);
            return;
        }

        LOG_INF("Regular advertising started");
    }
}

static void advertising_start(void) {
#if CONFIG_BT_DIRECTED_ADVERTISING
    k_msgq_purge(&bonds_queue);
    bt_foreach_bond(BT_ID_DEFAULT, bond_find, NULL);
#endif

    k_work_submit(&adv_work);
}

static void advertising_process(struct k_work *work) { advertising_continue(); }

static void pairing_process(struct k_work *work) {
    int err;
    struct pairing_data_mitm pairing_data;

    char addr[BT_ADDR_LE_STR_LEN];

    err = k_msgq_peek(&mitm_queue, &pairing_data);
    if (err) {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(pairing_data.conn), addr, sizeof(addr));

    printk("Passkey for %s: %06u\n", addr, pairing_data.passkey);
    printk("Press Button 1 to confirm, Button 2 to reject.\n");
}

static void insert_conn_object(struct bt_conn *conn) {
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (!conn_mode[i].conn) {
            conn_mode[i].conn = conn;
            conn_mode[i].in_boot_mode = false;

            return;
        }
    }

    LOG_ERR("Connection object could not be inserted %p", (void *)conn);
}

static bool is_conn_slot_free(void) {
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (!conn_mode[i].conn) {
            return true;
        }
    }

    return false;
}

static void connected(struct bt_conn *conn, uint8_t err) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err) {
        if (err == BT_HCI_ERR_ADV_TIMEOUT) {
            LOG_WRN("Direct advertising to %s timed out", addr);
            k_work_submit(&adv_work);
        } else {
            LOG_ERR("Failed to connect to %s (%u)", addr, err);
        }
        return;
    }

    LOG_INF("Connected %s", addr);

    err = bt_hids_connected(&hids_obj, conn);

    if (err) {
        LOG_ERR("Failed to notify HID service about connection");
        return;
    }

    insert_conn_object(conn);

    if (is_conn_slot_free()) {
        advertising_start();
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    int err;
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected from %s (reason %u)", addr, reason);

    err = bt_hids_disconnected(&hids_obj, conn);

    if (err) {
        LOG_ERR("Failed to notify HID service about disconnection");
    }

    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (conn_mode[i].conn == conn) {
            conn_mode[i].conn = NULL;
            break;
        }
    }

    advertising_start();
}

#ifdef CONFIG_BT_HIDS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
                             enum bt_security_err err) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err) {
        LOG_WRN("Security changed: %s level %u", addr, level);
    } else {
        LOG_ERR("Security failed: %s level %u err %d", addr, level, err);
    }
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
#ifdef CONFIG_BT_HIDS_SECURITY_ENABLED
    .security_changed = security_changed,
#endif
};

static void hids_pm_evt_handler(enum bt_hids_pm_evt evt, struct bt_conn *conn) {
    char addr[BT_ADDR_LE_STR_LEN];
    size_t i;

    for (i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (conn_mode[i].conn == conn) {
            break;
        }
    }

    if (i >= CONFIG_BT_HIDS_MAX_CLIENT_COUNT) {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    switch (evt) {
    case BT_HIDS_PM_EVT_BOOT_MODE_ENTERED:
        LOG_INF("Boot mode entered %s", addr);
        conn_mode[i].in_boot_mode = true;
        break;

    case BT_HIDS_PM_EVT_REPORT_MODE_ENTERED:
        LOG_INF("Report mode entered %s", addr);
        conn_mode[i].in_boot_mode = false;
        break;

    default:
        break;
    }
}

static void hid_init(void) {
    int err;
    struct bt_hids_init_param hids_init_param = {0};
    struct bt_hids_inp_rep *hids_inp_rep;
    static const uint8_t
        mouse_movement_mask[DIV_ROUND_UP(INPUT_REP_MOVEMENT_LEN, 8)] = {0};

    static const uint8_t report_map[] = {
        0x05, 0x01, /* Usage Page (Generic Desktop) */
        0x09, 0x02, /* Usage (Mouse) */

        0xA1, 0x01, /* Collection (Application) */

        /* Report ID 1: Mouse buttons + scroll/pan */
        0x85, 0x01,       /* Report Id 1 */
        0x09, 0x01,       /* Usage (Pointer) */
        0xA1, 0x00,       /* Collection (Physical) */
        0x95, 0x05,       /* Report Count (3) */
        0x75, 0x01,       /* Report Size (1) */
        0x05, 0x09,       /* Usage Page (Buttons) */
        0x19, 0x01,       /* Usage Minimum (01) */
        0x29, 0x04,       /* Usage Maximum (04) */
        0x15, 0x00,       /* Logical Minimum (0) */
        0x25, 0x01,       /* Logical Maximum (1) */
        0x81, 0x02,       /* Input (Data, Variable, Absolute) */
        0x95, 0x01,       /* Report Count (1) */
        0x75, 0x03,       /* Report Size (3) */
        0x81, 0x01,       /* Input (Constant) for padding */
        0x75, 0x08,       /* Report Size (8) */
        0x95, 0x01,       /* Report Count (1) */
        0x05, 0x01,       /* Usage Page (Generic Desktop) */
        0x09, 0x38,       /* Usage (Wheel) */
        0x15, 0x81,       /* Logical Minimum (-127) */
        0x25, 0x7F,       /* Logical Maximum (127) */
        0x81, 0x06,       /* Input (Data, Variable, Relative) */
        0x05, 0x0C,       /* Usage Page (Consumer) */
        0x0A, 0x38, 0x02, /* Usage (AC Pan) */
        0x95, 0x01,       /* Report Count (1) */
        0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
        0xC0,             /* End Collection (Physical) */

        /* Report ID 2: Mouse motion */
        0x85, 0x02,       /* Report Id 2 */
        0x09, 0x01,       /* Usage (Pointer) */
        0xA1, 0x00,       /* Collection (Physical) */
        0x75, 0x0C,       /* Report Size (12) */
        0x95, 0x02,       /* Report Count (2) */
        0x05, 0x01,       /* Usage Page (Generic Desktop) */
        0x09, 0x30,       /* Usage (X) */
        0x09, 0x31,       /* Usage (Y) */
        0x16, 0x01, 0xF8, /* Logical maximum (2047) */
        0x26, 0xFF, 0x07, /* Logical minimum (-2047) */
        0x81, 0x06,       /* Input (Data, Variable, Relative) */
        0xC0,             /* End Collection (Physical) */
        0xC0,             /* End Collection (Application) */

        /* Report ID 3: Advanced buttons */
        0x05, 0x0C, /* Usage Page (Consumer) */
        0x09, 0x01, /* Usage (Consumer Control) */
        0xA1, 0x01, /* Collection (Application) */
        0x85, 0x03, /* Report Id (3) */
        0x15, 0x00, /* Logical minimum (0) */
        0x25, 0x01, /* Logical maximum (1) */
        0x75, 0x01, /* Report Size (1) */
        0x95, 0x01, /* Report Count (1) */

        0x09, 0xCD,       /* Usage (Play/Pause) */
        0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
        0x0A, 0x83, 0x01, /* Usage (Consumer Control Configuration) */
        0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
        0x09, 0xB5,       /* Usage (Scan Next Track) */
        0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
        0x09, 0xB6,       /* Usage (Scan Previous Track) */
        0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */

        0x09, 0xEA,       /* Usage (Volume Down) */
        0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
        0x09, 0xE9,       /* Usage (Volume Up) */
        0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
        0x0A, 0x25, 0x02, /* Usage (AC Forward) */
        0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
        0x0A, 0x24, 0x02, /* Usage (AC Back) */
        0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
        0xC0              /* End Collection */
    };

    hids_init_param.rep_map.data = report_map;
    hids_init_param.rep_map.size = sizeof(report_map);

    hids_init_param.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
    hids_init_param.info.b_country_code = 0x00;
    hids_init_param.info.flags =
        (BT_HIDS_REMOTE_WAKE | BT_HIDS_NORMALLY_CONNECTABLE);

    hids_inp_rep = &hids_init_param.inp_rep_group_init.reports[0];
    hids_inp_rep->size = INPUT_REP_BUTTONS_LEN;
    hids_inp_rep->id = INPUT_REP_REF_BUTTONS_ID;
    hids_init_param.inp_rep_group_init.cnt++;

    hids_inp_rep++;
    hids_inp_rep->size = INPUT_REP_MOVEMENT_LEN;
    hids_inp_rep->id = INPUT_REP_REF_MOVEMENT_ID;
    hids_inp_rep->rep_mask = mouse_movement_mask;
    hids_init_param.inp_rep_group_init.cnt++;

    hids_inp_rep++;
    hids_inp_rep->size = INPUT_REP_MEDIA_PLAYER_LEN;
    hids_inp_rep->id = INPUT_REP_REF_MPLAYER_ID;
    hids_init_param.inp_rep_group_init.cnt++;

    hids_init_param.is_mouse = true;
    hids_init_param.pm_evt_handler = hids_pm_evt_handler;

    err = bt_hids_init(&hids_obj, &hids_init_param);
    __ASSERT(err == 0, "HIDS initialization failed\n");
}

static void mouse_movement_send(int16_t x_delta, int16_t y_delta) {
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {

        if (!conn_mode[i].conn) {
            continue;
        }

        if (conn_mode[i].in_boot_mode) {
            x_delta = MAX(MIN(x_delta, SCHAR_MAX), SCHAR_MIN);
            y_delta = MAX(MIN(y_delta, SCHAR_MAX), SCHAR_MIN);

            bt_hids_boot_mouse_inp_rep_send(&hids_obj, conn_mode[i].conn, NULL,
                                            (int8_t)x_delta, (int8_t)y_delta,
                                            NULL);
        } else {
            uint8_t x_buff[2];
            uint8_t y_buff[2];
            uint8_t buffer[INPUT_REP_MOVEMENT_LEN];

            int16_t x = MAX(MIN(x_delta, 0x07ff), -0x07ff);
            int16_t y = MAX(MIN(y_delta, 0x07ff), -0x07ff);

            /* Convert to little-endian. */
            sys_put_le16(x, x_buff);
            sys_put_le16(y, y_buff);

            /* Encode report. */
            BUILD_ASSERT(sizeof(buffer) == 3,
                         "Only 2 axis, 12-bit each, are supported");

            buffer[0] = x_buff[0];
            buffer[1] = (y_buff[0] << 4) | (x_buff[1] & 0x0f);
            buffer[2] = (y_buff[1] << 4) | (y_buff[0] >> 4);

            bt_hids_inp_rep_send(&hids_obj, conn_mode[i].conn,
                                 INPUT_REP_MOVEMENT_INDEX, buffer,
                                 sizeof(buffer), NULL);
        }
    }
}

static void set_boot_mode_button_value(button_key_t key, bool pressed,
                                       uint8_t *boot_mode_button_state) {
    switch (key) {
    case BUTTON_LEFT: {
        uint8_t val = (pressed ? 1 : 0);
        // clear button bit
        *boot_mode_button_state &= ~(0x1 << LEFT_BUTTON_BOOT_MODE_IDX);
        // set button bit
        *boot_mode_button_state |= (val << LEFT_BUTTON_BOOT_MODE_IDX);
        break;
    }
    case BUTTON_MIDDLE: {
        uint8_t val = (pressed ? 1 : 0);
        // clear button bit
        *boot_mode_button_state &= ~(0x1 << MIDDLE_BUTTON_BOOT_MODE_IDX);
        // set button bit
        *boot_mode_button_state |= (val << MIDDLE_BUTTON_BOOT_MODE_IDX);
        break;
    }
    case BUTTON_RIGHT: {
        uint8_t val = (pressed ? 1 : 0);
        // clear button bit
        *boot_mode_button_state &= ~(0x1 << RIGHT_BUTTON_BOOT_MODE_IDX);
        // set button bit
        *boot_mode_button_state |= (val << RIGHT_BUTTON_BOOT_MODE_IDX);
        break;
    }
    default:
        break;
    }
}

static void mouse_event_send(button_key_t key, bool pressed) {
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {

        if (!conn_mode[i].conn) {
            continue;
        }

        if (conn_mode[i].in_boot_mode) {
            static uint8_t previous_button_state = 0;
            uint8_t button_state = previous_button_state;

            set_boot_mode_button_value(key, pressed, &button_state);
            previous_button_state = button_state;

            bt_hids_boot_mouse_inp_rep_send(&hids_obj, conn_mode[i].conn,
                                            &button_state, 0, 0, NULL);
        } else {
            static uint8_t previous_button_states[INPUT_REP_BUTTONS_LEN] = {
                0, 0, 0};

            uint8_t buffer[INPUT_REP_BUTTONS_LEN];
            buffer[LEFT_BUTTON_IDX] = previous_button_states[LEFT_BUTTON_IDX];
            buffer[RIGHT_BUTTON_IDX] = previous_button_states[RIGHT_BUTTON_IDX];
            buffer[MIDDLE_BUTTON_IDX] =
                previous_button_states[MIDDLE_BUTTON_IDX];

            switch (key) {
            case BUTTON_LEFT:
                buffer[LEFT_BUTTON_IDX] = (pressed ? 1 : 0);
                break;
            case BUTTON_MIDDLE:
                buffer[MIDDLE_BUTTON_IDX] = (pressed ? 1 : 0);
                break;
            case BUTTON_RIGHT:
                buffer[RIGHT_BUTTON_IDX] = (pressed ? 1 : 0);
                break;
            }

            previous_button_states[LEFT_BUTTON_IDX] = buffer[LEFT_BUTTON_IDX];
            previous_button_states[RIGHT_BUTTON_IDX] = buffer[RIGHT_BUTTON_IDX];
            previous_button_states[MIDDLE_BUTTON_IDX] =
                buffer[MIDDLE_BUTTON_IDX];

            bt_hids_inp_rep_send(&hids_obj, conn_mode[i].conn,
                                 INPUT_REP_BUTTONS_INDEX, buffer,
                                 sizeof(buffer), NULL);
        }
    }
}

static void mouse_movement_handler(struct k_work *work) {
    struct mouse_pos pos;

    while (!k_msgq_get(&movement_queue, &pos, K_NO_WAIT)) {
        mouse_movement_send(pos.x_val, pos.y_val);
    }
}

static void mouse_click_handler(struct k_work *work) {
    struct mouse_event click;
    while (!k_msgq_get(&mouse_event_queue, &click, K_NO_WAIT)) {
        mouse_event_send(click.key, click.state);
    }
}

#if defined(CONFIG_BT_HIDS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey) {
    int err;

    struct pairing_data_mitm pairing_data;

    pairing_data.conn = bt_conn_ref(conn);
    pairing_data.passkey = passkey;

    err = k_msgq_put(&mitm_queue, &pairing_data, K_NO_WAIT);
    if (err) {
        LOG_ERR("Pairing queue is full. Purge previous data.");
    }

    /* In the case of multiple pairing requests, trigger
     * pairing confirmation which needed user interaction only
     * once to avoid display information about all devices at
     * the same time. Passkey confirmation for next devices will
     * be proccess from queue after handling the earlier ones.
     */
    if (k_msgq_num_used_get(&mitm_queue) == 1) {
        k_work_submit(&pairing_work);
    }
}

static void auth_cancel(struct bt_conn *conn) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_WRN("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason) {
    char addr[BT_ADDR_LE_STR_LEN];
    struct pairing_data_mitm pairing_data;

    if (k_msgq_peek(&mitm_queue, &pairing_data) != 0) {
        return;
    }

    if (pairing_data.conn == conn) {
        bt_conn_unref(pairing_data.conn);
        k_msgq_get(&mitm_queue, &pairing_data, K_NO_WAIT);
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_ERR("Pairing failed conn: %s, reason %d", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
    .passkey_display = auth_passkey_display,
    .passkey_confirm = auth_passkey_confirm,
    .cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete, .pairing_failed = pairing_failed};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif /* defined(CONFIG_BT_HIDS_SECURITY_ENABLED) */

static void num_comp_reply(bool accept) {
    struct pairing_data_mitm pairing_data;
    struct bt_conn *conn;

    if (k_msgq_get(&mitm_queue, &pairing_data, K_NO_WAIT) != 0) {
        return;
    }

    conn = pairing_data.conn;

    if (accept) {
        bt_conn_auth_passkey_confirm(conn);
        printk("Numeric Match, conn %p\n", conn);
    } else {
        bt_conn_auth_cancel(conn);
        printk("Numeric Reject, conn %p\n", conn);
    }

    bt_conn_unref(pairing_data.conn);

    if (k_msgq_num_used_get(&mitm_queue)) {
        k_work_submit(&pairing_work);
    }
}

void button_cb(button_key_t key, button_state_t state) {
    struct mouse_event click;
    click.key = key;
    click.state = (state == BUTTON_STATE_PRESSED ? true : false);

    int err;

    if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
        if (k_msgq_num_used_get(&mitm_queue)) {
            if (key == KEY_PAIRING_ACCEPT) {
                num_comp_reply(true);

                return;
            }

            if (key == KEY_PAIRING_REJECT) {
                num_comp_reply(false);

                return;
            }
        }
    }

    err = k_msgq_put(&mouse_event_queue, &click, K_NO_WAIT);
    if (err) {
        LOG_ERR("No space in the queue for button pressed");
        return;
    }
    if (k_msgq_num_used_get(&mouse_event_queue) == 1) {
        k_work_submit(&click_work);
    }
}

// void button_changed(uint32_t button_state, uint32_t has_changed) {
//     bool data_to_send = false;
//     struct mouse_pos pos;
//     uint32_t buttons = button_state & has_changed;

//     memset(&pos, 0, sizeof(struct mouse_pos));

//     if (buttons & KEY_LEFT_MASK) {
//         pos.x_val -= MOVEMENT_SPEED;
//         LOG_INF("%s(): left", __func__);
//         data_to_send = true;
//     }
//     if (buttons & KEY_UP_MASK) {
//         pos.y_val -= MOVEMENT_SPEED;
//         LOG_INF("%s(): up", __func__);
//         data_to_send = true;
//     }
//     if (buttons & KEY_RIGHT_MASK) {
//         pos.x_val += MOVEMENT_SPEED;
//         LOG_INF("%s(): right", __func__);
//         data_to_send = true;
//     }
//     if (buttons & KEY_DOWN_MASK) {
//         pos.y_val += MOVEMENT_SPEED;
//         LOG_INF("%s(): down", __func__);
//         data_to_send = true;
//     }

//     if (data_to_send) {
//         int err;

//         err = k_msgq_put(&movement_queue, &pos, K_NO_WAIT);
//         if (err) {
//             LOG_ERR("No space in the queue for button pressed");
//             return;
//         }
//         if (k_msgq_num_used_get(&movement_queue) == 1) {
//             k_work_submit(&movement_work);
//         }
//     }
// }

void configure_buttons(void) {
    // register our callback to receive button events from button.c
    my_app_button_cb = button_cb;
}

static void bas_notify(void) {
    uint8_t battery_level = battery_percentage();
    bt_bas_set_battery_level(battery_level);
}

int mouse_task(void) {
    int err;

    LOG_INF("Starting Bluetooth Peripheral HIDS mouse");

    // If left button was pressed at boot, clear any previous pairing
    if (button_l_at_boot == BUTTON_STATE_PRESSED) {
        LOG_INF("unpairing previous bluetooth connections");
        err = bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
        if (err) {
            LOG_ERR("failed to unpair: %d", err);
        }
    }

    if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
        err = bt_conn_auth_cb_register(&conn_auth_callbacks);
        if (err) {
            LOG_ERR("Failed to register authorization callbacks.");
            return 0;
        }

        err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
        if (err) {
            LOG_ERR("Failed to register authorization info callbacks.");
            return 0;
        }
    }

    /* DIS initialized at system boot with SYS_INIT macro. */
    hid_init();

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return 0;
    }

    LOG_INF("Bluetooth initialized");

    k_work_init(&movement_work, mouse_movement_handler);
    k_work_init(&click_work, mouse_click_handler);
    k_work_init(&adv_work, advertising_process);
    if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
        k_work_init(&pairing_work, pairing_process);
    }

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    advertising_start();

    configure_buttons();

    while (1) {
        k_sleep(K_SECONDS(600));
        /* Battery level simulation */
        bas_notify();
    }
}

#if MOUSE_ENABLE_TASK == 1

K_THREAD_DEFINE(mouse_task_id, 1024, mouse_task, NULL, NULL, NULL, 6, 0, 0);

#endif // MOUSE_ENABLE_TASK == 1