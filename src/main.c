#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <psa/crypto.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "relife_protocol.h"
#include "relife_debug.h"
#include "drivers/bma400/bma400_simple.h"
#include "drivers/fram/fram_cy15b108.h"
#include "drivers/max30101/max30101_simple.h"
#include "drivers/mlx90632/mlx90632_simple.h"
#include "drivers/power/board_power.h"
#include "drivers/rv3028/rv3028_simple.h"

LOG_MODULE_REGISTER(relife_m1_v3, LOG_LEVEL_INF);

extern const struct bt_gatt_service_static relife_svc;

#define BT_UUID_RELIFE_SERVICE_VAL BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789ABCDEF0ULL)
#define BT_UUID_RELIFE_DATA_VAL    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789ABCDEF1ULL)
#define BT_UUID_RELIFE_CTRL_VAL    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789ABCDEF2ULL)
#define BT_UUID_RELIFE_STATUS_VAL  BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789ABCDEF3ULL)

static struct bt_uuid_128 g_service_uuid = BT_UUID_INIT_128(BT_UUID_RELIFE_SERVICE_VAL);
static struct bt_uuid_128 g_data_uuid = BT_UUID_INIT_128(BT_UUID_RELIFE_DATA_VAL);
static struct bt_uuid_128 g_ctrl_uuid = BT_UUID_INIT_128(BT_UUID_RELIFE_CTRL_VAL);
static struct bt_uuid_128 g_status_uuid = BT_UUID_INIT_128(BT_UUID_RELIFE_STATUS_VAL);

static struct bt_conn *g_current_conn;
static bool g_data_notify_enabled;
static bool g_status_notify_enabled;

static struct relife_storage_meta g_storage_meta;
static struct relife_status_wire g_status_packet;

static bool g_storage_ready;
static enum relife_sync_state g_sync_state = RELIFE_SYNC_STATE_IDLE;
static enum relife_error_code g_error_code = RELIFE_ERROR_NONE;
static uint32_t g_snapshot_end_id;
static uint32_t g_sync_start_after_id;
static bool g_sync_abort_requested;

static K_MUTEX_DEFINE(g_storage_lock);
static K_MUTEX_DEFINE(g_bt_lock);
static K_MUTEX_DEFINE(g_sensor_lock);
static K_SEM_DEFINE(g_system_ready_sem, 0, 1);
static K_SEM_DEFINE(g_sync_sem, 0, 1);

static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static bool relife_record_crc_valid(const struct relife_record_wire *record)
{
    const uint16_t expected = crc16_ccitt_false((const uint8_t *)record, sizeof(*record) - sizeof(record->crc16));

    return record->crc16 == expected;
}

static void relife_finalize_record(struct relife_record_wire *record)
{
    record->crc16 = crc16_ccitt_false((const uint8_t *)record, sizeof(*record) - sizeof(record->crc16));
}

static uint32_t storage_slot_address(uint32_t slot_index)
{
    return RELIFE_FRAM_RECORDS_ADDR + slot_index * RELIFE_RECORD_SIZE;
}

static uint32_t storage_write_pointer(void)
{
    return storage_slot_address(g_storage_meta.tail_index);
}

static uint32_t storage_oldest_record_id_locked(void)
{
    return g_storage_meta.count > 0U ? (g_storage_meta.last_confirmed_id + 1U) : 0U;
}

static uint32_t storage_newest_record_id_locked(void)
{
    return g_storage_meta.count > 0U ? (g_storage_meta.next_record_id - 1U) : 0U;
}

static bool storage_meta_valid(uint32_t raw_write_pointer)
{
    if (g_storage_meta.magic != RELIFE_STORAGE_MAGIC ||
        g_storage_meta.version != RELIFE_STORAGE_VERSION ||
        g_storage_meta.head_index >= RELIFE_MAX_RECORDS ||
        g_storage_meta.tail_index >= RELIFE_MAX_RECORDS ||
        g_storage_meta.count > RELIFE_MAX_RECORDS ||
        g_storage_meta.next_record_id == 0U) {
        return false;
    }

    if (((g_storage_meta.head_index + g_storage_meta.count) % RELIFE_MAX_RECORDS) != g_storage_meta.tail_index) {
        return false;
    }

    if (g_storage_meta.next_record_id != (g_storage_meta.last_confirmed_id + g_storage_meta.count + 1U)) {
        return false;
    }

    if (raw_write_pointer != storage_write_pointer()) {
        return false;
    }

    return true;
}

static int storage_persist_locked(void)
{
    const uint32_t write_pointer = storage_write_pointer();
    int err;

    err = fram_drv_write(RELIFE_FRAM_WRITE_POINTER_ADDR, (const uint8_t *)&write_pointer, sizeof(write_pointer));
    if (err) {
        return err;
    }

    return fram_drv_write(RELIFE_FRAM_METADATA_ADDR, (const uint8_t *)&g_storage_meta, sizeof(g_storage_meta));
}

static int storage_reset_locked(void)
{
    memset(&g_storage_meta, 0, sizeof(g_storage_meta));
    g_storage_meta.magic = RELIFE_STORAGE_MAGIC;
    g_storage_meta.version = RELIFE_STORAGE_VERSION;
    g_storage_meta.next_record_id = 1U;
    return storage_persist_locked();
}

static int storage_init(void)
{
    uint32_t raw_write_pointer = 0U;
    int err;

    err = fram_drv_init();
    if (err) {
        return err;
    }

    k_mutex_lock(&g_storage_lock, K_FOREVER);

    err = fram_drv_read(RELIFE_FRAM_METADATA_ADDR, (uint8_t *)&g_storage_meta, sizeof(g_storage_meta));
    if (err) {
        err = storage_reset_locked();
        goto out;
    }

    err = fram_drv_read(RELIFE_FRAM_WRITE_POINTER_ADDR, (uint8_t *)&raw_write_pointer, sizeof(raw_write_pointer));
    if (err || !storage_meta_valid(raw_write_pointer)) {
        err = storage_reset_locked();
        goto out;
    }

out:
    g_storage_ready = (err == 0);
    k_mutex_unlock(&g_storage_lock);
    return err;
}

static int storage_read_record_by_id(uint32_t record_id, struct relife_record_wire *record)
{
    uint32_t first_id;
    uint32_t offset;
    uint32_t slot_index;
    int err;

    if ((record == NULL) || !g_storage_ready) {
        return -EINVAL;
    }

    k_mutex_lock(&g_storage_lock, K_FOREVER);

    first_id = storage_oldest_record_id_locked();
    if ((g_storage_meta.count == 0U) || (record_id < first_id) || (record_id > storage_newest_record_id_locked())) {
        err = -ENOENT;
        goto out;
    }

    offset = record_id - first_id;
    slot_index = (g_storage_meta.head_index + offset) % RELIFE_MAX_RECORDS;
    err = fram_drv_read(storage_slot_address(slot_index), (uint8_t *)record, sizeof(*record));
    if (err) {
        goto out;
    }

    if ((record->record_id != record_id) || !relife_record_crc_valid(record)) {
        err = -EIO;
        goto out;
    }

out:
    k_mutex_unlock(&g_storage_lock);
    return err;
}

static int storage_append_record(struct relife_record_wire *record)
{
    struct relife_record_wire verify = { 0 };
    uint32_t slot_index;
    int err;

    if ((record == NULL) || !g_storage_ready) {
        return -EINVAL;
    }

    k_mutex_lock(&g_storage_lock, K_FOREVER);

    if (g_storage_meta.count >= RELIFE_MAX_RECORDS) {
        err = -ENOSPC;
        goto out;
    }

    record->record_id = g_storage_meta.next_record_id;
    relife_finalize_record(record);

    slot_index = g_storage_meta.tail_index;
    err = fram_drv_write(storage_slot_address(slot_index), (const uint8_t *)record, sizeof(*record));
    if (err) {
        goto out;
    }

    err = fram_drv_read(storage_slot_address(slot_index), (uint8_t *)&verify, sizeof(verify));
    if (err) {
        goto out;
    }

    if (memcmp(record, &verify, sizeof(*record)) != 0) {
        err = -EIO;
        goto out;
    }

    g_storage_meta.tail_index = (g_storage_meta.tail_index + 1U) % RELIFE_MAX_RECORDS;
    g_storage_meta.count++;
    g_storage_meta.next_record_id++;

    err = storage_persist_locked();

out:
    k_mutex_unlock(&g_storage_lock);
    return err;
}

static int storage_ack_up_to(uint32_t ack_id)
{
    uint32_t highest_ackable;
    uint32_t remove_count;
    int err = 0;

    if (!g_storage_ready) {
        return -ENODEV;
    }

    k_mutex_lock(&g_storage_lock, K_FOREVER);

    highest_ackable = storage_newest_record_id_locked();
    if (g_snapshot_end_id != 0U) {
        highest_ackable = MIN(highest_ackable, g_snapshot_end_id);
    }

    if ((g_storage_meta.count == 0U) || (ack_id <= g_storage_meta.last_confirmed_id)) {
        goto out;
    }

    ack_id = MIN(ack_id, highest_ackable);
    if (ack_id <= g_storage_meta.last_confirmed_id) {
        goto out;
    }

    remove_count = ack_id - g_storage_meta.last_confirmed_id;
    remove_count = MIN(remove_count, g_storage_meta.count);

    g_storage_meta.head_index = (g_storage_meta.head_index + remove_count) % RELIFE_MAX_RECORDS;
    g_storage_meta.count -= remove_count;
    g_storage_meta.last_confirmed_id += remove_count;

    err = storage_persist_locked();

out:
    k_mutex_unlock(&g_storage_lock);
    return err;
}

static void relife_refresh_status_packet(void)
{
    uint32_t oldest = 0U;
    uint32_t newest = 0U;
    uint32_t last_confirmed = 0U;

    k_mutex_lock(&g_storage_lock, K_FOREVER);
    if (g_storage_ready) {
        oldest = storage_oldest_record_id_locked();
        newest = storage_newest_record_id_locked();
        last_confirmed = g_storage_meta.last_confirmed_id;
    }
    k_mutex_unlock(&g_storage_lock);

    memset(&g_status_packet, 0, sizeof(g_status_packet));
    g_status_packet.protocol_version = RELIFE_PROTOCOL_VERSION;
    g_status_packet.state = (uint8_t)g_sync_state;
    g_status_packet.error_code = (uint8_t)g_error_code;
    g_status_packet.oldest_record_id = oldest;
    g_status_packet.newest_record_id = newest;
    g_status_packet.last_confirmed_id = last_confirmed;
    g_status_packet.snapshot_end_id = g_snapshot_end_id;
}

static struct bt_conn *bt_conn_get_ref(void)
{
    struct bt_conn *conn = NULL;

    k_mutex_lock(&g_bt_lock, K_FOREVER);
    if (g_current_conn != NULL) {
        conn = bt_conn_ref(g_current_conn);
    }
    k_mutex_unlock(&g_bt_lock);

    return conn;
}

static int relife_notify_status(void)
{
    struct bt_conn *conn;
    int err;

    if (!g_status_notify_enabled) {
        return 0;
    }

    conn = bt_conn_get_ref();
    if (conn == NULL) {
        return -ENOTCONN;
    }

    relife_refresh_status_packet();
    err = bt_gatt_notify(conn, &relife_svc.attrs[7], &g_status_packet, sizeof(g_status_packet));
    bt_conn_unref(conn);
    return err;
}

static void relife_set_error(enum relife_error_code error_code)
{
    g_error_code = error_code;
    if (error_code != RELIFE_ERROR_NONE) {
        g_sync_state = RELIFE_SYNC_STATE_ERROR;
    }
    relife_notify_status();
}

static ssize_t relife_read_status(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                  void *buf, uint16_t len, uint16_t offset)
{
    relife_refresh_status_packet();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &g_status_packet, sizeof(g_status_packet));
}

static void data_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    g_data_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    g_status_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    if (g_status_notify_enabled) {
        relife_notify_status();
    }
}

static bool decode_command(const void *buf, uint16_t len, uint8_t *opcode, uint32_t *value)
{
    const uint8_t *raw = buf;

    if ((buf == NULL) || (len == 0U) || (opcode == NULL) || (value == NULL)) {
        return false;
    }

    *opcode = raw[0];
    *value = 0U;

    if (len >= sizeof(struct relife_command_wire)) {
        *value = sys_get_le32(&raw[4]);
        return true;
    }

    if (len >= 5U) {
        *value = sys_get_le32(&raw[1]);
        return true;
    }

    return (len == 1U);
}

static int start_sync_session(uint32_t last_local_id)
{
    uint32_t newest_id = 0U;
    struct bt_conn *conn = NULL;

    k_mutex_lock(&g_storage_lock, K_FOREVER);
    if (!g_storage_ready) {
        k_mutex_unlock(&g_storage_lock);
        relife_set_error(RELIFE_ERROR_STORAGE);
        return -ENODEV;
    }
    newest_id = storage_newest_record_id_locked();
    k_mutex_unlock(&g_storage_lock);

    if (!g_data_notify_enabled) {
        relife_set_error(RELIFE_ERROR_SYNC);
        return -EACCES;
    }

    conn = bt_conn_get_ref();
    if (conn == NULL) {
        relife_set_error(RELIFE_ERROR_SYNC);
        return -ENOTCONN;
    }
    bt_conn_unref(conn);

    if ((newest_id == 0U) || (last_local_id >= newest_id)) {
        g_snapshot_end_id = 0U;
        g_sync_state = RELIFE_SYNC_STATE_IDLE;
        g_error_code = RELIFE_ERROR_NONE;
        relife_notify_status();
        return 0;
    }

    g_sync_start_after_id = last_local_id;
    g_snapshot_end_id = newest_id;
    g_sync_abort_requested = false;
    g_sync_state = RELIFE_SYNC_STATE_SYNCING;
    g_error_code = RELIFE_ERROR_NONE;
    relife_notify_status();
    k_sem_give(&g_sync_sem);
    return 0;
}

static void abort_sync_session(void)
{
    g_sync_abort_requested = true;
    g_snapshot_end_id = 0U;
    g_sync_state = RELIFE_SYNC_STATE_IDLE;
    g_error_code = RELIFE_ERROR_NONE;
    relife_notify_status();
}

static ssize_t relife_control_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    uint8_t opcode = 0U;
    uint32_t value = 0U;
    int err = 0;

    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

    if (offset != 0U) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (!decode_command(buf, len, &opcode, &value)) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    switch (opcode) {
    case RELIFE_OPCODE_GET_STATUS:
        relife_refresh_status_packet();
        relife_notify_status();
        break;

    case RELIFE_OPCODE_START_SYNC:
        err = start_sync_session(value);
        if (err) {
            LOG_WRN("START_SYNC failed (%d)", err);
        }
        break;

    case RELIFE_OPCODE_ACK_UP_TO:
        if (g_sync_state == RELIFE_SYNC_STATE_SYNCING) {
            relife_set_error(RELIFE_ERROR_PROTOCOL);
            break;
        }

        err = storage_ack_up_to(value);
        if (err) {
            relife_set_error(RELIFE_ERROR_STORAGE);
        } else {
            g_snapshot_end_id = 0U;
            g_sync_state = RELIFE_SYNC_STATE_IDLE;
            g_error_code = RELIFE_ERROR_NONE;
            relife_notify_status();
        }
        break;

    case RELIFE_OPCODE_ABORT_SYNC:
        abort_sync_session();
        break;

    case RELIFE_OPCODE_RTC_SYNC:
        err = rv3028_set_unix_time(value);
        if (err) {
            relife_set_error(RELIFE_ERROR_RTC);
        } else {
            k_mutex_lock(&g_storage_lock, K_FOREVER);
            g_storage_meta.last_rtc_sync_unix = value;
            if (g_storage_ready) {
                storage_persist_locked();
            }
            k_mutex_unlock(&g_storage_lock);
            g_error_code = RELIFE_ERROR_NONE;
            relife_notify_status();
        }
        break;

    default:
        relife_set_error(RELIFE_ERROR_PROTOCOL);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    return len;
}

BT_GATT_SERVICE_DEFINE(relife_svc,
    BT_GATT_PRIMARY_SERVICE(&g_service_uuid),
    BT_GATT_CHARACTERISTIC(&g_data_uuid.uuid,
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, NULL),
    BT_GATT_CCC(data_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&g_ctrl_uuid.uuid,
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE,
        NULL, relife_control_write, NULL),
    BT_GATT_CHARACTERISTIC(&g_status_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        relife_read_status, NULL, NULL),
    BT_GATT_CCC(status_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, RELIFE_DEVICE_NAME, sizeof(RELIFE_DEVICE_NAME) - 1),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_RELIFE_SERVICE_VAL),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connect failed (%u)", err);
        return;
    }

    k_mutex_lock(&g_bt_lock, K_FOREVER);
    if (g_current_conn != NULL) {
        bt_conn_unref(g_current_conn);
    }
    g_current_conn = bt_conn_ref(conn);
    k_mutex_unlock(&g_bt_lock);

    g_sync_state = RELIFE_SYNC_STATE_IDLE;
    g_snapshot_end_id = 0U;
    g_sync_abort_requested = false;
    LOG_INF("Connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);
    LOG_INF("Disconnected (0x%02x)", reason);

    k_mutex_lock(&g_bt_lock, K_FOREVER);
    if (g_current_conn != NULL) {
        bt_conn_unref(g_current_conn);
        g_current_conn = NULL;
    }
    k_mutex_unlock(&g_bt_lock);

    g_data_notify_enabled = false;
    g_status_notify_enabled = false;
    g_sync_abort_requested = true;
    g_sync_state = RELIFE_SYNC_STATE_IDLE;
    g_snapshot_end_id = 0U;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void sync_thread_fn(void *, void *, void *)
{
    for (;;) {
        uint32_t first_id;
        uint32_t last_id;
        struct bt_conn *conn;

        k_sem_take(&g_sync_sem, K_FOREVER);

        if (g_sync_state != RELIFE_SYNC_STATE_SYNCING) {
            continue;
        }

        conn = bt_conn_get_ref();
        if (conn == NULL) {
            relife_set_error(RELIFE_ERROR_SYNC);
            continue;
        }

        k_mutex_lock(&g_storage_lock, K_FOREVER);
        first_id = MAX(g_sync_start_after_id + 1U, storage_oldest_record_id_locked());
        last_id = MIN(g_snapshot_end_id, storage_newest_record_id_locked());
        k_mutex_unlock(&g_storage_lock);

        if ((last_id == 0U) || (first_id > last_id)) {
            g_snapshot_end_id = 0U;
            g_sync_state = RELIFE_SYNC_STATE_IDLE;
            g_error_code = RELIFE_ERROR_NONE;
            bt_conn_unref(conn);
            relife_notify_status();
            continue;
        }

        for (uint32_t record_id = first_id; record_id <= last_id; record_id++) {
            struct relife_record_wire record = { 0 };
            int err;

            if (g_sync_abort_requested) {
                g_sync_state = RELIFE_SYNC_STATE_IDLE;
                g_snapshot_end_id = 0U;
                break;
            }

            err = storage_read_record_by_id(record_id, &record);
            if (err) {
                relife_set_error(RELIFE_ERROR_STORAGE);
                break;
            }

            err = bt_gatt_notify(conn, &relife_svc.attrs[2], &record, sizeof(record));
            if (err) {
                relife_set_error(RELIFE_ERROR_SYNC);
                break;
            }

            k_msleep(10);
        }

        if ((g_sync_state == RELIFE_SYNC_STATE_SYNCING) && !g_sync_abort_requested) {
            g_sync_state = RELIFE_SYNC_STATE_WAITING_ACK;
            g_error_code = RELIFE_ERROR_NONE;
            relife_notify_status();
        }

        bt_conn_unref(conn);
    }
}

static void measurement_thread_fn(void *, void *, void *)
{
    k_sem_take(&g_system_ready_sem, K_FOREVER);
    k_sleep(K_SECONDS(RELIFE_FIRST_MEASUREMENT_DELAY_SEC));

    for (;;) {
        struct relife_record_wire record = {
            .heart_rate_bpm = 0xFFU,
            .spo2_percent = 0xFFU,
            .skin_temp_centi_c = INT16_MIN,
            .battery_percent = 0xFFU,
            .flags = 0U,
        };
        uint32_t timestamp = 0U;
        uint32_t steps_total = 0U;
        uint8_t activity = 0U;
        uint8_t hr = 0xFFU;
        uint8_t spo2 = 0xFFU;
        bool hr_valid = false;
        bool spo2_valid = false;
        int16_t temp_centi = INT16_MIN;
        bool any_value = false;

        if (g_sync_state == RELIFE_SYNC_STATE_SYNCING) {
            k_sleep(K_SECONDS(5));
            continue;
        }

        if (!g_storage_ready || !rv3028_is_ready() || !rv3028_has_valid_time()) {
            k_sleep(K_SECONDS(15));
            continue;
        }

        k_mutex_lock(&g_sensor_lock, K_FOREVER);

        if (rv3028_get_unix_time(&timestamp)) {
            k_mutex_unlock(&g_sensor_lock);
            relife_set_error(RELIFE_ERROR_RTC);
            k_sleep(K_SECONDS(15));
            continue;
        }

        record.timestamp_unix = timestamp;
        record.flags |= RELIFE_FLAG_RTC_VALID;

        if (!bma400_simple_get_steps(&steps_total, &activity)) {
            ARG_UNUSED(activity);
            record.steps_total = steps_total;
            record.flags |= RELIFE_FLAG_STEPS_VALID;
            any_value = true;
        }

        if (!mlx90632_simple_read_centi_degrees(&temp_centi)) {
            record.skin_temp_centi_c = temp_centi;
            record.flags |= RELIFE_FLAG_TEMP_VALID;
            any_value = true;
        }

        if (!max30101_simple_capture(&hr, &hr_valid, &spo2, &spo2_valid)) {
            if (hr_valid) {
                record.heart_rate_bpm = hr;
                record.flags |= RELIFE_FLAG_HR_VALID;
                any_value = true;
            }
            if (spo2_valid) {
                record.spo2_percent = spo2;
                record.flags |= RELIFE_FLAG_SPO2_VALID;
                any_value = true;
            }
        }

        k_mutex_unlock(&g_sensor_lock);

        if (!any_value) {
            relife_set_error(RELIFE_ERROR_SENSOR);
            k_sleep(K_SECONDS(RELIFE_MEASUREMENT_INTERVAL_SEC));
            continue;
        }

        if (storage_append_record(&record)) {
            relife_set_error(RELIFE_ERROR_STORAGE);
        } else {
            g_error_code = RELIFE_ERROR_NONE;
            relife_refresh_status_packet();
            relife_notify_status();
        }

        k_sleep(K_SECONDS(RELIFE_MEASUREMENT_INTERVAL_SEC));
    }
}

K_THREAD_DEFINE(sync_thread_id, 4096, sync_thread_fn, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(measurement_thread_id, 6144, measurement_thread_fn, NULL, NULL, NULL, 7, 0, 0);

static int relife_bt_start(void)
{
    const struct bt_le_adv_param *adv_param =
        BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE,
                        BT_GAP_ADV_FAST_INT_MIN_2,
                        BT_GAP_ADV_FAST_INT_MAX_2,
                        NULL);
    psa_status_t psa_err;
    int err;

    psa_err = psa_crypto_init();
    if (psa_err != PSA_SUCCESS) {
        return -EIO;
    }

    err = bt_enable(NULL);
    if (err) {
        return err;
    }

    return bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
}

static void relife_system_init(void)
{
    int err;

    err = board_power_init();
    if (!err) {
        err = board_power_enable_sensor_rails();
    }
    if (err) {
        relife_set_error(RELIFE_ERROR_SENSOR);
    }

    err = storage_init();
    if (err) {
        relife_set_error(RELIFE_ERROR_FRAM);
    }

    if (rv3028_init()) {
        relife_set_error(RELIFE_ERROR_RTC);
    }
    if (bma400_simple_init()) {
        relife_set_error(RELIFE_ERROR_SENSOR);
    }
    if (max30101_simple_init()) {
        relife_set_error(RELIFE_ERROR_SENSOR);
    }
    if (mlx90632_simple_init()) {
        relife_set_error(RELIFE_ERROR_SENSOR);
    }

    relife_refresh_status_packet();
    relife_notify_status();
    k_sem_give(&g_system_ready_sem);
}

int main(void)
{
    int err;

    LOG_INF("ReLife_M1_V3 boot");

    err = relife_bt_start();
    if (err) {
        LOG_ERR("Bluetooth start failed (%d)", err);
        return err;
    }

    LOG_INF("Advertising as %s", RELIFE_DEVICE_NAME);
    relife_system_init();
    return 0;
}

int relife_debug_get_storage_meta(struct relife_storage_meta *meta)
{
    int err = 0;

    if (meta == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&g_storage_lock, K_FOREVER);
    if (!g_storage_ready) {
        err = -ENODEV;
    } else {
        *meta = g_storage_meta;
    }
    k_mutex_unlock(&g_storage_lock);

    return err;
}

int relife_debug_get_latest_record(struct relife_record_wire *record)
{
    uint32_t newest_id;

    if (record == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&g_storage_lock, K_FOREVER);
    if (!g_storage_ready) {
        k_mutex_unlock(&g_storage_lock);
        return -ENODEV;
    }

    if (g_storage_meta.count == 0U) {
        k_mutex_unlock(&g_storage_lock);
        return -ENOENT;
    }

    newest_id = storage_newest_record_id_locked();
    k_mutex_unlock(&g_storage_lock);

    return storage_read_record_by_id(newest_id, record);
}

int relife_debug_get_record_by_id(uint32_t record_id, struct relife_record_wire *record)
{
    return storage_read_record_by_id(record_id, record);
}

void relife_debug_get_status(struct relife_status_wire *status)
{
    if (status == NULL) {
        return;
    }

    relife_refresh_status_packet();
    *status = g_status_packet;
}

void relife_debug_get_bt_state(struct relife_bt_debug_state *state)
{
    if (state == NULL) {
        return;
    }

    k_mutex_lock(&g_bt_lock, K_FOREVER);
    state->connected = (g_current_conn != NULL);
    state->data_notify_enabled = g_data_notify_enabled;
    state->status_notify_enabled = g_status_notify_enabled;
    k_mutex_unlock(&g_bt_lock);
}

int relife_debug_read_rtc(uint32_t *unix_time, bool *ready, bool *valid_time)
{
    int err;

    if (ready != NULL) {
        *ready = rv3028_is_ready();
    }
    if (valid_time != NULL) {
        *valid_time = rv3028_has_valid_time();
    }
    if (unix_time == NULL) {
        return -EINVAL;
    }
    if (!rv3028_is_ready()) {
        return -ENODEV;
    }

    k_mutex_lock(&g_sensor_lock, K_FOREVER);
    err = rv3028_get_unix_time(unix_time);
    k_mutex_unlock(&g_sensor_lock);

    return err;
}

int relife_debug_set_rtc(uint32_t unix_time)
{
    int err;

    if (!rv3028_is_ready()) {
        return -ENODEV;
    }

    k_mutex_lock(&g_sensor_lock, K_FOREVER);
    err = rv3028_set_unix_time(unix_time);
    k_mutex_unlock(&g_sensor_lock);
    if (err) {
        relife_set_error(RELIFE_ERROR_RTC);
        return err;
    }

    k_mutex_lock(&g_storage_lock, K_FOREVER);
    g_storage_meta.last_rtc_sync_unix = unix_time;
    if (g_storage_ready) {
        err = storage_persist_locked();
    } else {
        err = 0;
    }
    k_mutex_unlock(&g_storage_lock);
    if (err) {
        relife_set_error(RELIFE_ERROR_STORAGE);
        return err;
    }

    g_error_code = RELIFE_ERROR_NONE;
    relife_notify_status();
    return 0;
}

int relife_debug_read_steps(uint32_t *steps_total, uint8_t *activity, bool *ready)
{
    int err;

    if ((steps_total == NULL) || (activity == NULL)) {
        return -EINVAL;
    }
    if (ready != NULL) {
        *ready = bma400_simple_is_ready();
    }
    if (!bma400_simple_is_ready()) {
        return -ENODEV;
    }

    k_mutex_lock(&g_sensor_lock, K_FOREVER);
    err = bma400_simple_get_steps(steps_total, activity);
    k_mutex_unlock(&g_sensor_lock);

    return err;
}

int relife_debug_read_skin_temp(int16_t *temp_centi_c, bool *ready)
{
    int err;

    if (temp_centi_c == NULL) {
        return -EINVAL;
    }
    if (ready != NULL) {
        *ready = mlx90632_simple_is_ready();
    }
    if (!mlx90632_simple_is_ready()) {
        return -ENODEV;
    }

    k_mutex_lock(&g_sensor_lock, K_FOREVER);
    err = mlx90632_simple_read_centi_degrees(temp_centi_c);
    k_mutex_unlock(&g_sensor_lock);

    return err;
}

int relife_debug_read_pulse(uint8_t *heart_rate_bpm, bool *hr_valid, uint8_t *spo2_percent, bool *spo2_valid,
                            bool *ready)
{
    int err;

    if ((heart_rate_bpm == NULL) || (hr_valid == NULL) || (spo2_percent == NULL) || (spo2_valid == NULL)) {
        return -EINVAL;
    }
    if (ready != NULL) {
        *ready = max30101_simple_is_ready();
    }
    if (!max30101_simple_is_ready()) {
        return -ENODEV;
    }

    k_mutex_lock(&g_sensor_lock, K_FOREVER);
    err = max30101_simple_capture(heart_rate_bpm, hr_valid, spo2_percent, spo2_valid);
    k_mutex_unlock(&g_sensor_lock);

    return err;
}

int relife_debug_force_sync(uint32_t after_id)
{
    return start_sync_session(after_id);
}

int relife_debug_abort_sync(void)
{
    abort_sync_session();
    return 0;
}
