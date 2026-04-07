#ifndef RELIFE_PROTOCOL_H
#define RELIFE_PROTOCOL_H

#include <stdint.h>

#include <zephyr/sys/util.h>

#define RELIFE_PROTOCOL_VERSION              3U
#define RELIFE_DEVICE_NAME                   "ReLife M1"

#define RELIFE_FRAM_CAPACITY                 1048576UL
#define RELIFE_FRAM_WRITE_POINTER_ADDR       0x00000UL
#define RELIFE_FRAM_METADATA_ADDR            0x00004UL
#define RELIFE_FRAM_RECORDS_ADDR             0x00040UL

#define RELIFE_STORAGE_MAGIC                 0x33464C52UL
#define RELIFE_STORAGE_VERSION               0x00030000UL

#define RELIFE_RECORD_SIZE                   20U
#define RELIFE_MAX_RECORDS                   ((RELIFE_FRAM_CAPACITY - RELIFE_FRAM_RECORDS_ADDR) / RELIFE_RECORD_SIZE)

#define RELIFE_MEASUREMENT_INTERVAL_SEC      60U
#define RELIFE_FIRST_MEASUREMENT_DELAY_SEC   5U
#define RELIFE_PPG_CAPTURE_MS                8000U
#define RELIFE_SENSOR_STABILIZE_MS           500U

enum relife_opcode {
    RELIFE_OPCODE_GET_STATUS = 0x01,
    RELIFE_OPCODE_START_SYNC = 0x02,
    RELIFE_OPCODE_ACK_UP_TO  = 0x03,
    RELIFE_OPCODE_ABORT_SYNC = 0x04,
    RELIFE_OPCODE_RTC_SYNC   = 0x05,
};

enum relife_sync_state {
    RELIFE_SYNC_STATE_IDLE = 0,
    RELIFE_SYNC_STATE_SYNCING = 1,
    RELIFE_SYNC_STATE_WAITING_ACK = 2,
    RELIFE_SYNC_STATE_ERROR = 3,
};

enum relife_error_code {
    RELIFE_ERROR_NONE = 0,
    RELIFE_ERROR_FRAM = 1,
    RELIFE_ERROR_STORAGE = 2,
    RELIFE_ERROR_RTC = 3,
    RELIFE_ERROR_SENSOR = 4,
    RELIFE_ERROR_SYNC = 5,
    RELIFE_ERROR_PROTOCOL = 6,
};

enum relife_record_flags {
    RELIFE_FLAG_HR_VALID = BIT(0),
    RELIFE_FLAG_SPO2_VALID = BIT(1),
    RELIFE_FLAG_TEMP_VALID = BIT(2),
    RELIFE_FLAG_STEPS_VALID = BIT(3),
    RELIFE_FLAG_BATTERY_VALID = BIT(4),
    RELIFE_FLAG_RTC_VALID = BIT(5),
};

struct relife_command_wire {
    uint8_t opcode;
    uint8_t reserved[3];
    uint32_t value;
} __packed;

struct relife_record_wire {
    uint32_t record_id;
    uint32_t timestamp_unix;
    uint8_t heart_rate_bpm;
    uint8_t spo2_percent;
    int16_t skin_temp_centi_c;
    uint32_t steps_total;
    uint8_t battery_percent;
    uint8_t flags;
    uint16_t crc16;
} __packed;

struct relife_status_wire {
    uint8_t protocol_version;
    uint8_t state;
    uint8_t error_code;
    uint8_t reserved;
    uint32_t oldest_record_id;
    uint32_t newest_record_id;
    uint32_t last_confirmed_id;
    uint32_t snapshot_end_id;
} __packed;

struct relife_storage_meta {
    uint32_t magic;
    uint32_t version;
    uint32_t head_index;
    uint32_t tail_index;
    uint32_t count;
    uint32_t next_record_id;
    uint32_t last_confirmed_id;
    uint32_t last_rtc_sync_unix;
};

BUILD_ASSERT(sizeof(struct relife_command_wire) == 8, "Command packet must stay fixed");
BUILD_ASSERT(sizeof(struct relife_record_wire) == RELIFE_RECORD_SIZE, "Record packet must fit one ATT notify");
BUILD_ASSERT(sizeof(struct relife_status_wire) == 20, "Status packet must stay fixed");

#endif
