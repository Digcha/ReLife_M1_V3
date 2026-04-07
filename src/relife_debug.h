#ifndef RELIFE_DEBUG_H
#define RELIFE_DEBUG_H

#include <stdbool.h>
#include <stdint.h>

#include "relife_protocol.h"

struct relife_bt_debug_state {
    bool connected;
    bool data_notify_enabled;
    bool status_notify_enabled;
};

int relife_debug_get_storage_meta(struct relife_storage_meta *meta);
int relife_debug_get_latest_record(struct relife_record_wire *record);
int relife_debug_get_record_by_id(uint32_t record_id, struct relife_record_wire *record);
void relife_debug_get_status(struct relife_status_wire *status);
void relife_debug_get_bt_state(struct relife_bt_debug_state *state);

int relife_debug_read_rtc(uint32_t *unix_time, bool *ready, bool *valid_time);
int relife_debug_set_rtc(uint32_t unix_time);
int relife_debug_read_steps(uint32_t *steps_total, uint8_t *activity, bool *ready);
int relife_debug_read_skin_temp(int16_t *temp_centi_c, bool *ready);
int relife_debug_read_pulse(uint8_t *heart_rate_bpm, bool *hr_valid, uint8_t *spo2_percent, bool *spo2_valid,
                            bool *ready);

int relife_debug_force_sync(uint32_t after_id);
int relife_debug_abort_sync(void);

#endif
