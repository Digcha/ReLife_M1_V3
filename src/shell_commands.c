#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <zephyr/shell/shell.h>

#include "relife_debug.h"
#include "relife_protocol.h"
#include "drivers/bma400/bma400_simple.h"
#include "drivers/fram/fram_cy15b108.h"
#include "drivers/max30101/max30101_simple.h"
#include "drivers/mlx90632/mlx90632_simple.h"
#include "drivers/rv3028/rv3028_simple.h"

static const char *bool_text(bool value)
{
    return value ? "ja" : "nein";
}

static const char *sync_state_text(uint8_t state)
{
    switch (state) {
    case RELIFE_SYNC_STATE_IDLE:
        return "idle";
    case RELIFE_SYNC_STATE_SYNCING:
        return "syncing";
    case RELIFE_SYNC_STATE_WAITING_ACK:
        return "waiting_ack";
    case RELIFE_SYNC_STATE_ERROR:
        return "error";
    default:
        return "unknown";
    }
}

static const char *error_text(uint8_t error_code)
{
    switch (error_code) {
    case RELIFE_ERROR_NONE:
        return "none";
    case RELIFE_ERROR_FRAM:
        return "fram";
    case RELIFE_ERROR_STORAGE:
        return "storage";
    case RELIFE_ERROR_RTC:
        return "rtc";
    case RELIFE_ERROR_SENSOR:
        return "sensor";
    case RELIFE_ERROR_SYNC:
        return "sync";
    case RELIFE_ERROR_PROTOCOL:
        return "protocol";
    default:
        return "unknown";
    }
}

static void print_empty_storage_hint(const struct shell *shell)
{
    if (!rv3028_has_valid_time()) {
        shell_print(shell, "Hinweis: Solange die RTC noch keine gueltige Zeit hat, werden keine Records gespeichert.");
        shell_print(shell, "Pruefe zuerst mit `zeit`, ob die Uhrzeit gueltig ist.");
    }
}

static void print_temp_centi(const struct shell *shell, const char *label, int16_t temp_centi_c)
{
    int whole = temp_centi_c / 100;
    int frac = temp_centi_c % 100;

    if (frac < 0) {
        frac = -frac;
    }

    shell_print(shell, "%s=%d.%02d C (%d centiC)", label, whole, frac, temp_centi_c);
}

static int parse_u32_arg(const struct shell *shell, const char *text, uint32_t *value)
{
    char *end = NULL;
    unsigned long parsed;

    if ((text == NULL) || (value == NULL) || (text[0] == '\0')) {
        shell_error(shell, "ungueltiger Zahlenwert");
        return -EINVAL;
    }

    parsed = strtoul(text, &end, 0);
    if ((*end != '\0') || (parsed > UINT32_MAX)) {
        shell_error(shell, "ungueltiger Zahlenwert: %s", text);
        return -EINVAL;
    }

    *value = (uint32_t)parsed;
    return 0;
}

static int load_latest_record(struct relife_record_wire *record)
{
    return relife_debug_get_latest_record(record);
}

static void print_record(const struct shell *shell, const struct relife_record_wire *record)
{
    shell_print(shell,
                "record_id=%" PRIu32 " timestamp_unix=%" PRIu32 " flags=0x%02x crc16=0x%04x",
                record->record_id,
                record->timestamp_unix,
                record->flags,
                record->crc16);

    if (record->flags & RELIFE_FLAG_HR_VALID) {
        shell_print(shell, "heart_rate=%u bpm", record->heart_rate_bpm);
    } else {
        shell_print(shell, "heart_rate=ungueltig");
    }

    if (record->flags & RELIFE_FLAG_SPO2_VALID) {
        shell_print(shell, "spo2=%u %%", record->spo2_percent);
    } else {
        shell_print(shell, "spo2=ungueltig");
    }

    if (record->flags & RELIFE_FLAG_TEMP_VALID) {
        print_temp_centi(shell, "skin_temp", record->skin_temp_centi_c);
    } else {
        shell_print(shell, "skin_temp=ungueltig");
    }

    if (record->flags & RELIFE_FLAG_STEPS_VALID) {
        shell_print(shell, "steps_total=%" PRIu32, record->steps_total);
    } else {
        shell_print(shell, "steps_total=ungueltig");
    }

    if (record->flags & RELIFE_FLAG_BATTERY_VALID) {
        shell_print(shell, "battery=%u %%", record->battery_percent);
    } else {
        shell_print(shell, "battery=ungueltig");
    }

    shell_print(shell, "rtc_valid=%s", bool_text((record->flags & RELIFE_FLAG_RTC_VALID) != 0U));
}

static void print_no_record(const struct shell *shell, int err)
{
    if (err == -ENOENT) {
        shell_print(shell, "Im FRAM ist noch kein gespeicherter Record vorhanden.");
        print_empty_storage_hint(shell);
    } else {
        shell_error(shell, "FRAM-Record konnte nicht gelesen werden (%d)", err);
    }
}

static int cmd_hilfe(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(shell, "ReLife Schnellhilfe");
    shell_print(shell, "Wichtige Commands:");
    shell_print(shell, "  status        - Gesamtstatus von Bluetooth, Sensoren und FRAM");
    shell_print(shell, "  zeit          - RTC live pruefen");
    shell_print(shell, "  zeit_setzen   - RTC setzen: zeit_setzen <unix>");
    shell_print(shell, "  schritte      - BMA400 live + letzter FRAM-Wert");
    shell_print(shell, "  temperatur    - MLX90632 live + letzter FRAM-Wert");
    shell_print(shell, "  puls          - MAX30101 live + letzter FRAM-Wert");
    shell_print(shell, "  fram_info     - FRAM-Zustand und Anzahl Records");
    shell_print(shell, "  fram_letzter  - letzten gespeicherten Record zeigen");
    shell_print(shell, "  bt_status     - BLE-Verbindung und Notify-Status");
    shell_print(shell, "  sync_status   - Sync-Zustand");
    shell_print(shell, "  sync_force    - Sync manuell starten");
    shell_print(shell, "  sync_abort    - laufenden Sync abbrechen");
    shell_print(shell, "Typischer Ablauf:");
    shell_print(shell, "  1. status");
    shell_print(shell, "  2. zeit oder zeit_setzen <unix>");
    shell_print(shell, "  3. temperatur / schritte / puls");
    shell_print(shell, "  4. fram_info");
    shell_print(shell, "  5. bt_status und sync_force");
    shell_print(shell, "Unix-Zeit = Sekunden seit 01.01.1970 UTC");
    print_empty_storage_hint(shell);

    return 0;
}

static int cmd_status(const struct shell *shell, size_t argc, char **argv)
{
    struct relife_status_wire status = { 0 };
    struct relife_bt_debug_state bt_state = { 0 };
    struct relife_storage_meta meta = { 0 };
    int meta_err;

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    relife_debug_get_status(&status);
    relife_debug_get_bt_state(&bt_state);
    meta_err = relife_debug_get_storage_meta(&meta);

    shell_print(shell,
                "Bluetooth: verbunden=%s, Datenkanal=%s, Statuskanal=%s",
                bool_text(bt_state.connected),
                bool_text(bt_state.data_notify_enabled),
                bool_text(bt_state.status_notify_enabled));
    shell_print(shell,
                "Sync: Zustand=%s, Fehler=%s, oldest=%" PRIu32 ", newest=%" PRIu32 ", bestaetigt=%" PRIu32
                ", snapshot_end=%" PRIu32,
                sync_state_text(status.state),
                error_text(status.error_code),
                status.oldest_record_id,
                status.newest_record_id,
                status.last_confirmed_id,
                status.snapshot_end_id);
    shell_print(shell,
                "Sensoren: RTC bereit=%s, RTC gueltig=%s, BMA400=%s, MAX30101=%s, MLX90632=%s, FRAM=%s",
                bool_text(rv3028_is_ready()),
                bool_text(rv3028_has_valid_time()),
                bool_text(bma400_simple_is_ready()),
                bool_text(max30101_simple_is_ready()),
                bool_text(mlx90632_simple_is_ready()),
                bool_text(fram_drv_is_ready()));

    if (meta_err == 0) {
        if (meta.count == 0U) {
            shell_print(shell, "FRAM: leer, noch kein Datensatz gespeichert.");
        } else {
            shell_print(shell,
                        "FRAM: %" PRIu32 " Record(s), head=%" PRIu32 ", tail=%" PRIu32 ", next_id=%" PRIu32,
                        meta.count,
                        meta.head_index,
                        meta.tail_index,
                        meta.next_record_id);
        }

        if (meta.last_rtc_sync_unix == 0U) {
            shell_print(shell, "RTC-Sync: noch nie gesetzt");
        } else {
            shell_print(shell, "RTC-Sync: last_rtc_sync_unix=%" PRIu32, meta.last_rtc_sync_unix);
        }
    } else {
        shell_print(shell, "FRAM-Metadaten nicht verfuegbar (%d)", meta_err);
    }

    if (!rv3028_has_valid_time()) {
        print_empty_storage_hint(shell);
    }

    return 0;
}

static int cmd_zeit(const struct shell *shell, size_t argc, char **argv)
{
    struct relife_storage_meta meta = { 0 };
    struct relife_record_wire latest = { 0 };
    bool ready = false;
    bool valid = false;
    uint32_t unix_time = 0U;
    int err;
    int latest_err;

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    err = relife_debug_read_rtc(&unix_time, &ready, &valid);
    shell_print(shell, "RTC: bereit=%s, gueltig=%s", bool_text(ready), bool_text(valid));
    if (err == 0) {
        shell_print(shell, "Live-Zeit (unix) = %" PRIu32, unix_time);
    } else {
        shell_error(shell, "RTC lesen fehlgeschlagen (%d)", err);
    }

    if (relife_debug_get_storage_meta(&meta) == 0) {
        if (meta.last_rtc_sync_unix == 0U) {
            shell_print(shell, "Gespeicherter RTC-Sync: noch nie gesetzt");
        } else {
            shell_print(shell, "Gespeicherter RTC-Sync (unix) = %" PRIu32, meta.last_rtc_sync_unix);
        }
    }

    latest_err = load_latest_record(&latest);
    if (latest_err == 0) {
        shell_print(shell,
                    "Letzter Record: id=%" PRIu32 ", timestamp=%" PRIu32 ", rtc_flag=%s",
                    latest.record_id,
                    latest.timestamp_unix,
                    bool_text((latest.flags & RELIFE_FLAG_RTC_VALID) != 0U));
    } else {
        print_no_record(shell, latest_err);
    }

    return 0;
}

static int cmd_zeit_setzen(const struct shell *shell, size_t argc, char **argv)
{
    bool ready = false;
    bool valid = false;
    uint32_t unix_time = 0U;
    uint32_t verify_time = 0U;
    int err;

    if (argc != 2) {
        shell_print(shell, "Verwendung: zeit_setzen <unix>");
        shell_print(shell, "Beispiel: zeit_setzen 1775512800");
        return -EINVAL;
    }

    err = parse_u32_arg(shell, argv[1], &unix_time);
    if (err) {
        return err;
    }

    err = relife_debug_set_rtc(unix_time);
    if (err) {
        shell_error(shell, "RTC setzen fehlgeschlagen (%d)", err);
        return err;
    }

    err = relife_debug_read_rtc(&verify_time, &ready, &valid);
    shell_print(shell, "RTC wurde gesetzt.");
    shell_print(shell, "Gesetzte Unix-Zeit = %" PRIu32, unix_time);
    if (err == 0) {
        shell_print(shell, "RTC jetzt: bereit=%s, gueltig=%s, live_unix=%" PRIu32,
                    bool_text(ready), bool_text(valid), verify_time);
    } else {
        shell_print(shell, "RTC gesetzt, Ruecklesen aber fehlgeschlagen (%d)", err);
    }

    if (valid) {
        shell_print(shell, "Hinweis: Neue Messungen koennen jetzt im FRAM gespeichert werden.");
    }

    return 0;
}

static int cmd_schritte(const struct shell *shell, size_t argc, char **argv)
{
    struct relife_record_wire latest = { 0 };
    bool ready = false;
    uint32_t steps_total = 0U;
    uint8_t activity = 0U;
    int err;
    int latest_err;

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    err = relife_debug_read_steps(&steps_total, &activity, &ready);
    shell_print(shell, "BMA400: bereit=%s", bool_text(ready));
    if (err == 0) {
        shell_print(shell, "Live: Schritte gesamt = %" PRIu32, steps_total);
        shell_print(shell, "Live: Aktivitaetswert roh = 0x%02x", activity);
    } else {
        shell_error(shell, "BMA400 lesen fehlgeschlagen (%d)", err);
    }

    latest_err = load_latest_record(&latest);
    if (latest_err == 0) {
        shell_print(shell, "Letzter Record: id=%" PRIu32, latest.record_id);
        if (latest.flags & RELIFE_FLAG_STEPS_VALID) {
            shell_print(shell, "FRAM: Schritte gesamt = %" PRIu32, latest.steps_total);
        } else {
            shell_print(shell, "FRAM: Schritte in letztem Record ungueltig");
        }
    } else {
        print_no_record(shell, latest_err);
    }

    return 0;
}

static int cmd_temperatur(const struct shell *shell, size_t argc, char **argv)
{
    struct relife_record_wire latest = { 0 };
    bool ready = false;
    int16_t temp_centi_c = 0;
    int err;
    int latest_err;

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    err = relife_debug_read_skin_temp(&temp_centi_c, &ready);
    shell_print(shell, "MLX90632: bereit=%s", bool_text(ready));
    if (err == 0) {
        print_temp_centi(shell, "Live-Temperatur", temp_centi_c);
    } else {
        shell_error(shell, "MLX90632 lesen fehlgeschlagen (%d)", err);
    }

    latest_err = load_latest_record(&latest);
    if (latest_err == 0) {
        shell_print(shell, "Letzter Record: id=%" PRIu32, latest.record_id);
        if (latest.flags & RELIFE_FLAG_TEMP_VALID) {
            print_temp_centi(shell, "FRAM-Temperatur", latest.skin_temp_centi_c);
        } else {
            shell_print(shell, "FRAM: Temperatur im letzten Record ungueltig");
        }
    } else {
        print_no_record(shell, latest_err);
    }

    return 0;
}

static int cmd_puls(const struct shell *shell, size_t argc, char **argv)
{
    struct relife_record_wire latest = { 0 };
    bool ready = false;
    bool hr_valid = false;
    bool spo2_valid = false;
    uint8_t hr = 0U;
    uint8_t spo2 = 0U;
    int err;
    int latest_err;

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(shell, "MAX30101 misst jetzt am Handgelenk. Bitte Arm und Uhr moeglichst ruhig halten...");
    err = relife_debug_read_pulse(&hr, &hr_valid, &spo2, &spo2_valid, &ready);
    shell_print(shell, "MAX30101: bereit=%s", bool_text(ready));
    if (err == 0) {
        shell_print(shell, "Hinweis: Am Handgelenk schwanken HR/SpO2 staerker als am Finger.");
        if (hr_valid) {
            shell_print(shell, "Live: Herzfrequenz = %u bpm", hr);
        } else {
            shell_print(shell, "Live: Herzfrequenz ungueltig");
        }

        if (spo2_valid) {
            shell_print(shell, "Live: SpO2 = %u %%", spo2);
        } else {
            shell_print(shell, "Live: SpO2 ungueltig");
        }
    } else {
        shell_error(shell, "MAX30101 capture fehlgeschlagen (%d)", err);
    }

    latest_err = load_latest_record(&latest);
    if (latest_err == 0) {
        shell_print(shell, "Letzter Record: id=%" PRIu32, latest.record_id);
        if (latest.flags & RELIFE_FLAG_HR_VALID) {
            shell_print(shell, "FRAM: Herzfrequenz = %u bpm", latest.heart_rate_bpm);
        } else {
            shell_print(shell, "FRAM: Herzfrequenz im letzten Record ungueltig");
        }

        if (latest.flags & RELIFE_FLAG_SPO2_VALID) {
            shell_print(shell, "FRAM: SpO2 = %u %%", latest.spo2_percent);
        } else {
            shell_print(shell, "FRAM: SpO2 im letzten Record ungueltig");
        }
    } else {
        print_no_record(shell, latest_err);
    }

    return 0;
}

static int cmd_fram_info(const struct shell *shell, size_t argc, char **argv)
{
    struct relife_storage_meta meta = { 0 };
    struct relife_status_wire status = { 0 };
    int err;

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(shell, "FRAM: bereit=%s", bool_text(fram_drv_is_ready()));

    err = relife_debug_get_storage_meta(&meta);
    relife_debug_get_status(&status);
    if (err) {
        shell_error(shell, "FRAM-Metadaten nicht verfuegbar (%d)", err);
        return 0;
    }

    if (meta.count == 0U) {
        shell_print(shell, "FRAM ist leer. Es gibt aktuell keine gespeicherten Records.");
    } else {
        shell_print(shell, "FRAM enthaelt %" PRIu32 " Record(s).", meta.count);
    }

    shell_print(shell,
                "Meta: magic=0x%08" PRIx32 ", version=0x%08" PRIx32,
                meta.magic,
                meta.version);
    shell_print(shell,
                "Ringpuffer: head=%" PRIu32 ", tail=%" PRIu32 ", count=%" PRIu32 ", next_id=%" PRIu32
                ", bestaetigt=%" PRIu32,
                meta.head_index,
                meta.tail_index,
                meta.count,
                meta.next_record_id,
                meta.last_confirmed_id);
    shell_print(shell,
                "IDs: oldest=%" PRIu32 ", newest=%" PRIu32 ", last_rtc_sync_unix=%" PRIu32,
                status.oldest_record_id,
                status.newest_record_id,
                meta.last_rtc_sync_unix);
    print_empty_storage_hint(shell);

    return 0;
}

static int cmd_fram_letzter(const struct shell *shell, size_t argc, char **argv)
{
    struct relife_record_wire record = { 0 };
    int err;

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    err = load_latest_record(&record);
    if (err) {
        print_no_record(shell, err);
        return 0;
    }

    print_record(shell, &record);
    return 0;
}

static int cmd_fram_record(const struct shell *shell, size_t argc, char **argv)
{
    struct relife_record_wire record = { 0 };
    uint32_t record_id;
    int err;

    if (argc != 2) {
        shell_error(shell, "usage: fram_record <record_id>");
        return -EINVAL;
    }

    err = parse_u32_arg(shell, argv[1], &record_id);
    if (err) {
        return err;
    }

    err = relife_debug_get_record_by_id(record_id, &record);
    if (err) {
        print_no_record(shell, err);
        return 0;
    }

    print_record(shell, &record);
    return 0;
}

static int cmd_bt_status(const struct shell *shell, size_t argc, char **argv)
{
    struct relife_status_wire status = { 0 };
    struct relife_bt_debug_state bt_state = { 0 };

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    relife_debug_get_status(&status);
    relife_debug_get_bt_state(&bt_state);

    shell_print(shell,
                "Bluetooth: verbunden=%s, Datenkanal=%s, Statuskanal=%s",
                bool_text(bt_state.connected),
                bool_text(bt_state.data_notify_enabled),
                bool_text(bt_state.status_notify_enabled));
    if (!bt_state.connected) {
        shell_print(shell, "Hinweis: Es ist aktuell kein BLE-Client verbunden.");
    } else if (!bt_state.data_notify_enabled) {
        shell_print(shell, "Hinweis: Verbunden, aber der Datenkanal ist noch nicht abonniert.");
    }
    shell_print(shell, "Sync: Zustand=%s, Fehler=%s, snapshot_end=%" PRIu32,
                sync_state_text(status.state), error_text(status.error_code), status.snapshot_end_id);

    return 0;
}

static int cmd_sync_status(const struct shell *shell, size_t argc, char **argv)
{
    struct relife_status_wire status = { 0 };
    struct relife_bt_debug_state bt_state = { 0 };

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    relife_debug_get_status(&status);
    relife_debug_get_bt_state(&bt_state);

    shell_print(shell,
                "Bluetooth: verbunden=%s, Datenkanal=%s, Statuskanal=%s",
                bool_text(bt_state.connected),
                bool_text(bt_state.data_notify_enabled),
                bool_text(bt_state.status_notify_enabled));
    shell_print(shell,
                "Sync: Zustand=%s, Fehler=%s, oldest=%" PRIu32 ", newest=%" PRIu32 ", bestaetigt=%" PRIu32
                ", snapshot_end=%" PRIu32,
                sync_state_text(status.state),
                error_text(status.error_code),
                status.oldest_record_id,
                status.newest_record_id,
                status.last_confirmed_id,
                status.snapshot_end_id);

    return 0;
}

static int cmd_sync_force(const struct shell *shell, size_t argc, char **argv)
{
    struct relife_storage_meta meta = { 0 };
    struct relife_status_wire status = { 0 };
    struct relife_bt_debug_state bt_state = { 0 };
    uint32_t after_id = 0U;
    int err;

    if (argc > 2) {
        shell_error(shell, "usage: sync_force [after_id]");
        return -EINVAL;
    }

    err = relife_debug_get_storage_meta(&meta);
    if (err) {
        shell_error(shell, "Storage-Metadaten nicht verfuegbar (%d)", err);
        return err;
    }

    if (argc == 2) {
        err = parse_u32_arg(shell, argv[1], &after_id);
        if (err) {
            return err;
        }
    } else {
        after_id = meta.last_confirmed_id;
    }

    relife_debug_get_status(&status);
    relife_debug_get_bt_state(&bt_state);

    if (!bt_state.connected) {
        shell_print(shell, "Sync nicht gestartet: kein BLE-Client verbunden.");
        return 0;
    }

    if (!bt_state.data_notify_enabled) {
        shell_print(shell, "Sync nicht gestartet: der Datenkanal ist noch nicht aktiv.");
        shell_print(shell, "Die App bzw. der BLE-Client muss zuerst Notifications fuer die Daten abonnieren.");
        return 0;
    }

    if (status.state == RELIFE_SYNC_STATE_SYNCING || status.state == RELIFE_SYNC_STATE_WAITING_ACK) {
        shell_print(shell, "Es laeuft bereits ein Sync.");
        return 0;
    }

    if (meta.count == 0U) {
        shell_print(shell, "Nichts zu synchronisieren: FRAM ist leer.");
        print_empty_storage_hint(shell);
        return 0;
    }

    if (after_id >= status.newest_record_id) {
        shell_print(shell, "Keine neueren Records verfuegbar. Es gibt aktuell nichts zu syncen.");
        return 0;
    }

    err = relife_debug_force_sync(after_id);
    if (err) {
        shell_error(shell, "Sync konnte nicht gestartet werden (%d)", err);
        return 0;
    }

    relife_debug_get_status(&status);
    shell_print(shell,
                "Sync gestartet: sende Records groesser als %" PRIu32,
                after_id);
    shell_print(shell, "Neuer Sync-Zustand: %s, snapshot_end=%" PRIu32,
                sync_state_text(status.state), status.snapshot_end_id);

    return 0;
}

static int cmd_sync_abort(const struct shell *shell, size_t argc, char **argv)
{
    struct relife_status_wire status = { 0 };

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    relife_debug_get_status(&status);
    if (status.state == RELIFE_SYNC_STATE_IDLE) {
        shell_print(shell, "Es laeuft aktuell kein Sync.");
        return 0;
    }

    relife_debug_abort_sync();
    relife_debug_get_status(&status);

    shell_print(shell,
                "Sync abgebrochen: Zustand=%s, snapshot_end=%" PRIu32,
                sync_state_text(status.state),
                status.snapshot_end_id);
    return 0;
}

SHELL_CMD_REGISTER(hilfe, NULL, "Kurze benutzerfreundliche ReLife-Hilfe", cmd_hilfe);
SHELL_CMD_REGISTER(menu, NULL, "Kurze benutzerfreundliche ReLife-Hilfe", cmd_hilfe);
SHELL_CMD_REGISTER(status, NULL, "Systemstatus mit Sensor-, BT- und FRAM-Uebersicht", cmd_status);
SHELL_CMD_REGISTER(zeit, NULL, "RTC lesen und gespeicherten Zeitstatus anzeigen", cmd_zeit);
SHELL_CMD_REGISTER(zeit_setzen, NULL, "RTC setzen: zeit_setzen <unix>", cmd_zeit_setzen);
SHELL_CMD_REGISTER(schritte, NULL, "BMA400 Schritte live lesen und gespeicherten Wert anzeigen", cmd_schritte);
SHELL_CMD_REGISTER(temperatur, NULL, "MLX90632 live lesen und gespeicherten Wert anzeigen", cmd_temperatur);
SHELL_CMD_REGISTER(puls, NULL, "MAX30101 live messen und gespeicherte HR/SpO2 anzeigen", cmd_puls);
SHELL_CMD_REGISTER(fram_info, NULL, "FRAM-Metadaten und Ringpufferstatus anzeigen", cmd_fram_info);
SHELL_CMD_REGISTER(fram_letzter, NULL, "Letzten gespeicherten FRAM-Record anzeigen", cmd_fram_letzter);
SHELL_CMD_REGISTER(fram_record, NULL, "Einen bestimmten FRAM-Record lesen: fram_record <record_id>", cmd_fram_record);
SHELL_CMD_REGISTER(bt_status, NULL, "Bluetooth- und Notify-Status anzeigen", cmd_bt_status);
SHELL_CMD_REGISTER(sync_status, NULL, "Aktuellen Sync-Status anzeigen", cmd_sync_status);
SHELL_CMD_REGISTER(sync_force, NULL, "Sync sofort starten: sync_force [after_id]", cmd_sync_force);
SHELL_CMD_REGISTER(sync_abort, NULL, "Aktiven Sync abbrechen", cmd_sync_abort);
