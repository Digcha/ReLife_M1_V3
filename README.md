# ReLife_M1_V3

`ReLife_M1_V3` ist neu aus den funktionierenden Referenzpfaden aufgebaut und nutzt **nicht** `ReLife_M1_v1_1` als Vorlage.

Verwendete Quellen im Projektordner:

- `ReLife_M1_bluetoothtest`
  Bluetooth-Start auf dem nRF5340 mit Network-Core via `hci_ipc`
- `ReLife_M1_Peripheraltest`
  echtes Board-Mapping für `i2c0`, `spi1`, FRAM-CS und Power-Pins
- `MAX30101_test`
  HR-/SpO2-Sampling und FIFO-Verarbeitung
- `MLX90632_test`
  Temperatur-Library und Melexis-Abhängigkeiten
- `BMA400_test`
  Bosch-BMA400-Library und Schrittzähler
- `rtctest1`
  RV3028-Zeit lesen/schreiben
- `CHALAKOV/Data_Transfer`
  sauberes GATT-/Notify-Muster

## Ablauf

1. Board bekommt Strom.
2. Bluetooth startet sofort.
3. Advertising-Name ist `ReLife M1`.
4. App verbindet sich und schreibt ihre letzte lokale `record_id`.
5. Board sendet alle Records mit größerer `record_id`.
6. App bestätigt mit `ACK_UP_TO`.
7. Erst dann werden die bestätigten FRAM-Slots freigegeben.

## BLE V3

Service-UUID:

- `12345678-1234-5678-1234-56789ABCDEF0`

Characteristics:

- Data Notify: `...DEF1`
- Control Write: `...DEF2`
- Status Read/Notify: `...DEF3`

Control-Opcodes:

- `0x01 = GET_STATUS`
- `0x02 = START_SYNC`
- `0x03 = ACK_UP_TO`
- `0x04 = ABORT_SYNC`
- `0x05 = RTC_SYNC`

Control-Payload:

- kompatibel zu `1 Byte opcode`
- oder `5 Byte opcode + uint32_le`
- oder `8 Byte struct relife_command_wire`

## Record-Format

Ein Record ist bewusst genau `20 Byte` groß, damit er in **ein BLE-Notify ohne MTU-Tricks** passt:

```c
struct relife_record_wire {
    uint32_t record_id;
    uint32_t timestamp_unix;
    uint8_t  heart_rate_bpm;
    uint8_t  spo2_percent;
    int16_t  skin_temp_centi_c;
    uint32_t steps_total;
    uint8_t  battery_percent;
    uint8_t  flags;
    uint16_t crc16;
};
```

Wichtig:

- `crc16` schützt jeden einzelnen Datensatz zusätzlich zu BLE.
- `steps_total` ist der absolute BMA400-Schrittzählerstand.
- `battery_percent` bleibt in V3 bewusst `invalid`, solange kein echter Board-Messpfad dafür dokumentiert und geprüft ist.

## FRAM

FRAM bleibt lokal die Quelle für noch nicht bestätigte Daten:

- `0x00000`: aktueller Write-Pointer
- `0x00004`: Metadaten
- `0x00040`: Record-Ringpuffer

Metadaten halten:

- `head_index`
- `tail_index`
- `count`
- `next_record_id`
- `last_confirmed_id`
- `last_rtc_sync_unix`

Es wird **nicht** nach dem Senden gelöscht, sondern nur nach `ACK_UP_TO`.

## Messlogik

- `RV3028` liefert den Zeitstempel
- `BMA400` liefert absolute Schritte
- `MAX30101` macht ein echtes PPG-Capture-Fenster für HR/SpO2
- `MLX90632` liefert Hauttemperatur

Wenn ein Sensor in einem Zyklus keine gültigen Werte liefert, wird der Record trotzdem mit den anderen gültigen Messwerten gespeichert. Ungültige Felder sind über `flags` markiert.

## Dual-Core Build

Bluetooth läuft auf dem `network core`, die eigentliche Firmware auf dem `application core`.

Build mit Sysbuild:

```bash
west build --sysbuild -b relife_m1_nrf5340_cpuapp
```

## Noch offen

- iOS-App ist noch nicht auf dieses `V3`-Protokoll umgestellt.
- Batterie-Messwert ist bewusst noch nicht verdrahtet.
- Hier konnte kein echter `west build` laufen, weil die Nordic-Toolchain auf diesem Mac fehlt.
