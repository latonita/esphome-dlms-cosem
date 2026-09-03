[\[English version\]](README.md) [\[Русская версия\]](README.ru.md)

[DLMS/COSEM](https://github.com/latonita/esphome-dlms-cosem) •
[IEC-61107](https://github.com/latonita/esphome-iec61107-meter) •
[Energomera IEC](https://github.com/latonita/esphome-energomera-iec) •
[Energomera CE](https://github.com/latonita/esphome-energomera-ce) •
[SPb ZIP CE2727A](https://github.com/latonita/esphome-ce2727a-meter) •
[Lenelektro LE-2](https://github.com/latonita/esphome-le2-meter) •
[Pulsar-M](https://github.com/latonita/esphome-pulsar-m) •
[Energomera BLE](https://github.com/latonita/esphome-energomera-ble) •
[Nordic UART (BLE NUS)](https://github.com/latonita/esphome-nordic-uart-ble)

# esphome-dlms-cosem
ESPHome integration for DLMS/COSEM (SPODES) electricity meters (Energomera CE207/CE307/CE308, Milur 107S, MIR, Nartis, RiM, Pulsar, Aidon, Kamstrup, Landis+Gyr, any HAN, ZPA AM375, ZPA ZE312, Sagemcom XT211 and many others) via RS‑485 or optical port (*). 

Additionally, Bluetooth BLE UART connection is possible (Nartis I300/I100) using the [Nordic UART (BLE NUS)](https://github.com/latonita/esphome-nordic-uart-ble) component.

Two operating modes: request/response polling and passive PUSH (meter-originated data).

For ESP32/ESP8266 physical wiring examples see: https://github.com/latonita/esphome-energomera-iec

(*) Optical head currently supported only when the meter already uses 9600 baud. The sequence “start at 300 baud then switch to 9600” isn’t implemented yet (needs testing hardware).

# Table of Contents
- [Features](#features)
  - [Implemented](#implemented)
  - [Roadmap / Future Ideas](#roadmap--future-ideas)
- [Installation](#installation)
- [Quick Start](#quick-start)
  - [Minimal hub + one sensor (polling mode)](#minimal-hub--one-sensor-polling-mode)
  - [Minimal hub + one sensor (PUSH mode)](#minimal-hub--one-sensor-push-mode)
- [Hub configuration (dlms_cosem)](#hub-configuration-dlms_cosem)
  - [Addressing: client_address & server_address](#addressing-client_address--server_address)
- [cp1251 and Cyrillic strings](#cp1251-and-cyrillic-strings)
- [Sensors](#sensors)
  - [Numeric sensor (sensor)](#numeric-sensor-sensor)
  - [Text sensor (text_sensor)](#text-sensor-text_sensor)
  - [Binary sensors (binary_sensor)](#binary-sensors-binary_sensor)
- [Power limiter (COSEM class 71)](#power-limiter-cosem-class-71)
- [Multiple meters](#multiple-meters)
- [Meter specifics](#meter-specifics)
  - [Nartis I100-W112](#nartis-i100-w112)
  - [Nartis I300/I100 RF2400 - Bluetooth BLE](#nartis-i300i100-rf2400---bluetooth-ble)
  - [RiM489.38 and related models](#rim48938-and-related-models)
- [Configuration examples](#configuration-examples)
  - [Single-phase meter (Category D)](#single-phase-meter-category-d)
  - [Three-phase meter in PUSH mode](#three-phase-meter-in-push-mode)
- [Diagnostics & tips](#diagnostics--tips)
- [License](#license)

# Features
## Implemented
- HDLC binary transport, authentication NONE and LOW (password)
- Polling mode and passive PUSH mode
- Basic numeric data types (int/float)
- Basic textual data (octet-string)
- Major obis classes - 1 (Data), 2 (Register), 3 (Extended Register)
- Clock obis class - 8 (Clocj) 
- Limiter obis class - 71 (power limit threshold and whether it is in force)
- Cyrillic (cp1251) decoding to UTF‑8 (Nartis I100-W112, RiM 489, …)
- Logical & physical address specification
- Multiple meters on one bus

## Roadmap / Future Ideas
- Time synchronization
- Relay control
- Full optical head speed negotiation (300 → 9600)

If you can help with testing, contact: anton.viktorov@live.com

---

## Installation
Add the external component to your ESPHome config:

```yaml
external_components:
  - source: github://latonita/esphome-dlms-cosem
    components: [dlms_cosem]
    refresh: 1s
```

A configured UART (RS‑485 adapter) or optical head is required.

---

## Quick Start
### Minimal hub + one sensor (polling mode)
```yaml
uart:
  id: bus_1
  rx_pin: GPIO16
  tx_pin: GPIO17
  baud_rate: 9600
  data_bits: 8
  parity: NONE
  stop_bits: 1

dlms_cosem:
  client_address: 32
  server_address: 1      # see your meter manual
  auth: true
  password: "12345678"
  update_interval: 60s

sensor:
  - platform: dlms_cosem
    name: Active Power
    obis_code: 1.0.1.7.0.255
    unit_of_measurement: W
    accuracy_decimals: 1
    device_class: power
    state_class: measurement
```

---

### Minimal hub + one sensor (PUSH mode)
```yaml
uart:
  id: bus_1
  rx_pin: GPIO16
  tx_pin: GPIO17
  baud_rate: 9600
  data_bits: 8
  parity: NONE
  stop_bits: 1

dlms_cosem:
  push_mode: true

sensor:
  - platform: dlms_cosem
    name: Active Power
    obis_code: 1.0.1.7.0.255
    unit_of_measurement: W
    accuracy_decimals: 1
    device_class: power
    state_class: measurement
```

---

## Hub configuration (`dlms_cosem`)
```yaml
dlms_cosem:
  client_address: 32
  server_address:
    logical_device: 1
    physical_device: 576
    address_length: 2
  auth: true
  password: "12345678"
  update_interval: 60s
  receive_timeout: 500ms
  delay_between_requests: 50ms
  flow_control_pin: GPIO12
  id: energo_01
  cp1251: true
  push_mode: false
  # push_custom_pattern: TV,TC,TSU,TO    # See PUSH chapter
```
Parameters:
- **client_address** (*Optional*) — client access level. 32 often used (needs password). Default: 16.
- **server_address** (*Optional*) — HDLC address. Number or object. Default: 1.
  - **logical_device** (*Optional*) — logical device id. Default: 1.
  - **physical_device** (**Required**) — physical device id (often derived from serial). See manual.
  - **address_length** (*Optional*) — address length (1|2|4 bytes). Default: 2.
- **auth** (*Optional*) — enable authentication. Default: false.
- **password** (*Optional*) — LOW auth password.
- **update_interval** (*Optional*) — polling interval. Default: 60s.
- **receive_timeout** (*Optional*) — response timeout. Default: 500ms.
- **delay_between_requests** (*Optional*) — pause between requests. Default: 50ms.
- **flow_control_pin** (*Optional*) — RE/DE direction pin for RS‑485.
- **id** (*Optional*) — hub id (if you have several).
- **cp1251** (*Optional*) — cp1251 → UTF‑8 conversion. Default: true.
- **push_mode** (*Optional*) — passive push mode. In PUSH most other params ignored. Default: false.
- **push_show_log** (*Optional*) - show detailed log - which Cosem objects found in passive mode (Push mode). Default: false.
- **push_custom_pattern** (*Optional) - custom Cosem object pattern. Default: None.

### Addressing: client_address & server_address
- Not needed in PUSH mode.
- If omitted defaults are used (16 and 1). Always check meter docs.
- client_address 32 is common (password required). Levels:

| Code | Level | Operations | Protection |
|------|-------|-----------|------------|
| 16 | Public client | read | none |
| 32 | Meter reading | read, selective read, some actions | password |
| 48 | Configurator | read/write/select/actions | password or encryption (*) |

(*) Encryption not supported yet.

server_address usually 2 bytes: high byte = logical address, low byte = physical. See manual.

Example (Milur 107S): physical = (last 4 serial digits + 16).

---

## cp1251 and Cyrillic strings
Some meters output cp1251 (e.g. type at `0.0.96.1.1.255`). Enable `cp1251: true` at hub or per text sensor. Disable globally or per‑sensor if conversion breaks something.

---

## Sensors

Sensor configuration in polling and passive (Push) modes is identical, except that in passive mode you often don’t know in advance which objects the meter will send. To discover what the meter is pushing, enable extended logging:

```yaml
dlms_cosem:
  push_mode: true
  push_show_log: true
```

After that the component will print recognized COSEM objects to the log. Example log: [cosem-search.log](cosem-search.log). Disable `push_show_log` once you’ve created the needed sensors.

### Numeric sensor (`sensor`)
```yaml
sensor:
  - platform: dlms_cosem
    name: Phase Current
    obis_code: 1.0.11.7.0.255
    multiplier: 1.0        # pre-multiply (before filters:)
    dont_publish: false    # do not publish, only log
    unit_of_measurement: A
    accuracy_decimals: 1
    device_class: current
    state_class: measurement
```
- **obis_class** — COSEM interface class of the object. Default 3 (Register).
- **attribute** — COSEM attribute index to read. Default 2, which holds the value for Data (1),
  Register (3) and Extended Register (4). Change it only for objects that keep their values in other
  attributes - see [Power limiter (COSEM class 71)](#power-limiter-cosem-class-71).

### Text sensor (`text_sensor`)
```yaml
text_sensor:
  - platform: dlms_cosem
    name: Type
    obis_code: 0.0.96.1.1.255
    dont_publish: false
    # cp1251: false        # override hub setting (optional)
    entity_category: diagnostic
```
- **cp1251** — per-sensor override. Useful for fields like `0.0.96.1.1.255`.
- **obis_class** — default 1 (Data). Use 8 for Clock objects.
- **attribute** — COSEM attribute index, default 2. See
  [Power limiter (COSEM class 71)](#power-limiter-cosem-class-71).

### Binary sensors (`binary_sensor`)
```yaml
binary_sensor:
  - platform: dlms_cosem
    connection:
      name: Connection      # link available
    session:
      name: Session         # active session
    transmission:
      name: Transmission    # each request
```
LED activity example:
```yaml
binary_sensor:
  - platform: dlms_cosem
    transmission:
      name: Transmission
      on_press:
        output.turn_on: transmission_led
      on_release:
        output.turn_off: transmission_led

output:
  - platform: gpio
    id: transmission_led
    pin: GPIO04
    inverted: true
```

---

## Power limiter (COSEM class 71)

The Limiter (COSEM interface class 71, usually `0.0.17.0.0.255`) holds the load limit the meter
enforces. Unlike a Register, its value does not live in attribute 2, so the attribute has to be
named explicitly with `attribute:`:

| `attribute` | COSEM name | Meaning |
| --- | --- | --- |
| 3 | `threshold_active` | The threshold in force right now (read-only). Normally equal to `threshold_normal`, and to `threshold_emergency` while an emergency profile runs. |
| 4 | `threshold_normal` | The configured normal limit. |
| 5 | `threshold_emergency` | The limit used while an emergency profile is running. |
| 6 | `min_over_threshold_duration` | Seconds over the threshold before the action fires. |
| 7 | `min_under_threshold_duration` | Seconds under the threshold before the action fires. |
| 10 | `emergency_profile_active` | `1`/`0` - whether an emergency profile is in force. |

`attribute` defaults to 3 for `obis_class: 71`, and to 2 for every other class.

```yaml
sensor:
  # What limit is configured
  - platform: dlms_cosem
    name: Power limit
    obis_code: 0.0.17.0.0.255
    obis_class: 71
    attribute: 4           # threshold_normal
    unit_of_measurement: W
    device_class: power
    state_class: measurement
    accuracy_decimals: 0

  # What limit the meter is enforcing right now
  - platform: dlms_cosem
    name: Power limit in force
    obis_code: 0.0.17.0.0.255
    obis_class: 71
    attribute: 3           # threshold_active
    unit_of_measurement: W
    device_class: power
    state_class: measurement
    accuracy_decimals: 0
```

**Is the limiter in force?** There is no single "enabled" attribute. Compare `threshold_active` (3)
against `threshold_normal` (4) and `threshold_emergency` (5) to see which one the meter is applying.
A limiter that is switched off reads back as `0` on some meters and as the maximum value of the data
type (e.g. `4294967295` for `double-long-unsigned`) on others - check your log once and filter
accordingly.

**Units.** The limiter has no scaler/unit attribute of its own: the threshold is expressed in the
units of the register it monitors, named by attribute 2 (`monitored_value`) - commonly
`{3, 1-0:90.7.0.255, 2}` or `{3, 1-0:1.24.0.255, 2}`. Adding an ordinary sensor for that OBIS code
with `obis_class: 3` makes the component fetch its scaler and unit and print them to the log, which
tells you how the threshold has to be scaled. Then set `unit_of_measurement` yourself and use
`multiplier` if needed (e.g. `multiplier: 0.001` for a threshold given in watts but wanted in kW).

**Access rights.** On many meters the whole Limiter object is readable only by the highest-privilege
client - an access-rights column reading `-/-/G` for clients `16/102/1` means only client 1 may get
it. The hub defaults to `client_address: 16`, so if limiter reads come back as access-denied, raise
`client_address` and supply the matching `password`.

A limiter attribute can also be read as a `text_sensor`; `attribute: 10` then publishes
`true`/`false` instead of `1`/`0`.

---

## Multiple meters
- NB: Only one meter per bus in PUSH mode.
```yaml
uart:
  - id: bus_1
    rx_pin: GPIO16
    tx_pin: GPIO17
    baud_rate: 9600
    data_bits: 8
    parity: NONE
    stop_bits: 1

  - id: bus_2
    rx_pin: GPIO23
    tx_pin: GPIO22
    baud_rate: 9600
    data_bits: 8
    parity: NONE
    stop_bits: 1

dlms_cosem:
  - id: energo_1
    uart_id: bus_1
    client_address: 32
    server_address:
      logical_device: 1
      physical_device: 576
    auth: true
    password: "12345678"
    update_interval: 30s

  - id: energo_2
    uart_id: bus_1
    client_address: 32
    server_address:
      logical_device: 1
      physical_device: 16
    auth: true
    password: "12345678"
    update_interval: 30s

  - id: nartis_3
    uart_id: bus_2
    client_address: 32
    server_address:
      logical_device: 1
      physical_device: 17
      address_length: 2
    auth: true
    password: "00000001"

text_sensor:
  - platform: dlms_cosem
    dlms_cosem_id: energo_2
    name: Serial number
    obis_code: 0.0.96.1.0.255
    entity_category: diagnostic
    cp1251: true

sensor:
  - platform: dlms_cosem
    dlms_cosem_id: nartis_3
    name: Current
    obis_code: 1.0.11.7.0.255
    unit_of_measurement: A
    accuracy_decimals: 1
    device_class: current
    state_class: measurement
```

# Meter specifics
## Nartis I100-W112
- Device type in Russian. For `0.0.96.1.1.255` set `cp1251: true`.
- Sometimes manual values differ from reality. Example working set:
  * Admin password: 0000000100000001
  * Read password: 00000001
  * Logical address: 1
  * Physical address: 17
  * Address size: 2

## Nartis I300/I100 RF2400 - Bluetooth BLE
Nartis meters with RF2400 option can be connected via the companion `ble_nus_client` component. See [Nordic UART (BLE NUS)](https://github.com/latonita/esphome-nordic-uart-ble).
Tested with NARTIS-I300-SP31-2-A1R1-230-5-100A-TN-RF2400/2-RS485-P1-EНKMOQ1V3-D.

```yaml

external_components:
  - source: github://latonita/esphome-dlms-cosem
    refresh: 10s
    components: [dlms_cosem]
  - source: github://latonita/esphome-nordic-uart-ble
    refresh: 10s
    components: [ble_nus_client]
  
ble_client:
  - mac_address: "11:22:33:44:55:66" # Bluetooth MAC address of the meter
    id: nartis_i300_ble
    auto_connect: false
    
esp32_ble_tracker:
  scan_parameters:
    interval: 300ms
    window: 300ms
    active: true    
    continuous: false

ble_nus_client:
  id: ble_uart
  pin: 123456  # Bluetooth PIN code
  service_uuid: 6e400001-b5a3-f393-e0a9-e50e24dc4179
  rx_uuid: 6e400002-b5a3-f393-e0a9-e50e24dc4179
  tx_uuid: 6e400003-b5a3-f393-e0a9-e50e24dc4179   
  mtu: 247
  connect_on_demand: true
  idle_timeout: 5min

dlms_cosem:
  id: nartis_dlms
  uart_id: ble_uart
  client_address: 32
  server_address: 1
  auth: true
  password: "00002080"  # Access password. Your password may differ - check your meter documentation.
  receive_timeout: 5000ms
  update_interval: 60s


```
## RiM489.38 and related models
- Device type in Russian: for `0.0.96.1.1.255` use `cp1251: true`.

# Configuration examples
## Single-phase meter (Category D)
Uses Category D parameter list from SPODES standard. Tested with Energomera CE207-SPds.
```yaml
esphome:
  name: energomera-ce207-spds
  friendly_name: Energomera-ce207-spds

esp32:
  board: esp32dev
  framework:
    type: arduino

external_components:
  - source: github://latonita/esphome-dlms-cosem
    components: [dlms_cosem]
    refresh: 1s

uart:
  id: bus_1
  rx_pin: GPIO16
  tx_pin: GPIO17
  baud_rate: 9600
  data_bits: 8
  parity: NONE
  stop_bits: 1

dlms_cosem:
  id: energo_01
  client_address: 32
  server_address: 1
  auth: true
  password: "12345678"
  update_interval: 60s
  receive_timeout: 1s

sensor:
  - platform: dlms_cosem
    name: Phase Current
    obis_code: 1.0.11.7.0.255
    unit_of_measurement: A
    accuracy_decimals: 1
    device_class: current
    state_class: measurement
  - platform: dlms_cosem
    name: Neutral Current
    obis_code: 1.0.91.7.0.255
    unit_of_measurement: A
    accuracy_decimals: 1
    device_class: current
    state_class: measurement
  - platform: dlms_cosem
    name: Phase Voltage
    obis_code: 1.0.12.7.0.255
    unit_of_measurement: V
    accuracy_decimals: 1
    device_class: voltage
    state_class: measurement
  - platform: dlms_cosem
    name: Power Factor
    obis_code: 1.0.13.7.0.255
    unit_of_measurement: ''
    accuracy_decimals: 2
    device_class: power_factor
    state_class: measurement
  - platform: dlms_cosem
    name: Grid Frequency
    obis_code: 1.0.14.7.0.255
    unit_of_measurement: Hz
    accuracy_decimals: 1
    device_class: frequency
    state_class: measurement
  - platform: dlms_cosem
    name: Apparent Power
    obis_code: 1.0.9.7.0.255
    unit_of_measurement: W
    accuracy_decimals: 1
    device_class: power
    state_class: measurement
  - platform: dlms_cosem
    name: Active Power
    obis_code: 1.0.1.7.0.255
    unit_of_measurement: W
    accuracy_decimals: 1
    device_class: power
    state_class: measurement
  - platform: dlms_cosem
    name: Reactive Power
    obis_code: 1.0.3.7.0.255
    unit_of_measurement: W
    accuracy_decimals: 1
    device_class: power
    state_class: measurement
  - platform: dlms_cosem
    name: Active Energy
    obis_code: 1.0.1.8.0.255
    unit_of_measurement: kWh
    accuracy_decimals: 3
    device_class: energy
    state_class: total_increasing
    filters:
      - multiply: 0.001
  - platform: dlms_cosem
    name: Active Energy T1
    obis_code: 1.0.1.8.1.255
    unit_of_measurement: kWh
    accuracy_decimals: 3
    device_class: energy
    state_class: total_increasing
    filters:
      - multiply: 0.001
  - platform: dlms_cosem
    name: Active Energy T2
    obis_code: 1.0.1.8.2.255
    unit_of_measurement: kWh
    accuracy_decimals: 3
    device_class: energy
    state_class: total_increasing
    filters:
      - multiply: 0.001
  - platform: dlms_cosem
    name: Active Energy Export
    obis_code: 1.0.2.8.0.255
    unit_of_measurement: kWh
    accuracy_decimals: 3
    device_class: energy
    state_class: total_increasing
    filters:
      - multiply: 0.001
  - platform: dlms_cosem
    name: Reactive Energy
    obis_code: 1.0.3.8.0.255
    unit_of_measurement: kWh
    accuracy_decimals: 3
    device_class: energy
    state_class: total_increasing
    filters:
      - multiply: 0.001
  - platform: dlms_cosem
    name: Reactive Energy Export
    obis_code: 1.0.4.8.0.255
    unit_of_measurement: kWh
    accuracy_decimals: 3
    device_class: energy
    state_class: total_increasing
    filters:
      - multiply: 0.001

text_sensor:
  - platform: dlms_cosem
    name: Date/Time
    obis_code: 0.0.1.0.0.255
    entity_category: diagnostic
    class: 8
  - platform: dlms_cosem
    name: Serial Number
    obis_code: 0.0.96.1.0.255
    entity_category: diagnostic
  - platform: dlms_cosem
    name: Type
    obis_code: 0.0.96.1.1.255
    cp1251: true
    entity_category: diagnostic
  - platform: dlms_cosem
    name: Metrology Software Version
    obis_code: 0.0.96.1.2.255
    entity_category: diagnostic
  - platform: dlms_cosem
    name: Manufacturer
    obis_code: 0.0.96.1.3.255
    entity_category: diagnostic
```

## Three-phase meter in PUSH mode
Example: ZPA AM375.
```yaml
esphome:
  name: energomera-ce207-spds
  friendly_name: Energomera-ce207-spds

esp32:
  board: esp32dev
  framework:
    type: arduino

external_components:
  - source: github://latonita/esphome-dlms-cosem
    components: [dlms_cosem]
    refresh: 1s

uart:
  id: bus_1
  rx_pin: GPIO16
  tx_pin: GPIO17
  baud_rate: 9600
  data_bits: 8
  parity: NONE
  stop_bits: 1

dlms_cosem:
  id: cosem1
  uart_id: bus_1
  push_mode: true

text_sensor:
  - platform: dlms_cosem
    name: Date/Time
    obis_code: 0.0.1.0.0.255
    entity_category: diagnostic
    class: 8

  - platform: dlms_cosem
    name: Serial number
    obis_code: 0.0.96.1.1.255
    entity_category: diagnostic

  - platform: dlms_cosem
    name: Current tariff
    obis_code: 0.0.96.14.0.255
    entity_category: diagnostic

sensor:

  - platform: dlms_cosem
    id: active_energy_consumed
    name: Energy
    obis_code: 1.0.1.8.0.255
    unit_of_measurement: kWh
    accuracy_decimals: 0
    device_class: energy
    state_class: total_increasing

  - platform: dlms_cosem
    id: active_energy_consumed_t1
    name: Energy T1
    obis_code: 1.0.1.8.1.255
    unit_of_measurement: kWh
    accuracy_decimals: 0
    device_class: energy
    state_class: total_increasing

  - platform: dlms_cosem
    id: active_energy_consumed_t2
    name: Energy T2
    obis_code: 1.0.1.8.2.255
    unit_of_measurement: kWh
    accuracy_decimals: 0
    device_class: energy
    state_class: total_increasing

  - platform: dlms_cosem
    id: active_power
    name: Active power Total
    obis_code: 1.0.1.7.0.255
    unit_of_measurement: W
    accuracy_decimals: 0
    device_class: power
    state_class: measurement

  - platform: dlms_cosem
    id: active_power_l1
    name: Active power L1
    obis_code: 1.0.21.7.0.255
    unit_of_measurement: W
    accuracy_decimals: 0
    device_class: power
    state_class: measurement

  - platform: dlms_cosem
    id: active_power_l2
    name: Active power L2
    obis_code: 1.0.41.7.0.255
    unit_of_measurement: W
    accuracy_decimals: 0
    device_class: power
    state_class: measurement

  - platform: dlms_cosem
    id: active_power_l3
    name: Active power L3
    obis_code: 1.0.61.7.0.255
    unit_of_measurement: W
    accuracy_decimals: 0
    device_class: power
    state_class: measurement
```

---

## Diagnostics & tips
- No response: check RE/DE pin (flow_control_pin) and RS‑485 adapter direction
- Timeouts: increase `receive_timeout` and `delay_between_requests`
- Addresses: re-check `client_address` / `server_address` (optical vs RS‑485 may differ)
- Cyrillic garbage: enable `cp1251: true`

---

## License
See LICENSE file.
