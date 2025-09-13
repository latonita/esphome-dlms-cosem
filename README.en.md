[English version](README.en.md) [Русская версия](README.md)

[DLMS/COSEM](https://github.com/latonita/esphome-dlms-cosem) •
[IEC-61107](https://github.com/latonita/esphome-iec61107-meter) •
[Energomera IEC](https://github.com/latonita/esphome-energomera-iec) •
[Energomera CE](https://github.com/latonita/esphome-energomera-ce) •
[SPb ZIP CE2727A](https://github.com/latonita/esphome-ce2727a-meter) •
[Lenelektro LE-2](https://github.com/latonita/esphome-le2-meter) •
[Pulsar-M](https://github.com/latonita/esphome-pulsar-m)

# esphome-dlms-cosem
ESPHome integration for DLMS/COSEM (SPODES) electricity meters (Energomera CE207/CE307/CE308, Milur 107S, MIR, Nartis, RiM, Pulsar, ZPA AM375 and many others) via RS‑485 or optical port (*).

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
- [Multiple meters](#multiple-meters)
- [Meter specifics](#meter-specifics)
  - [Nartis I100-W112](#nartis-i100-w112)
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
- Cyrillic (cp1251) decoding to UTF‑8 (Nartis I100-W112, RiM 489, …)
- Logical & physical address specification
- Multiple meters on one bus

## Roadmap / Future Ideas
- Datetime objects
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
