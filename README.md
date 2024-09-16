# esphome-dlms-cosem
Подключение EspHome к счетчикам электроэнергии по протоколу DLMS/COSEM/СПОДЭС (Энергомера CE207, CE307, CE308 и другие) по rs485. 

Версия нуль-нуль-нуль-первая. Супер-бета.

# Реализовано:
- Подключение без аутентификации (NONE) и с низким уровнем (LOW)
- Поддержка базовых цифровых типов данных (int/float/enum/bool)


На будущее:
- Работа с данными типа string, datetime
- .. предлагайте ваши варианты


# Примеры 

## ce207 (СПОДЭС)

```
...

external_components:
  components: [dlms_cosem]
   - source: github://latonita/esphome-dlms-cosem

uart:
  - id: bus_1
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
  update_interval: 30s

sensor:
  - platform: dlms_cosem
    name: Электроэнергия
    obis_code: 1.0.1.8.0.255
    unit_of_measurement: kWh
    accuracy_decimals: 0
    device_class: energy
    state_class: total_increasing
    filters:
      - multiply: 0.001

  - platform: dlms_cosem
    name: Электроэнергия Т1
    obis_code: 1.0.1.8.1.255
    unit_of_measurement: kWh
    accuracy_decimals: 0
    device_class: energy
    state_class: total_increasing
    filters:
      - multiply: 0.001

  - platform: dlms_cosem
    name: Электроэнергия Т2
    obis_code: 1.0.1.8.2.255
    unit_of_measurement: kWh
    accuracy_decimals: 0
    device_class: energy
    state_class: total_increasing
    filters:
      - multiply: 0.001

  - platform: dlms_cosem
    name: Активная мощность
    obis_code: 1.0.1.7.0.255
    unit_of_measurement: W
    accuracy_decimals: 0
    device_class: power
    state_class: measurement

  - platform: dlms_cosem
    name: Напряжение фазы
    obis_code: 1.0.12.7.0.255
    unit_of_measurement: V
    accuracy_decimals: 3
    device_class: voltage
    state_class: measurement

```