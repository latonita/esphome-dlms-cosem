# esphome-dlms-cosem (development in progress)
Подключение EspHome к счетчикам электроэнергии по протоколу DLMS/COSEM/СПОДЭС (Энергомера CE207, CE307, CE308 и другие) по rs485. 

Инструкции по подключению esp32/esp8266 к счётчику можно увидеть в соседнем компоненте https://github.com/latonita/esphome-energomera-iec

Компонент еще в процессе разработки.


# Реализовано:
- Подключение без аутентификации (NONE) и с низким уровнем (LOW - доступ с паролем).
- Поддержка базовых цифровых типов данных (int/float)
- Поддержка базовых текстовых данных (octet-string)

# На будущее:
- Работа с данными типа datetime 
- .. предлагайте ваши варианты


# Примеры 

## ce207 (СПОДЭС)

```
esphome:
  name: energomera-ce207-spds
  friendly_name: Energomera-ce207-spds

esp32:
  board: esp32dev
  framework:
    type: arduino

#...

external_components:
  - source: github://latonita/esphome-dlms-cosem
    components: [dlms_cosem]
    refresh: 1s

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
  update_interval: 60s
  receive_timeout: 1s

text_sensor:
  - platform: dlms_cosem
    name: Заводской номер
    obis_code: 0.0.96.1.0.255
    entity_category: diagnostic
  - platform: dlms_cosem
    name: Тип
    obis_code: 0.0.96.1.1.255
    entity_category: diagnostic
  - platform: dlms_cosem
    name: Версия метрологического ПО
    obis_code: 0.0.96.1.2.255
    entity_category: diagnostic
  - platform: dlms_cosem
    name: Производитель
    obis_code: 0.0.96.1.3.255
    entity_category: diagnostic

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
