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

## Однофазный счетчик (ПУ категории D) 
Используется список параметров ПУ категории D из стандарта СПОДЭС. Они применяются в однофазных ПУ потребителей.

Пример файла конфигурации, протестированого на Энергомера CE207-SPds.

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

sensor:
# 2 Current - Iph Ток фазы 1.0.11.7.0.255
  - platform: dlms_cosem
    name: Ток фазы
    obis_code: 1.0.11.7.0.255
    unit_of_measurement: A
    accuracy_decimals: 1
    device_class: current
    state_class: measurement

# 3 Current - In Ток нулевого провода 1.0.91.7.0.255
  - platform: dlms_cosem
    name: Ток нулевого провода
    obis_code: 1.0.91.7.0.255
    unit_of_measurement: A
    accuracy_decimals: 1
    device_class: current
    state_class: measurement

# 4 Voltage - V Напряжение фазы 1.0.12.7.0.255
  - platform: dlms_cosem
    name: Напряжение фазы
    obis_code: 1.0.12.7.0.255
    unit_of_measurement: V
    accuracy_decimals: 1
    device_class: voltage
    state_class: measurement

# 5 Power Factor - PF Коэффициент мощности 1.0.13.7.0.255
  - platform: dlms_cosem
    name: Коэффициент мощности
    obis_code: 1.0.13.7.0.255
    unit_of_measurement: ''
    accuracy_decimals: 2
    device_class: power_factor
    state_class: measurement

# 6 Frequency Частота сети 1.0.14.7.0.255
  - platform: dlms_cosem
    name: Частота сети
    obis_code: 1.0.14.7.0.255
    unit_of_measurement: Hz
    accuracy_decimals: 1
    device_class: frequency
    state_class: measurement

# 7 Apparent Power Полная мощность 1.0.9.7.0.255
  - platform: dlms_cosem
    name: Полная мощность
    obis_code: 1.0.9.7.0.255
    unit_of_measurement: W
    accuracy_decimals: 1
    device_class: power
    state_class: measurement

# 8 Signed Active Power (+Import; -Export) Активная мощность 1.0.1.7.0.255
  - platform: dlms_cosem
    name: Активная мощность
    obis_code: 1.0.1.7.0.255
    unit_of_measurement: W
    accuracy_decimals: 1
    device_class: power
    state_class: measurement

# 9 Signed Reactive Power (+Import; -Export) Реактивная мощность 1.0.3.7.0.255
  - platform: dlms_cosem
    name: Реактивная мощность
    obis_code: 1.0.3.7.0.255
    unit_of_measurement: W
    accuracy_decimals: 1
    device_class: power
    state_class: measurement

# 10 Cumulative Active Energy (Import) Активная энергия, импорт 1.0.1.8.0.255
  - platform: dlms_cosem
    name: Электроэнергия
    obis_code: 1.0.1.8.0.255
    unit_of_measurement: kWh
    accuracy_decimals: 3
    device_class: energy
    state_class: total_increasing
    filters:
      - multiply: 0.001

# 10.1 Cumulative Active Energy (Import) tariff 1
  - platform: dlms_cosem
    name: Электроэнергия Т1
    obis_code: 1.0.1.8.1.255
    unit_of_measurement: kWh
    accuracy_decimals: 3
    device_class: energy
    state_class: total_increasing
    filters:
      - multiply: 0.001

# 10.2 Cumulative Active Energy (Import) tariff 2
  - platform: dlms_cosem
    name: Электроэнергия Т2
    obis_code: 1.0.1.8.2.255
    unit_of_measurement: kWh
    accuracy_decimals: 3
    device_class: energy
    state_class: total_increasing
    filters:
      - multiply: 0.001

# 11 Cumulative Active Energy (Export) Активная энергия, экспорт 1.0.2.8.0.255
  - platform: dlms_cosem
    name: Электроэнергия, экспорт
    obis_code: 1.0.2.8.0.255
    unit_of_measurement: kWh
    accuracy_decimals: 3
    device_class: energy
    state_class: total_increasing
    filters:
      - multiply: 0.001

# 12 Cumulative Reactive Energy (Import) Реактивная энергия, импорт 1.0.3.8.0.255
  - platform: dlms_cosem
    name: Реактивная энергия
    obis_code: 1.0.3.8.0.255
    unit_of_measurement: kWh
    accuracy_decimals: 3
    device_class: energy
    state_class: total_increasing
    filters:
      - multiply: 0.001

# 13 Cumulative Reactive Energy (Export) Реактивная энергия, экспорт 1.0.4.8.0.255
  - platform: dlms_cosem
    name: Реактивная энергия, экспорт
    obis_code: 1.0.4.8.0.255
    unit_of_measurement: kWh
    accuracy_decimals: 3
    device_class: energy
    state_class: total_increasing
    filters:
      - multiply: 0.001

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

```