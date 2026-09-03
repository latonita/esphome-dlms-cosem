[\[Русская версия\]](README.ru.md) [\[English version\]](README.md)


[СПОДЭС/DLMS/COSEM](https://github.com/latonita/esphome-dlms-cosem) •
[МЭК-61107/IEC-61107](https://github.com/latonita/esphome-iec61107-meter) •
[Энергомера МЭК/IEC](https://github.com/latonita/esphome-energomera-iec) •
[Энергомера CE](https://github.com/latonita/esphome-energomera-ce) •
[СПб ЗИП ЦЭ2727А](https://github.com/latonita/esphome-ce2727a-meter) •
[Ленэлектро ЛЕ-2](https://github.com/latonita/esphome-le2-meter) •
[Пульсар-М](https://github.com/latonita/esphome-pulsar-m) •
[Энергомера BLE](https://github.com/latonita/esphome-energomera-ble) •
[Nordic UART (BLE NUS)](https://github.com/latonita/esphome-nordic-uart-ble)

# esphome-dlms-cosem
Подключение EspHome к счетчикам электроэнергии по протоколу DLMS/COSEM/СПОДЭС (Энергомера CE207/CE307/CE308, Милур 107S, Мир, Нартис, РиМ, Пульсар, ZPA AM375, ZPA ZE312, Sagemcom XT211 и многие другие) через RS-485 интерфейст или через оптопорт(*). Кроме того, возможно подключение через Bluetooth BLE UART (НАРТИС-И100-SP1 и НАРТИС-И300-SP31), используя компонент [Nordic UART (BLE NUS)](https://github.com/latonita/esphome-nordic-uart-ble).

Два режима работы - запрос-ответ и режим ожидания данных от счетчика (PUSH).

Инструкции по подключению esp32/esp8266 к счётчику можно увидеть в соседнем компоненте https://github.com/latonita/esphome-energomera-iec

(*) Через оптопорт можно работать с приборами, которые сразу работают на скорости 9600. Вариант, когда необходимо сначала подключаться на скорости 300, а потом выходить на рабочую скорость - пока не поддерживается (нужно только найти того, кто сможет протестировать).


# Оглавление
- [Функции](#функции)
  - [Реализованы](#реализованы)
  - [Возможные задачи на будущее](#возможные-задачи-на-будущее)
- [Установка](#установка)
- [Быстрый старт](#быстрый-старт)
  - [Минимальная конфигурация хаба и одного сенсора, режим запрос-ответ](#минимальная-конфигурация-хаба-и-одного-сенсора-режим-запрос-ответ)
  - [Минимальная конфигурация хаба и одного сенсора, режим PUSH](#минимальная-конфигурация-хаба-и-одного-сенсора-режим-push)
- [Конфигурация хаба (dlms_cosem)](#конфигурация-хаба-dlms_cosem)
  - [Адресация: client_address и server_address](#адресация-client_address-и-server_address)
- [cp1251 и русские строки](#cp1251-и-русские-строки)
- [Сенсоры](#сенсоры)
  - [Числовой сенсор (sensor)](#числовой-сенсор-sensor)
  - [Текстовый сенсор (text_sensor)](#текстовый-сенсор-text_sensor)
  - [Бинарные сенсоры (binary_sensor)](#бинарные-сенсоры-binary_sensor)
- [Ограничитель мощности (COSEM класс 71)](#ограничитель-мощности-cosem-класс-71)
- [Несколько счётчиков](#несколько-счётчиков)
- [Особенности счетчиков](#особенности-счетчиков)
  - [Нартис И100-W112](#нартис-и100-w112)
  - [Нартис И300/И100 RF2400 - Bluetooth BLE](#нартис-и300и100-rf2400---bluetooth-ble)
  - [РиМ489.38 и другие из серии](#рим48938-и-другие-из-серии)
- [Примеры конфигураций](#примеры-конфигураций)
  - [Однофазный счетчик (ПУ категории D)](#однофазный-счетчик-пу-категории-d)
  - [Трехфазный счетчик в режиме push](#трехфазный-счетчик-в-режиме-push)
- [Диагностика и советы](#диагностика-и-советы)
- [Лицензия](#лицензия)

# Функции
## Реализованы
- Подключние по бинарному протоколу HDLC без аутентификации (NONE) и с низким уровнем (LOW - доступ с паролем)
- Работа в режиме опроса счетчика и режиме ожидания
- Поддержка базовых цифровых типов данных (int/float)
- Поддержка базовых текстовых данных (octet-string)
- Поддержка OBIS классов 1 (Данные), 2 (Регистр), 3 (Расширенный регистр)
- Поддержка OBIS класса 8 (Часы)
- Поддержка OBIS класса 71 (Ограничитель - величина лимита мощности и факт его действия)
- Поддержка русских символов в ответах от счетчиков (Нартис И100-W112, РиМ 489 , ... )
- Задание логического и физического адресов
- Работа с несколькими счетчиками на одной шине


## Возможные задачи на будущее
- Синхронизация времени
- Управление реле
- Полноценная работа через оптопорт по стандартной процедуре (300 → 9600)

Если готовы помочь тестированием — пишите на anton.viktorov@live.com.

---

## Установка
Добавьте внешний компонент в конфигурацию ESPHome:

```yaml
external_components:
  - source: github://latonita/esphome-dlms-cosem
    components: [dlms_cosem]
    refresh: 1s
```

Требуется настроенный UART (RS‑485 через конвертер) или оптопорт.

---

## Быстрый старт
### Минимальная конфигурация хаба и одного сенсора, режим запрос-ответ:

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
  server_address: 1      # см. документацию на ваш счётчик
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

### Минимальная конфигурация хаба и одного сенсора, режим PUSH:

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

## Конфигурация хаба (`dlms_cosem`)

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
  push_show_log: false
  # push_custom_pattern: TV,TC,TSU,TO    # Подробнее в разделе PUSH
```

Параметры:
- **client_address** (*Optional*) — уровень доступа клиента. Для чтения часто используется 32 (требует пароль). По умолчанию: 16.
- **server_address** (*Optional*) — HDLC‑адрес. Можно указать числом либо как объект. По умолчанию: 1.
  - **logical_device** (*Optional*) — логический адрес устройства. По умолчанию: 1.
  - **physical_device** (**Required**) — физический адрес устройства (часто зависит от серийного номера). См. инструкцию к счётчику.
  - **address_length** (*Optional*) — длина адреса (1|2|4 байта). По умолчанию: 2.
- **auth** (*Optional*) — включить авторизацию. По умолчанию: false.
- **password** (*Optional*) — пароль при LOW‑auth.
- **update_interval** (*Optional*) — период опроса. По умолчанию: 60s.
- **receive_timeout** (*Optional*) — таймаут ожидания ответа. По умолчанию: 500ms.
- **delay_between_requests** (*Optional*) — пауза между запросами. По умолчанию: 50ms.
- **flow_control_pin** (*Optional*) — пин управления направлением RE/DE RS‑485‑модуля.
- **id** (*Optional*) — идентификатор хаба (укажите, если их несколько).
- **cp1251** (*Optional*) — конвертация cp1251 → UTF‑8 для текстовых значений. По умолчанию: true.
- **push_mode** (*Optional*) — включить пассивный режим (Push mode), если поддерживается. В режиме PUSH большинство параметров не имеют значения. По умолчанию: false.
- **push_show_log** (*Optional*) - в пассивном режиме (Push mode) выводить подробный лог о найденных COSEM объектах. По умолчанию: false.
- **push_custom_pattern** (*Optional) - Формат Cosem объекта. По умолчанию: нет.

### Адресация: client_address и server_address
- Адреса не нужны, если используется режим PUSH.
- Если не указать, будут использованы значения по умолчанию (16 и 1). Но лучше свериться с документацией к конкретному счётчику.
- Часто используется client_address = 32 (требуется пароль). Уровни по СПОДЭС:

| Код | Уровень | Операции | Защита |
|-----|---------|----------|--------|
| 16  | Публичный клиент | чтение | нет |
| 32  | Считыватель показаний | чтение, выборка, отдельные действия | пароль |
| 48  | Конфигуратор | чтение/запись/выборка/действия | пароль или шифрование (*) |

(*) Шифрование пока не поддерживается.

server_address обычно двухбайтный: старший байт — логический адрес, младший — физический. Детали — в инструкции к счётчику.

Пример (Милур 107S): физический адрес = (последние 4 цифры серийного номера + 16).



---

### cp1251 и русские строки
Некоторые счётчики отдают строки в cp1251 (например, тип ПУ по `0.0.96.1.1.255`). Для корректного отображения в Home Assistant включите `cp1251: true` на уровне хаба или конкретного текстового сенсора. Если конвертация мешает, её можно отключить глобально или точечно в сенсоре.

---

## Сенсоры

Настройка сенсоров в режиме запрос-ответ и в пассивном режиме (Push) не отличается ничем, кроме того, что в пассивном режиме мы не всегда знаем, какие данные присылает счетчик. Для того, чтобы узнать, какие данные счетчик отправляет включите режим логирования:

```yaml
dlms_cosem:
  push_mode: true
  push_show_log: true
```

После чего компонент будет выводить в лог опознанные Cosem объекты. Пример лога тут: [cosem-search.log](cosem-search.log). После настройки сенсоров выключите лог.


### Числовой сенсор (`sensor`)
```yaml
sensor:
  - platform: dlms_cosem
    name: Phase Current
    obis_code: 1.0.11.7.0.255
    multiplier: 1.0        # предварительная мультипликация (до filters:)
    dont_publish: false    # не публиковать в шину (видно только в логах)
    unit_of_measurement: A
    accuracy_decimals: 1
    device_class: current
    state_class: measurement
```
- **obis_class** — класс интерфейса COSEM. По умолчанию 3 (Регистр).
- **attribute** — номер атрибута COSEM. По умолчанию 2 - в нём лежит значение для классов Данные (1),
  Регистр (3) и Расширенный регистр (4). Менять нужно только для объектов, у которых значения лежат в
  других атрибутах - см. [Ограничитель мощности (COSEM класс 71)](#ограничитель-мощности-cosem-класс-71).

### Текстовый сенсор (`text_sensor`)
```yaml
text_sensor:
  - platform: dlms_cosem
    name: Type
    obis_code: 0.0.96.1.1.255
    # obis_class: 8         # для объектов CLOCK (класс 8) необходимо указать класс вручную
    dont_publish: false
    # cp1251: false         # переопределение настройки хаба (опционально)
    entity_category: diagnostic
```
- **cp1251** — если указано у сенсора, перекрывает настройку хаба. Полезно для полей вроде `0.0.96.1.1.255` (тип ПУ на русском).
- **obis_class** — по умолчанию 1 (Данные). Для объектов Часы укажите 8.
- **attribute** — номер атрибута COSEM, по умолчанию 2. См.
  [Ограничитель мощности (COSEM класс 71)](#ограничитель-мощности-cosem-класс-71).

### Бинарные сенсоры (`binary_sensor`)
```yaml
binary_sensor:
  - platform: dlms_cosem
    connection:
      name: Connection      # есть связь с прибором
    session:
      name: Session         # активна сессия
    transmission:
      name: Transmission    # идёт обмен (каждый запрос)
```
Пример использования для индикации активности светодиодом:
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

## Ограничитель мощности (COSEM класс 71)

Ограничитель (COSEM класс 71, обычно `0.0.17.0.0.255`) хранит лимит мощности, который применяет
счетчик. В отличие от регистра, значение лежит не в атрибуте 2, поэтому нужный атрибут указывается
явно через `attribute:`:

| `attribute` | Имя в COSEM | Значение |
| --- | --- | --- |
| 3 | `threshold_active` | Порог, действующий прямо сейчас (только чтение). Обычно равен `threshold_normal`, а во время аварийного профиля - `threshold_emergency`. |
| 4 | `threshold_normal` | Настроенный нормальный лимит. |
| 5 | `threshold_emergency` | Лимит на время работы аварийного профиля. |
| 6 | `min_over_threshold_duration` | Секунд превышения порога до срабатывания действия. |
| 7 | `min_under_threshold_duration` | Секунд ниже порога до срабатывания действия. |
| 10 | `emergency_profile_active` | `1`/`0` - действует ли аварийный профиль. |

Для `obis_class: 71` значение `attribute` по умолчанию равно 3, для остальных классов - 2.

```yaml
sensor:
  # Какой лимит настроен
  - platform: dlms_cosem
    name: Лимит мощности
    obis_code: 0.0.17.0.0.255
    obis_class: 71
    attribute: 4           # threshold_normal
    unit_of_measurement: W
    device_class: power
    state_class: measurement
    accuracy_decimals: 0

  # Какой лимит действует сейчас
  - platform: dlms_cosem
    name: Действующий лимит мощности
    obis_code: 0.0.17.0.0.255
    obis_class: 71
    attribute: 3           # threshold_active
    unit_of_measurement: W
    device_class: power
    state_class: measurement
    accuracy_decimals: 0
```

**Действует ли ограничение?** Отдельного атрибута "включен" нет. Сравнивайте `threshold_active` (3)
с `threshold_normal` (4) и `threshold_emergency` (5) - так видно, какой порог применяет счетчик.
Выключенный ограничитель на одних счетчиках читается как `0`, на других - как максимум типа данных
(например, `4294967295` для `double-long-unsigned`); посмотрите лог и фильтруйте соответственно.

**Единицы.** У ограничителя нет собственного атрибута scaler/unit: порог выражен в единицах того
регистра, который он контролирует. Этот регистр указан в атрибуте 2 (`monitored_value`) - обычно
`{3, 1-0:90.7.0.255, 2}` или `{3, 1-0:1.24.0.255, 2}`. Если добавить обычный сенсор на этот OBIS с
`obis_class: 3`, компонент запросит его scaler и unit и выведет их в лог - отсюда понятно, как
масштабировать порог. Единицы задавайте сами через `unit_of_measurement`, при необходимости
используйте `multiplier` (например, `multiplier: 0.001`, если порог приходит в ваттах, а нужны
киловатты).

**Права доступа.** На многих счетчиках объект Ограничителя доступен только клиенту с наивысшими
правами: колонка прав вида `-/-/G` для клиентов `16/102/1` означает, что читать может лишь клиент 1.
По умолчанию хаб использует `client_address: 16`, поэтому если чтение ограничителя возвращает отказ
в доступе - поднимите `client_address` и укажите соответствующий `password`.

Атрибут ограничителя можно читать и как `text_sensor`; для `attribute: 10` тогда публикуется
`true`/`false` вместо `1`/`0`.

---

## Несколько счётчиков

- NB: В режиме PUSH может быть только один счетчик на одной шине.

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

# Особенности счетчиков

## Нартис И100-W112

- Передает тип ПУ на русском языке. Для текстового сенсора `0.0.96.1.1.255` необходимо установить `cp1251: true`
- Иногда пароли и явки в инструкции отличаются от реальных. Пример рабочих параметро с одного из счетчиков:

    * Пароль администрирования: 0000000100000001
    * Пароль чтения: 00000001
    * Логический адрес: 1
    * Физический адрес: 17
    * Размер адреса: 2

## Нартис И300/И100 RF2400 - Bluetooth BLE
Счетчики Нартис с опцией RF2400 могут быть подключены через соседний компонент `ble_nus_client`. См. [Nordic UART (BLE NUS)](https://github.com/latonita/esphome-nordic-uart-ble).
Проверено с НАРТИС-И300-SP31-2-A1R1-230-5-100A-TN-RF2400/2-RS485-P1-EНKMOQ1V3-D.

```yaml

external_components:
  - source: github://latonita/esphome-dlms-cosem
    refresh: 10s
    components: [dlms_cosem]
  - source: github://latonita/esphome-nordic-uart-ble
    refresh: 10s
    components: [ble_nus_client]
  
ble_client:
  - mac_address: "11:22:33:44:55:66" # Bluetooth MAC адрес счетчика
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
  pin: 123456  # пин код Bluetooth
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
  password: "00002080"  # Пароль доступа. Ваш пароль может быть другим - проверьте паспорт на ваш прибор учета.
  receive_timeout: 5000ms
  update_interval: 60s


```
## РиМ489.38 и другие из серии
- Передает тип ПУ на русском языке. Для текстового сенсора `0.0.96.1.1.255` необходимо установить `cp1251: true`


# Примеры конфигураций

## Однофазный счетчик (ПУ категории D) 
Используется список параметров ПУ категории D из стандарта СПОДЭС. Они применяются в однофазных ПУ потребителей.

Пример файла конфигурации, протестированого на Энергомера CE207-SPds.

```yaml
esphome:
  name: energomera-ce207-spds
  friendly_name: Energomera-ce207-spds

esp32:
  board: esp32dev

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


## Трехфазный счетчик в режиме push

Работа в режиме PUSH на примере счетчика ZPA AM375.

```yaml
esphome:
  name: zpa-am375
  friendly_name: zpa-am375

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

## Диагностика и советы
- Нет ответа: проверьте RE/DE (flow_control_pin) и направление конвертера RS‑485
- Таймауты: увеличьте `receive_timeout` и `delay_between_requests`
- Адреса: перепроверьте `client_address` и `server_address` по инструкции (учитывайте адреса для оптопорта vs RS‑485)
- Русский текст «кракозябрами»: включите `cp1251: true`

---

## Лицензия
См. [LICENSE](LICENSE) в репозитории.
