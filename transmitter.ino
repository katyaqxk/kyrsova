/*
 * transmitter.ino — передавальний вузол (вузол датчика)
 * Проєкт: Дистанційна передача даних датчика температури і вологості
 *         на модулі зв'язку 2.4 GHz (nRF24L01) на OLED-дисплей
 * Плата:  Arduino Uno R3 (ATmega328P)
 * Бібліотеки:
 *   RF24 by TMRh20                v1.4.9
 *   DHT sensor library (Adafruit) v1.4.6
 *   Adafruit Unified Sensor       v1.1.14
 * Ліцензія: MIT
 */
#include <SPI.h>
#include <RF24.h>
#include <DHT.h>

// ---------- Призначення пінів ----------
#define PIN_DHT     2     // лінія DATA датчика DHT11
#define PIN_LED_TX  4     // світлодіод індикації успішної передачі
#define PIN_RF_CE   9     // CE  модуля nRF24L01
#define PIN_RF_CSN  10    // CSN модуля nRF24L01 (SPI: D11, D12, D13)
#define DHT_TYPE    DHT11

// ---------- Параметри радіоканалу ----------
const uint8_t RF_CHANNEL      = 80;       // 2400 + 80 = 2480 МГц (верх ISM-діапазону)
const byte    PIPE_ADDRESS[6] = "TH001";  // 5-байтова адреса каналу

// ---------- Параметри роботи ----------
const unsigned long SEND_INTERVAL_MS = 2000; // період вимірювання і передачі
const float TEMP_OFFSET = 0.5;  // калібрувальна поправка температури, °C
const float HUM_OFFSET  = 0.0;  // калібрувальна поправка вологості, %

// ---------- Структура пакета даних (13 байт) ----------
struct SensorPacket {
  uint32_t packetId;     // порядковий номер пакета
  float    temperature;  // температура, °C
  float    humidity;     // відносна вологість, %
  uint8_t  status;       // 0 - дані коректні, 1 - помилка датчика
};

// ---------- Прототипи функцій ----------
void readSensor();
void sendPacket();
void blinkLed();

RF24 radio(PIN_RF_CE, PIN_RF_CSN);
DHT  dht(PIN_DHT, DHT_TYPE);

SensorPacket  packet;
unsigned long lastSend = 0;
uint32_t      sentOk = 0, sentFail = 0;

void setup() {
  Serial.begin(9600);
  pinMode(PIN_LED_TX, OUTPUT);
  dht.begin();

  // Ініціалізація радіомодуля; при помилці - аварійна індикація
  if (!radio.begin()) {
    Serial.println(F("nRF24L01 not responding! Check wiring and 3.3V power."));
    while (true) {
      digitalWrite(PIN_LED_TX, !digitalRead(PIN_LED_TX));
      delay(100);
    }
  }
  radio.setChannel(RF_CHANNEL);
  radio.setDataRate(RF24_250KBPS);          // мін. швидкість -> макс. дальність
  radio.setPALevel(RF24_PA_HIGH);           // потужність передавача
  radio.setPayloadSize(sizeof(SensorPacket));
  radio.setRetries(5, 15);                  // до 15 повторів через 1.5 мс
  radio.openWritingPipe(PIPE_ADDRESS);
  radio.stopListening();                    // режим передавача (PTX)

  packet.packetId = 0;
  Serial.println(F("Transmitter ready"));
}

void loop() {
  // Неблокуюче планування: вимірювання і передача кожні SEND_INTERVAL_MS
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();
    readSensor();
    sendPacket();
  }
}

// Зчитує DHT11 і заповнює поля пакета
void readSensor() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();          // градуси Цельсія

  if (isnan(h) || isnan(t)) {               // датчик не відповів
    packet.status      = 1;
    packet.temperature = 0;
    packet.humidity    = 0;
    Serial.println(F("DHT11 read error"));
  } else {
    packet.status      = 0;
    packet.temperature = t + TEMP_OFFSET;
    packet.humidity    = constrain(h + HUM_OFFSET, 0, 100);
  }
  packet.packetId++;
}

// Надсилає пакет і перевіряє підтвердження (ACK) від приймача
void sendPacket() {
  unsigned long t0 = micros();
  bool ok = radio.write(&packet, sizeof(packet));
  unsigned long dt = micros() - t0;         // час передачі з очікуванням ACK

  if (ok) { sentOk++; blinkLed(); }
  else    { sentFail++; }

  // Журнал у Serial Monitor
  Serial.print(F("#"));       Serial.print(packet.packetId);
  Serial.print(F(" T="));     Serial.print(packet.temperature, 1);
  Serial.print(F(" H="));     Serial.print(packet.humidity, 1);
  Serial.print(ok ? F(" ACK ") : F(" FAIL "));
  Serial.print(dt);           Serial.print(F(" us, ok/fail="));
  Serial.print(sentOk);       Serial.print('/');
  Serial.println(sentFail);
}

// Короткий спалах світлодіода після успішної передачі
void blinkLed() {
  digitalWrite(PIN_LED_TX, HIGH);
  delay(50);
  digitalWrite(PIN_LED_TX, LOW);
}
