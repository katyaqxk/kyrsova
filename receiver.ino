/*
 * receiver.ino — приймальний вузол (вузол відображення)
 * Проєкт: Дистанційна передача даних датчика температури і вологості
 *         на модулі зв'язку 2.4 GHz (nRF24L01) на OLED-дисплей
 * Плата:  Arduino Uno R3 (ATmega328P)
 * Бібліотеки:
 *   RF24 by TMRh20       v1.4.9
 *   Adafruit SSD1306     v2.5.9
 *   Adafruit GFX Library v1.11.9
 * Ліцензія: MIT
 */
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- Призначення пінів ----------
#define PIN_RF_CE   9     // CE  модуля nRF24L01
#define PIN_RF_CSN  10    // CSN модуля nRF24L01 (SPI: D11, D12, D13)
// OLED підключено до апаратної шини I2C: SDA - A4, SCL - A5

// ---------- Параметри дисплея ----------
#define SCREEN_W    128
#define SCREEN_H    64
#define OLED_ADDR   0x3C
#define OLED_RESET  -1    // окремий вивід RESET не використовується

// ---------- Параметри радіоканалу (мають збігатися з передавачем) ----------
const uint8_t RF_CHANNEL      = 80;
const byte    PIPE_ADDRESS[6] = "TH001";

// ---------- Параметри роботи ----------
const unsigned long LINK_TIMEOUT_MS   = 6000; // 3 пропущені пакети -> зв'язку немає
const unsigned long DISPLAY_PERIOD_MS = 500;  // період оновлення екрана

// ---------- Структура пакета даних (ідентична передавачу) ----------
struct SensorPacket {
  uint32_t packetId;
  float    temperature;
  float    humidity;
  uint8_t  status;
};

// ---------- Прототипи функцій ----------
void receivePacket();
void drawScreen();
void showMessage(const __FlashStringHelper *l1, const __FlashStringHelper *l2);

RF24 radio(PIN_RF_CE, PIN_RF_CSN);
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);

SensorPacket  packet;
bool          hasData      = false;  // чи було прийнято хоча б один пакет
bool          strongSignal = false;  // рівень сигналу > -64 dBm (RPD)
unsigned long lastRx = 0, lastDraw = 0;
uint32_t      received = 0, lost = 0, lastId = 0;

void setup() {
  Serial.begin(9600);

  // Ініціалізація OLED-дисплея
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 not found! Check I2C wiring/address."));
    while (true) {}
  }
  display.cp437(true);                 // кодова таблиця з символом градуса
  display.setTextColor(SSD1306_WHITE);
  showMessage(F("2.4GHz T/H monitor"), F("Starting..."));

  // Ініціалізація радіомодуля
  if (!radio.begin()) {
    Serial.println(F("nRF24L01 not responding!"));
    showMessage(F("RF MODULE ERROR"), F("Check wiring/3.3V"));
    while (true) {}
  }
  radio.setChannel(RF_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setPayloadSize(sizeof(SensorPacket));
  radio.openReadingPipe(1, PIPE_ADDRESS);
  radio.startListening();              // режим приймача (PRX)

  Serial.println(F("Receiver ready"));
}

void loop() {
  if (radio.available()) {
    receivePacket();
  }
  if (millis() - lastDraw >= DISPLAY_PERIOD_MS) {
    lastDraw = millis();
    drawScreen();
  }
}

// Читає пакет з радіомодуля і оновлює статистику зв'язку
void receivePacket() {
  radio.read(&packet, sizeof(packet));
  strongSignal = radio.testRPD();      // true, якщо сигнал сильніший за -64 dBm

  if (hasData && packet.packetId <= lastId) {
    received = 0;                      // передавач перезапущено - скидання
    lost     = 0;
  } else if (hasData && packet.packetId > lastId + 1) {
    lost += packet.packetId - lastId - 1;   // пропущені номери = втрачені пакети
  }
  lastId  = packet.packetId;
  received++;
  lastRx  = millis();
  hasData = true;

  Serial.print(F("RX #")); Serial.print(packet.packetId);
  Serial.print(F(" T="));  Serial.print(packet.temperature, 1);
  Serial.print(F(" H="));  Serial.print(packet.humidity, 1);
  Serial.print(F(" status=")); Serial.println(packet.status);
}

// Формує зображення на OLED-дисплеї
void drawScreen() {
  bool linkOk = hasData && (millis() - lastRx < LINK_TIMEOUT_MS);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("Link: "));
  display.print(linkOk ? F("OK") : F("LOST"));
  display.setCursor(96, 0);
  display.print(strongSignal && linkOk ? F("RF+") : F("RF-"));
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  if (!hasData) {
    display.setCursor(0, 28);
    display.print(F("Waiting for data..."));
  } else if (packet.status != 0) {
    display.setTextSize(2);
    display.setCursor(0, 16); display.print(F("SENSOR"));
    display.setCursor(0, 34); display.print(F("ERROR"));
  } else {
    display.setTextSize(2);
    display.setCursor(0, 15);
    display.print(F("T:"));
    display.print(packet.temperature, 1);
    display.write(248);                // символ градуса
    display.print(F("C"));
    display.setCursor(0, 35);
    display.print(F("H:"));
    display.print(packet.humidity, 0);
    display.print(F(" %"));
  }

  // Нижній рядок: номер пакета і відсоток втрат
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print(F("#"));
  display.print(lastId);
  display.print(F("  Loss:"));
  uint32_t total = received + lost;
  display.print(total ? (lost * 100.0 / total) : 0.0, 1);
  display.print(F("%"));
  display.display();
}

// Виводить два рядки службового повідомлення
void showMessage(const __FlashStringHelper *l1, const __FlashStringHelper *l2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 20); display.print(l1);
  display.setCursor(0, 36); display.print(l2);
  display.display();
}
