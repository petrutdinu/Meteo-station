#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>

#define TOUCH_PIN 2
#define LED_PIN 7
#define SD_CS 10
#define LOG_INTERVAL 10000  // 10 secunde

Adafruit_BME280 bme;
LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;

int currentMode = 0;
bool lastTouchState = LOW;

unsigned long lastLogTime = 0;
float tempSum = 0, humSum = 0, presSum = 0;
int sampleCount = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(TOUCH_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();

  if (!bme.begin(0x76)) {
    lcd.print("Eroare BME280");
    Serial.println("Eroare senzor BME280!");
    while (1);
  }

  if (!rtc.begin()) {
    lcd.print("Eroare RTC");
    Serial.println("Eroare senzor RTC!");
    while (1);
  }

  // Setează ora doar prima dată, apoi comentează linia de mai jos
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  if (!SD.begin(SD_CS)) {
    Serial.println("⚠️ SD card nedetectat!");
    lcd.setCursor(0, 0);
    lcd.print("Eroare SD Card");
  } else {
    Serial.println("✅ SD card OK");
  }

  Serial.println("Sistem pornit.");
}

void loop() {
  // === PRAG DE TEMPERATURĂ ===
  int potValue = analogRead(A0);
  int pragTemp = potValue * 50 / 1023;

  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure() / 100.0F;

  // === ALERTĂ TEMPERATURĂ ===
  if (t > pragTemp) {
    digitalWrite(LED_PIN, HIGH); // aprinde LED
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ALERTA TEMP!");
    lcd.setCursor(0, 1);
    lcd.print("T=");
    lcd.print(t, 1);
    lcd.print((char)223);
    lcd.print("C > ");
    lcd.print(pragTemp);
    lcd.print((char)223);

    Serial.println("⚠️ Temperatura peste prag!");
    Serial.print("T = "); Serial.print(t); Serial.print(" > ");
    Serial.print("prag = "); Serial.println(pragTemp);

    delay(2000);
    lcd.clear();
  } else {
    digitalWrite(LED_PIN, LOW); // stinge LED dacă e sub prag
  }

  // === BUTON MOD AFISAJ ===
  bool touchState = digitalRead(TOUCH_PIN);
  if (touchState == HIGH && lastTouchState == LOW) {
    currentMode++;
    if (currentMode > 3) currentMode = 0;
    lcd.clear();
    delay(200); // debounce
  }
  lastTouchState = touchState;

  // === LCD in functie de mod ===
  if (currentMode == 0) {
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(t, 1);
    lcd.print((char)223);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("Prag: ");
    lcd.print(pragTemp);
    lcd.print((char)223);
    lcd.print("C");

    Serial.print("Temperatura: "); Serial.print(t); Serial.println(" °C");
    Serial.print("Prag setat: "); Serial.println(pragTemp);

  } else if (currentMode == 1) {
    lcd.setCursor(0, 0); lcd.print("Umiditate:");
    lcd.setCursor(0, 1); lcd.print(h, 1); lcd.print(" %");
    Serial.print("Umiditate: "); Serial.print(h); Serial.println(" %");

  } else if (currentMode == 2) {
    lcd.setCursor(0, 0); lcd.print("Presiune:");
    lcd.setCursor(0, 1); lcd.print(p, 1); lcd.print(" hPa");
    Serial.print("Presiune: "); Serial.print(p); Serial.println(" hPa");

  } else if (currentMode == 3) {
    DateTime now = rtc.now();
    lcd.setCursor(0, 0);
    lcd.print("Ora: ");
    if (now.hour() < 10) lcd.print("0"); lcd.print(now.hour()); lcd.print(":");
    if (now.minute() < 10) lcd.print("0"); lcd.print(now.minute()); lcd.print(":");
    if (now.second() < 10) lcd.print("0"); lcd.print(now.second());

    lcd.setCursor(0, 1);
    lcd.print("Data: ");
    lcd.print(now.day()); lcd.print("/");
    lcd.print(now.month()); lcd.print("/");
    lcd.print(now.year());

    Serial.print("Timp: ");
    Serial.print(now.hour()); Serial.print(":");
    Serial.print(now.minute()); Serial.print(":");
    Serial.print(now.second());

    Serial.print(" | Data: ");
    Serial.print(now.day()); Serial.print("/");
    Serial.print(now.month()); Serial.print("/");
    Serial.println(now.year());
  }

  // === Colectare pentru medie ===
  tempSum += t;
  humSum += h;
  presSum += p;
  sampleCount++;

  // === Salvare pe SD la fiecare 10 secunde ===
  if (millis() - lastLogTime >= LOG_INTERVAL && sampleCount > 0) {
    float tAvg = tempSum / sampleCount;
    float hAvg = humSum / sampleCount;
    float pAvg = presSum / sampleCount;

    DateTime now = rtc.now();
    File logFile = SD.open("log.csv", FILE_WRITE);

    if (logFile) {
      if (logFile.size() == 0) {
        logFile.println("Data,Ora,Temperatura (C),Umiditate (%),Presiune (hPa)");
      }

      logFile.print(now.year()); logFile.print("-");
      if (now.month() < 10) logFile.print("0");
      logFile.print(now.month()); logFile.print("-");
      if (now.day() < 10) logFile.print("0");
      logFile.print(now.day()); logFile.print(",");

      if (now.hour() < 10) logFile.print("0");
      logFile.print(now.hour()); logFile.print(":");
      if (now.minute() < 10) logFile.print("0");
      logFile.print(now.minute()); logFile.print(":");
      if (now.second() < 10) logFile.print("0");
      logFile.print(now.second()); logFile.print(",");

      logFile.print(tAvg, 1); logFile.print(",");
      logFile.print(hAvg, 1); logFile.print(",");
      logFile.println(pAvg, 1);
      logFile.close();

      Serial.println("✅ Salvare CSV:");
      Serial.print("T="); Serial.print(tAvg, 1); Serial.print("C, ");
      Serial.print("H="); Serial.print(hAvg, 1); Serial.print("%, ");
      Serial.print("P="); Serial.print(pAvg, 1); Serial.println(" hPa");
    } else {
      Serial.println("⚠️ Eroare la scrierea in log.csv!");
    }

    tempSum = humSum = presSum = 0;
    sampleCount = 0;
    lastLogTime = millis();
  }

  delay(1000);
}
