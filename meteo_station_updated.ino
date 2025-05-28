#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>

// ==== Hardware configuration using registers ====
#define TOUCH_BIT       PD2   // digital pin 2
#define RED_LED_BIT     PD7   // digital pin 7
#define GREEN_LED_BIT   PB0   // digital pin 8 (PB0)
#define SD_CS_PIN       10    // digital pin 10 -> PB2
#define LOG_INTERVAL    10000UL  // 10 seconds

// Register shortcuts
#define TOUCH_DDR       DDRD
#define TOUCH_PINREG    PIND
#define RED_DDR         DDRD
#define RED_PORT        PORTD
#define GRN_DDR         DDRB
#define GRN_PORT        PORTB

Adafruit_BME280 bme;
LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;

// Custom “block” chars for big digits
byte LT[8]  = {0b00111,0b01111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111};
byte UB[8]  = {0b11111,0b11111,0b11111,0b00000,0b00000,0b00000,0b00000,0b00000};
byte RT_[8] = {0b11100,0b11110,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111};
byte LL[8]  = {0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b01111,0b00111};
byte LB[8]  = {0b00000,0b00000,0b00000,0b00000,0b00000,0b11111,0b11111,0b11111};
byte LR[8]  = {0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11110,0b11100};
byte UMB[8] = {0b11111,0b11111,0b11111,0b00000,0b00000,0b00000,0b11111,0b11111};
byte LMB[8] = {0b11111,0b00000,0b00000,0b00000,0b00000,0b11111,0b11111,0b11111};

const char* daysOfWeek[7] = {
  "Sunday","Monday","Tuesday","Wednesday",
  "Thursday","Friday","Saturday"
};

void loadClockChars() {
  lcd.createChar(0, LT);
  lcd.createChar(1, UB);
  lcd.createChar(2, RT_);
  lcd.createChar(3, LL);
  lcd.createChar(4, LB);
  lcd.createChar(5, LR);
  lcd.createChar(6, UMB);
  lcd.createChar(7, LMB);
}

// draw one big-digit at (x,y)
void drawDig(byte d, byte x, byte y) {
  switch (d) {
    case 0:
      lcd.setCursor(x,y);   lcd.write(0); lcd.write(1); lcd.write(2);
      lcd.setCursor(x,y+1); lcd.write(3); lcd.write(4); lcd.write(5);
      break;
    case 1:
      lcd.setCursor(x+1,y);   lcd.write(2);
      lcd.setCursor(x+1,y+1); lcd.write(5);
      break;
    case 2:
      lcd.setCursor(x,y);     lcd.write(1); lcd.write(1); lcd.write(2);
      lcd.setCursor(x,y+1);   lcd.write(3); lcd.write(7); lcd.write(7);
      break;
    case 3:
      lcd.setCursor(x,y);     lcd.write(1); lcd.write(1); lcd.write(2);
      lcd.setCursor(x,y+1);   lcd.write(7); lcd.write(7); lcd.write(5);
      break;
    case 4:
      lcd.setCursor(x,y);     lcd.write(3); lcd.write(7); lcd.write(2);
      lcd.setCursor(x+2,y+1); lcd.write(5);
      break;
    case 5:
      lcd.setCursor(x,y);     lcd.write(0); lcd.write(1); lcd.write(1);
      lcd.setCursor(x,y+1);   lcd.write(7); lcd.write(7); lcd.write(5);
      break;
    case 6:
      lcd.setCursor(x,y);     lcd.write(0); lcd.write(1); lcd.write(1);
      lcd.setCursor(x,y+1);   lcd.write(3); lcd.write(7); lcd.write(5);
      break;
    case 7:
      lcd.setCursor(x,y);     lcd.write(1); lcd.write(1); lcd.write(2);
      lcd.setCursor(x+2,y+1); lcd.write(0);
      break;
    case 8:
      lcd.setCursor(x,y);     lcd.write(0); lcd.write(6); lcd.write(2);
      lcd.setCursor(x,y+1);   lcd.write(3); lcd.write(7); lcd.write(5);
      break;
    case 9:
      lcd.setCursor(x,y);     lcd.write(0); lcd.write(6); lcd.write(2);
      lcd.setCursor(x+1,y+1); lcd.write(7); lcd.write(5);
      break;
    case 10:
      for (byte i=0;i<3;i++){
        lcd.setCursor(x+i,y);   lcd.print(' ');
        lcd.setCursor(x+i,y+1); lcd.print(' ');
      }
      break;
  }
}

// draw digital clock HH:MM on LCD
void drawClock(byte H, byte M) {
  if (H/10==0) drawDig(10,0,0);
  else         drawDig(H/10,0,0);
  drawDig(H%10,4,0);
  lcd.setCursor(8,0); lcd.write((byte)223);
  lcd.setCursor(8,1); lcd.write((byte)223);
  drawDig(M/10,9,0);
  drawDig(M%10,13,0);
}

// low-level ADC read on channel 0..7
uint16_t readADC(uint8_t channel) {
  ADMUX = (1<<REFS0) | (channel & 0x07);
  ADCSRA |= (1<<ADSC);
  while (ADCSRA & (1<<ADSC));
  return ADC;
}

// state
int currentMode = 0;
bool lastTouchState = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // GPIO setup using registers
  TOUCH_DDR   &= ~(1<<TOUCH_BIT);    // PD2 input
  RED_DDR     |=  (1<<RED_LED_BIT);  // PD7 output
  GRN_DDR     |=  (1<<GREEN_LED_BIT);// PB0 output
  // Ensure SPI SS pin (PB2) as output for SD
  DDRB        |=  (1<<PB2);

  RED_PORT    &= ~(1<<RED_LED_BIT);  // RED LED LOW
  GRN_PORT    |=  (1<<GREEN_LED_BIT);// GREEN LED HIGH

  // ADC prescaler = 128 -> ~125kHz ADC clock
  ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

  lcd.begin(16,2);
  lcd.backlight();
  lcd.clear();
  loadClockChars();

  // BME280
  if (!bme.begin(0x76) && !bme.begin(0x77)) {
    lcd.clear(); lcd.print("BME280 error");
    while (1);
  }

  // RTC
  if (!rtc.begin()) {
    lcd.clear(); lcd.print("RTC error");
    while (1);
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // SD
  if (!SD.begin(SD_CS_PIN)) {
    lcd.clear(); lcd.print("SD init error");
  }

  Serial.println("Setup complete");
}

void loop() {
  // read sensors
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure() / 100.0F;

  Serial.print("Sensor Readings -> T: "); Serial.print(t); Serial.print(" C, H: ");
  Serial.print(h); Serial.print(" %, P: "); Serial.print(p); Serial.println(" hPa");

  // threshold from ADC0
  uint16_t raw = readADC(0);
  int threshold = raw * 50 / 1023;
  Serial.print("Threshold Set To: "); Serial.print(threshold); Serial.println(" C");

  // read touch pin PD2
  bool touch = (TOUCH_PINREG & (1<<TOUCH_BIT)) != 0;

  // temp alert + LEDs
  if (t > threshold) {
    RED_PORT |=  (1<<RED_LED_BIT);
    GRN_PORT &= ~(1<<GREEN_LED_BIT);
    Serial.println("!!! TEMP ALERT !!!");
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("!!! TEMP ALERT !!!");
    lcd.setCursor(0,1);
    lcd.print("T>"); lcd.print(threshold);
    lcd.print((char)223); lcd.print("C");
    delay(2000);
    lcd.clear();
  } else {
    RED_PORT &= ~(1<<RED_LED_BIT);
    GRN_PORT |=  (1<<GREEN_LED_BIT);
  }

  // logging to SD every LOG_INTERVAL
  static unsigned long lastLog = 0;
  static float sumT=0, sumH=0, sumP=0;
  static int cnt = 0;
  unsigned long nowMillis = millis();

  sumT += t; sumH += h; sumP += p; cnt++;
  if ((nowMillis - lastLog) >= LOG_INTERVAL && cnt > 0) {
    DateTime dt = rtc.now();
    File f = SD.open("log.csv", FILE_WRITE);
    if (f) {
      if (f.size() == 0) f.println("Date,Time,Temp,Hum,Pres");
      // write date/time
      if (dt.month() < 10) f.print('0');
      f.print(dt.month()); f.print('/');
      if (dt.day() < 10) f.print('0');
      f.print(dt.day()); f.print('/');
      f.print(dt.year()); f.print(',');
      if (dt.hour() < 10) f.print('0');
      f.print(dt.hour()); f.print(':');
      if (dt.minute() < 10) f.print('0');
      f.print(dt.minute()); f.print(':');
      if (dt.second() < 10) f.print('0');
      f.print(dt.second()); f.print(',');
      // averages
      float avgT = sumT/cnt;
      float avgH = sumH/cnt;
      float avgP = sumP/cnt;
      f.print(avgT,1); f.print(',');
      f.print(avgH,1); f.print(',');
      f.println(avgP,1);
      f.close();

      Serial.print("Logged to SD -> Avg T: "); Serial.print(avgT); Serial.print(" C, Avg H: ");
      Serial.print(avgH); Serial.print(" %, Avg P: "); Serial.print(avgP); Serial.println(" hPa");

      lcd.clear(); lcd.setCursor(2,0); lcd.print("Saved to SD !");
      delay(1000); lcd.clear();
    }
    sumT=0; sumH=0; sumP=0; cnt=0; lastLog = nowMillis;
  }

  // mode switch on touch press
  if (touch && !lastTouchState) {
    currentMode = (currentMode + 1) % 6;
    Serial.print("Mode changed -> "); Serial.println(currentMode);
    lcd.clear();
    delay(150);
  }
  lastTouchState = touch;

  // display based on mode
  char buf[16];
  switch (currentMode) {
    case 0:
      lcd.setCursor(0,0); lcd.print("Temperature");
      dtostrf(t,5,1,buf);
      lcd.setCursor((16-strlen(buf)-1)/2,1);
      lcd.print(buf); lcd.print((char)223);
      Serial.print("Display Temperature: "); Serial.println(buf);
      break;
    case 1:
      lcd.setCursor(0,0); lcd.print("Threshold");
      sprintf(buf, "%d", threshold);
      lcd.setCursor((16-strlen(buf)-1)/2,1);
      lcd.print(buf); lcd.print((char)223);
      Serial.print("Display Threshold: "); Serial.println(buf);
      break;
    case 2:
      lcd.setCursor(0,0); lcd.print("Humidity");
      dtostrf(h,5,1,buf);
      lcd.setCursor((16-strlen(buf)-2)/2,1);
      lcd.print(buf); lcd.print(" %");
      Serial.print("Display Humidity: "); Serial.print(buf); Serial.println(" %");
      break;
    case 3:
      lcd.setCursor(0,0); lcd.print("Pressure");
      dtostrf(p,6,1,buf);
      lcd.setCursor((16-strlen(buf)-4)/2,1);
      lcd.print(buf); lcd.print(" hPa");
      Serial.print("Display Pressure: "); Serial.print(buf); Serial.println(" hPa");
      break;
    case 4: {
      DateTime dt = rtc.now();
      drawClock(dt.hour(), dt.minute());
      Serial.print("Display Clock: ");
      Serial.print(dt.hour()); Serial.print(":"); Serial.println(dt.minute());
      break;
    }
    case 5: {
      DateTime dt = rtc.now();
      int d = dt.dayOfTheWeek();
      lcd.setCursor(0,0); lcd.print(daysOfWeek[d]);
      sprintf(buf, "%02d/%02d/%04d", dt.day(), dt.month(), dt.year());
      lcd.setCursor((16-10)/2,1); lcd.print(buf);
      Serial.print("Display Date: "); Serial.println(buf);
      break;
    }
  }

  delay(150);
}