#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>

// ==== Hardware configuration ====
#define TOUCH_PIN      2
#define RED_LED_PIN    7
#define GREEN_LED_PIN  8
#define SD_CS          10
#define LOG_INTERVAL   10000UL  // 10 seconds

Adafruit_BME280 bme;
LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;

// ==== State ====
int currentMode = 0;
bool lastTouchState = LOW;

// ==== Custom “block” chars ====
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

// draw one big-digit
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

// draw clock
void drawClock(byte H, byte M) {
  if (H/10==0) drawDig(10,0,0);
  else         drawDig(H/10,0,0);
  drawDig(H%10,4,0);
  lcd.setCursor(8,0); lcd.write((byte)223);
  lcd.setCursor(8,1); lcd.write((byte)223);
  drawDig(M/10,9,0);
  drawDig(M%10,13,0);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(TOUCH_PIN, INPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN,   LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);

  lcd.begin(16,2);
  lcd.backlight();
  lcd.clear();
  loadClockChars();

  // BME280
  if (!bme.begin(0x76) && !bme.begin(0x77)) {
    Serial.println("BME280 init error");
    lcd.clear(); lcd.print("BME280 error");
    while(1);
  }
  Serial.println("BME280 OK");

  // RTC
  if (!rtc.begin()) {
    Serial.println("RTC init error");
    lcd.clear(); lcd.print("RTC error");
    while(1);
  }
  rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));
  Serial.println("RTC OK");

  // SD
  if (!SD.begin(SD_CS)) {
    Serial.println("SD init error");
    lcd.clear(); lcd.print("SD error");
  } else {
    Serial.println("SD OK");
  }

  Serial.println("Setup complete");
}

void loop() {
  // read sensors
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure()/100.0F;
  int threshold = analogRead(A0)*50/1023;
  Serial.print("T="); Serial.print(t,1);
  Serial.print("C, TH="); Serial.print(threshold);
  Serial.print("C, H="); Serial.print(h,1);
  Serial.print("%, P="); Serial.print(p,1);
  Serial.println("hPa");

  // temp alert + LED
  if (t > threshold) {
    digitalWrite(RED_LED_PIN,HIGH);
    digitalWrite(GREEN_LED_PIN,LOW);
    Serial.println("!!! TEMP ALERT !!!");
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("!!! TEMP ALERT !!!");
    lcd.setCursor(0,1); 
    lcd.print("T>"); lcd.print(threshold);
    lcd.print((char)223); lcd.print("C");
    delay(2000);
    lcd.clear();
  } else {
    digitalWrite(RED_LED_PIN,LOW);
    digitalWrite(GREEN_LED_PIN,HIGH);
  }

  // log to SD
  static unsigned long lastLog=0;
  static float sumT=0,sumH=0,sumP=0;
  static int cnt=0;
  unsigned long nowMillis=millis();
  sumT+=t; sumH+=h; sumP+=p; cnt++;
  if (nowMillis-lastLog>=LOG_INTERVAL && cnt>0) {
    DateTime dt=rtc.now();
    File f=SD.open("log.csv",FILE_WRITE);
    if(f) {
      if(f.size()==0) f.println("Date,Time,Temp,Hum,Pres");
      // date/time
      if(dt.month()<10) f.print('0');
      f.print(dt.month()); f.print('/');
      if(dt.day()<10) f.print('0');
      f.print(dt.day()); f.print('/');
      f.print(dt.year()); f.print(',');
      if(dt.hour()<10) f.print('0');
      f.print(dt.hour()); f.print(':');
      if(dt.minute()<10) f.print('0');
      f.print(dt.minute()); f.print(':');
      if(dt.second()<10) f.print('0');
      f.print(dt.second()); f.print(',');
      // averages
      f.print(sumT/cnt,1); f.print(',');
      f.print(sumH/cnt,1); f.print(',');
      f.println(sumP/cnt,1);
      f.close();

      // feedback LCD + Serial
      Serial.println(">> Logged to SD");
      lcd.clear();
      lcd.setCursor(2,0);
      lcd.print("Saved to SD !");
      delay(1000);
      lcd.clear();
    }
    sumT=sumH=sumP=0; cnt=0; lastLog=nowMillis;
  }

  // mode switch
  bool touch=digitalRead(TOUCH_PIN);
  if(touch && !lastTouchState) {
    currentMode=(currentMode+1)%6;
    Serial.print("Mode: "); Serial.println(currentMode);
    lcd.clear();
    delay(200);
  }
  lastTouchState=touch;

  // display
  char buf[8];
  switch(currentMode) {
    case 0:
      Serial.println("Display: Temperature");
      lcd.setCursor(0,0); lcd.print("Temperature");
      dtostrf(t,5,1,buf);
      { int pos=(16-strlen(buf)-1)/2;
        lcd.setCursor(pos,1);
        lcd.print(buf); lcd.print((char)223);
      }
      break;

    case 1:
      Serial.println("Display: Threshold");
      lcd.setCursor(0,0); lcd.print("Threshold");
      sprintf(buf,"%d",threshold);
      { int pos=(16-strlen(buf)-1)/2;
        lcd.setCursor(pos,1);
        lcd.print(buf); lcd.print((char)223);
      }
      break;

    case 2:
      Serial.println("Display: Humidity");
      lcd.setCursor(0,0); lcd.print("Humidity");
      dtostrf(h,5,1,buf);
      { int pos=(16-strlen(buf)-2)/2;
        lcd.setCursor(pos,1);
        lcd.print(buf); lcd.print(" %");
      }
      break;

    case 3:
      Serial.println("Display: Pressure");
      lcd.setCursor(0,0); lcd.print("Pressure");
      dtostrf(p,6,1,buf);
      { int pos=(16-strlen(buf)-3)/2;
        lcd.setCursor(pos,1);
        lcd.print(buf); lcd.print(" hPa");
      }
      break;

    case 4: {
      Serial.println("Display: Clock");
      DateTime dt=rtc.now();
      drawClock(dt.hour(),dt.minute());
      break;
    }

    case 5: {
      Serial.println("Display: Date & Weekday");
      DateTime dt=rtc.now();
      int d=dt.dayOfTheWeek();
      lcd.setCursor(0,0);
      lcd.print(daysOfWeek[d]);
      sprintf(buf,"%02d/%02d/%04d",dt.day(),dt.month(),dt.year());
      lcd.setCursor((16-10)/2,1);
      lcd.print(buf);
      break;
    }
  }

  delay(150);
}
