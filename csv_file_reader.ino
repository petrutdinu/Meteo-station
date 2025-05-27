#include <SPI.h>
#include <SD.h>

#define SD_CS 10    // chip-select pin for your SD module

void setup() {
  Serial.begin(9600);
  while (!Serial);                // wait for Serial
  Serial.println(F("Initializing SD card..."));

  if (!SD.begin(SD_CS)) {
    Serial.println(F("ERROR: SD init failed!"));
    while (1);
  }
  Serial.println(F("SD init OK"));

  // try to open our log file
  File logFile = SD.open("log.csv");
  if (!logFile) {
    Serial.println(F("ERROR: log.csv not found"));
    while (1);
  }

  Serial.println(F("Contents of log.csv:"));
  // read and print every line
  while (logFile.available()) {
    String line = logFile.readStringUntil('\n');
    Serial.println(line);
  }

  logFile.close();
  Serial.println(F("Done reading log.csv."));
}

void loop() {
  // nothing else to do
}
