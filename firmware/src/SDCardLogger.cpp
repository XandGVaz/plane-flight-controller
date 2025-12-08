#include "SDCardLogger.h"
#include <Arduino.h>

// Constructor: stores the CS pin
SDCardLogger::SDCardLogger(const int cs_pin) {
  csPin = cs_pin;
}

// Initializes the SD module. Returns true on success.
bool SDCardLogger::begin() {
  Serial.println("Initializing SD card...");

  // Initializes SPI for the SD Card
  // Note: SCK, MISO, and MOSI pins are standard for SPI on the ESP32.
  if (!SD.begin(csPin)) {
    Serial.println("Card Mount Failed");
    return false;
  }

  const uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return false;
  }

    String fileName;
    int fileIndex;

    // Procura um nome disponível entre Flightdata1.txt e Flightdata100.txt
    for (fileIndex = 1; fileIndex <= 100; fileIndex++) {
        fileName = "/Flightdata" + String(fileIndex) + ".txt";
        if (!SD.exists(fileName)) {
            break; // achou um nome livre
        }
    }

    // Se chegou em 101, significa que todos de 1 a 100 existem
    if (fileIndex > 100) {
        fileIndex = 1; // volta para Flightdata1
        fileName = "/Flightdata" + String(fileIndex) + ".txt";
    }

    logFile = SD.open(fileName, FILE_WRITE);

  if(!logFile){
    Serial.println("Failed to open log file for writing");
    return false;
  }

  // Escreve o cabeçalho
  const String logHeader = "Millis,State,"
                     "BMP_Alt,BMP_MaxAlt,BMP_Velocity,"
                     "MPU_AccelX,MPU_AccelY,MPU_AccelZ,"
                     "MPU_GyroX,MPU_GyroY,MPU_GyroZ,"
                     "MPU_Temp,"
                     "GPS_Lat,GPS_Long,GPS_Alt";
                     
  logFile.println(logHeader);
  logFile.flush(); // garante que foi salvo
  
  Serial.println("SD card initialized and log file created.");
  return true;
}

// Writes a line of text to the log file
void SDCardLogger::writeLine(const String &data) {
  if (logFile) {
    logFile.println(data);
  } else {
    Serial.println("Log file is not open!");
  }
}

void SDCardLogger::FlushPackage() {
  logFile.flush();
}

// Closes the log file
void SDCardLogger::closeFile() {
  if (logFile) {
    logFile.close();
    Serial.println("Log file closed.");
  }
}