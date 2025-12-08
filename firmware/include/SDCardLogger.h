#ifndef SDCARD_LOGGER_H
#define SDCARD_LOGGER_H

#include "FS.h"
#include "SD.h"

class SDCardLogger {
private:
  // Chip Select (CS) pin for the SD card reader
 int csPin;
  // Object for the log file
 File logFile;

public:
  // Constructor: receives the reader's CS pin
 SDCardLogger(int cs_pin);

  // Initializes the SD module. Returns true on success.
 bool begin();

  // Writes a line of text to the log file
 void writeLine(const String &data);

  // Writes the last 10 data packages
 void FlushPackage();

  // Closes the log file
 void closeFile();
};

#endif