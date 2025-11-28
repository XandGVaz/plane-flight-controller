#ifndef DISPLAY_HPP
#define DISPLAY_HPP

/*===============================================================================*/
// Bibliotecas de implementação das funções da ArduinoIDE
#include <Arduino.h> // funções arduino
#include <Wire.h>    // i2c

/*===============================================================================*/
// Biblioteca de implementação do display
#include <LiquidCrystal_I2C.h>      // autor: marcoschwartz

/*===============================================================================*/
// Defines
#define CI_ADDR1 0x27   // lcd1602 que usa PCF8574T, endereço I2C é 0x27 
#define CI_ADDR2 0x3F   // lcd1602 que usa PCF8574AT, endereço I2c é 0x3F

/*===============================================================================*/
// Classe display

class Display16x2{
  LiquidCrystal_I2C* display = NULL;
  uint8_t sdaPin;
  uint8_t sclPin;
  bool i2cAddrTest(byte addr);
 public:
  Display16x2(uint8_t sdaPin, uint8_t sclPin);
  bool setup();
  void updateMessage(String value);
  void writeMessage(String value, uint8_t line);
  void clear();
};

#endif