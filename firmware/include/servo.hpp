#ifndef SERVO_HPP
#define SERVO_HPP

/*===============================================================================*/
// Bibliotecas de implementação das funções da ArduinoIDE
#include <Arduino.h> // funções arduino

/*===============================================================================*/
// Defines
#define SERVO_PWM_FREQ 50           // hertz
#define SERVO_PWM_PERIOD 20000      // microssegundos
#define SERVO_PWM_BITS 13           // resolução pwm de 0 a 8191

/*===============================================================================*/
// Operação matemática angle -> duty cicle
#define angleToDutyCicle(angle, servoPwmBits, servoPwmPeriod) \
  (angle / 180.0)*(2500.0/servoPwmPeriod)*(1U << servoPwmBits)

/*===============================================================================*/
// Classe do servo
class Servo{
  uint8_t servoPin;
  uint8_t pwmChannel;
 public:
  Servo(uint8_t servoPin, uint8_t pwmChannel);
  bool setup();
  bool setAngle(uint8_t angle);
};

#endif