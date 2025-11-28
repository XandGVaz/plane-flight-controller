
/*===============================================================================*/
// Inclusão de bibliotecas necessárias

// Biblioteca Arduino
#include <Arduino.h>

// Biblioteca do servo
#include "servo.hpp"

// Biblioteca para display
#include "display.hpp"

/*===============================================================================*/
// Pinagem dos componentes

// Display
#define DISPLAY_SDA_PIN 21
#define DISPLAY_SCL_PIN 22

// Pinos servo flaps 
#define FLAP_LEFT_SERVO_PIN 2
#define FLAP_LEFT_SERVO_CHANNEL 0
#define FLAP_RIGHT_SERVO_PIN 3
#define FLAP_RIGHT_SERVO_CHANNEL 1

// Pinos servo ailerons
#define AILERON_LEFT_SERVO_PIN 4
#define AILERON_LEFT_SERVO_CHANNEL 2
#define AILERON_RIGHT_SERVO_PIN 5
#define AILERON_RIGHT_SERVO_CHANNEL 3

// Pinos servo profundor
#define ELEVATOR_LEFT_SERVO_PIN 6
#define ELEVATOR_LEFT_SERVO_CHANNEL 4
#define ELEVATOR_RIGHT_SERVO_PIN 7
#define ELEVATOR_RIGHT_SERVO_CHANNEL 5

// Pino servo leme
#define RUDDER_SERVO_PIN 8
#define RUDDER_SERVO_CHANNEL 6

/*===============================================================================*/
// Instanciação dos módulos

// Display
Display16x2 Display(DISPLAY_SDA_PIN, DISPLAY_SCL_PIN);

// Servos flaps
Servo FlapLeftServo(FLAP_LEFT_SERVO_PIN, FLAP_LEFT_SERVO_CHANNEL);
Servo FlapRightServo(FLAP_RIGHT_SERVO_PIN, FLAP_RIGHT_SERVO_CHANNEL);

// Servos ailerons
Servo AileronLeftServo(AILERON_LEFT_SERVO_PIN, AILERON_LEFT_SERVO_CHANNEL);
Servo AileronRightServo(AILERON_RIGHT_SERVO_PIN, AILERON_RIGHT_SERVO_CHANNEL);

// Servos profundor
Servo ElevatorLeftServo(ELEVATOR_LEFT_SERVO_PIN, ELEVATOR_LEFT_SERVO_CHANNEL);
Servo ElevatorRightServo(ELEVATOR_RIGHT_SERVO_PIN, ELEVATOR_RIGHT_SERVO_CHANNEL);

// Servo leme
Servo RudderServo(RUDDER_SERVO_PIN, RUDDER_SERVO_CHANNEL);


void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}
