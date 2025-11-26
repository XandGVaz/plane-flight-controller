#include "servo.hpp"

/*===============================================================================*/
// Definição de métodos do display

// Construtor da classe Servo
Servo::Servo(uint8_t servoPin, uint8_t pwmChannel){
    // Atribuição do pino de sinal pwm do servo
    this->servoPin = servoPin;

    // Atribuição do canal pwm do servo
    this->pwmChannel = pwmChannel;
}

// Método de configuração do servo
bool Servo::setup(){
    // Configuração do canal PWM para controle do servo
    bool result =  ledcAttachChannel(servoPin, SERVO_PWM_FREQ, SERVO_PWM_BITS, pwmChannel);

    // Retorno do resultado da operação
    return result;
}

// Método de definição do ângulo do servo
bool Servo::setAngle(uint8_t angle){
    // Cálculo do duty cycle correspondente ao ângulo desejado
    uint32_t dutyCicle = angleToDutyCicle(angle, SERVO_PWM_BITS, SERVO_PWM_PERIOD);

    // Escrita do duty cycle no canal PWM do servo
    bool result = ledcWrite(servoPin, dutyCicle);

    // Retorno do resultado da operação
    return result;
}