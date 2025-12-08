
/*===============================================================================*/
// Inclusão de bibliotecas necessárias

// Biblioteca Arduino
#include <Arduino.h>

// Biblioteca do servo
#include "servo.hpp"

// Biblioteca para display
#include "display.hpp"

// Biblioteca para cartão SD
#include "SDCardLogger.h"

// Biblioteca FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

/*===============================================================================*/
// Pinagem dos componentes

// Pinos Display
#define DISPLAY_SDA_PIN 21
#define DISPLAY_SCL_PIN 22

// Pinos cartão SD
#define SD_CS        15
/* Os pinos padrões na biblioteca SD usada em "SDCardLogger.h"  para ESP32 são:
   SCK  = 14
   MISO = 12
   MOSI = 13  
*/

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

// Pinos Joystick
#define JOYSTICK_ADC_X_PIN 34
#define JOYSTICK_ADC_y_PIN 35

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

/*===============================================================================*/
// Variáveis FreeRTOS

// Tasks FreeRTOS
xTaskHandle xDisplayWriteTaskHandle = NULL;
xTaskHandle xSDCardSaveTaskHandle = NULL;
xTaskHandle xJoystickReadTaskHandle = NULL;
xTaskHandle xControlTaskHandle = NULL;

// Prototypes de funções das tasks
void vDisplayWriteTask(void *pvParameters);
void vSDCardSaveTask(void *pvParameters);
void vJoystickReadTask(void *pvParameters);
void vControlTask(void *pvParameters);

// Prototype de callback de timer de leitura do joystick
void vJoystickReadTimerCallback(TimerHandle_t xTimer);

// Queues FreeRTOS
xQueueHandle xFlightStateToDisplay = NULL;
xQueueHandle xFlightStateToSDCard = NULL;
xQueueHandle xFlightStateToControl = NULL;

// Timers FreeRTOS
xTimerHandle xJoystickReadTimerHandle = NULL;


/*===============================================================================*/
// Estados de voo

// Enum de possíveis estados de voo
typedef enum{
  PITCH_UP = 0U,
  PITCH_DOWN,
  YAW_RIGHT,
  YAW_LEFT,
}flightState;

// Dado de controle com estado e angulatura do controle
typedef struct{
  flightState state;
  uint16_t xValue;
  uint16_t yValue;
}controlData;

// Textos para estados de voo
String flightStatesTexts[] = {
  "SUBIDA",
  "DESCIDA",
  "DIREITA",
  "ESQUERDA"
};

/*===============================================================================*/

void setup() {
  // Inicialização da serial para debug
  Serial.begin(115200);

  // Inicialização do display
  if(!Display.setup()){
    Serial.println("Erro na inicialização do display I2C!");
    while(1);
  } 

  // Inicialização dos servos
  if(!FlapLeftServo.setup()){
    Serial.println("Erro na inicialização do servo Flap Esquerdo!");
    while(1);
  }
  if(!FlapRightServo.setup()){
    Serial.println("Erro na inicialização do servo Flap Direito!");
    while(1);
  }
  if(!AileronLeftServo.setup()){
    Serial.println("Erro na inicialização do servo Aileron Esquerdo!");
    while(1);
  }
  if(!AileronRightServo.setup()){
    Serial.println("Erro na inicialização do servo Aileron Direito!");
    while(1);
  }
  if(!ElevatorLeftServo.setup()){
    Serial.println("Erro na inicialização do servo Profundor Esquerdo!");
    while(1);
  }
  if(!ElevatorRightServo.setup()){
    Serial.println("Erro na inicialização do servo Profundor Direito!");
    while(1);
  }
  if(!RudderServo.setup()){
    Serial.println("Erro na inicialização do servo Leme!");
    while(1);
  }

  // Inicialização do cartão SD
  SDCardLogger sdLogger(SD_CS);
  if(!sdLogger.begin()){
    Serial.println("Erro na inicialização do cartão SD!");
    while(1);
  } 

  // Criação das filas
  xFlightStateToDisplay = xQueueCreate(5, sizeof(flightState));
  xFlightStateToSDCard = xQueueCreate(5, sizeof(flightState));
  xFlightStateToControl = xQueueCreate(5, sizeof(controlData));

  // Criação dos software timers
  xJoystickReadTimerHandle = xTimerCreate("JOYSTICK_READ_TIMER", pdMS_TO_TICKS(100), pdTRUE, NULL, vJoystickReadTimerCallback);
  if(xJoystickReadTaskHandle == NULL){
    Serial.println("Erro na criação do timer de leitura do joystick!");
    while(1);
  }

  // Criação das tasks
  xTaskCreatePinnedToCore(vDisplayWriteTask, "DISPLAY_WRITE_TASK", 4096, NULL, 1, &xDisplayWriteTaskHandle, APP_CPU_NUM);
  if(xDisplayWriteTaskHandle == NULL){
    Serial.println("Erro na criação da task de escrita no display!");
    while(1);
  }
  xTaskCreatePinnedToCore(vSDCardSaveTask, "SDCARD_SAVE_TASK", 8192, NULL, 1, &xSDCardSaveTaskHandle, APP_CPU_NUM);
  if(xSDCardSaveTaskHandle == NULL){
    Serial.println("Erro na criação da task de salvamento no cartão SD!");
    while(1);
  }
  xTaskCreatePinnedToCore(vJoystickReadTask, "JOYSTICK_READ_TASK", 4096, NULL, 1, &xJoystickReadTaskHandle, PRO_CPU_NUM);
  if(xJoystickReadTaskHandle == NULL){
    Serial.println("Erro na criação da task de leitura do joystick!");
    while(1);
  }
  xTaskCreatePinnedToCore(vControlTask, "CONTROL_TASK", 8192, NULL, 1, &xControlTaskHandle, PRO_CPU_NUM);
  if(xControlTaskHandle == NULL){
    Serial.println("Erro na criação da task de controle dos servos!");
    while(1);
  }

}

void loop() {
  vTaskDelete(NULL);
}

/*===============================================================================*/
// Definição da callback do timer de leitura do joystick

void vJoystickReadTimerCallback(TimerHandle_t xTimer){
  // Notificação para a task de leitura do joystick
  vTaskNotifyGiveFromISR(xJoystickReadTaskHandle, NULL);
}

/*===============================================================================*/
// Definição das tasks FreeRTOS

// Task de escrita no display
void vDisplayWriteTask(void *pvParameters){
  // Variável para receber o estado de voo da fila
  flightState receivedFlightState;
  
  while(1){
    // Espera por um estado de voo na fila
    xQueueReceive(xFlightStateToDisplay, &receivedFlightState, 0);
    
    // Escreve o estado de voo no display
    Display.clear();
    Display.writeMessage("Estado Voo:", 0);
    Display.writeMessage(flightStatesTexts[receivedFlightState],1);
    
    // Pequena pausa para evitar sobrecarga de escrita no display
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// Task de salvamento no cartão SD
void vSDCardSaveTask(void *pvParameters){
  // Variável para receber o estado de voo da fila
  flightState receivedFlightState;

  // Instanciação do logger do cartão SD
  SDCardLogger sdLogger(SD_CS);

  while(1){
    // Espera por um estado de voo na fila
    if(xQueueReceive(xFlightStateToSDCard, &receivedFlightState, portMAX_DELAY) == pdTRUE){
      // Formata a linha de dados para salvar
      String dataLine = String(millis()) + "," + flightStatesTexts[receivedFlightState];
      
      // Salva a linha no cartão SD
      sdLogger.writeLine(dataLine);
    }
  }
}

// Task de leitura do joystick
void vJoystickReadTask(void *pvParameters){
  // Inicialização do timer de leitura do joystick
  xTimerStart(xJoystickReadTimerHandle, 0);

  // Variável para receber dado de controle do joystick
  controlData controlData;

  while(1){
    // Espera pela notificação do timer
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Leitura dos valores do joystick
    controlData.xValue = analogRead(JOYSTICK_ADC_X_PIN);
    controlData.yValue = analogRead(JOYSTICK_ADC_y_PIN);

    // Determinação do estado de voo baseado nos valores lidos
    flightState currentFlightState;
    if(controlData.yValue > 3000){
      currentFlightState = PITCH_UP;
    } 
    else if(controlData.yValue < 1000){
      currentFlightState = PITCH_DOWN;
    } 
    else if(controlData.xValue > 3000){
      currentFlightState = YAW_RIGHT;
    } 
    else if(controlData.xValue < 1000){
      currentFlightState = YAW_LEFT;
    } 
    else {
      continue; // Nenhuma ação detectada
    }

    // Atribuição do estado de voo ao dado de controle
    controlData.state = currentFlightState;

    // Envio do estado de voo para as filas correspondentes
    xQueueSend(xFlightStateToDisplay, &currentFlightState, 0);
    xQueueSend(xFlightStateToSDCard, &currentFlightState, 0);
    xQueueSend(xFlightStateToControl, &controlData, 0);
  }
}


// Task de controle dos servos
void vControlTask(void *pvParameters){
  // Variável para receber o estado de voo da fila
  controlData receivedControlData;

  // Seta todos os servos na posição neutra
  FlapLeftServo.setAngle(90);
  FlapRightServo.setAngle(90);
  AileronLeftServo.setAngle(90);
  AileronRightServo.setAngle(90);
  ElevatorLeftServo.setAngle(90);
  ElevatorRightServo.setAngle(90);
  RudderServo.setAngle(90);

  while(1){
    // Espera por um estado de voo na fila
    if(xQueueReceive(xFlightStateToControl, &receivedControlData, portMAX_DELAY) == pdTRUE){
      
      // Controle dos servos baseado no estado de voo recebido
      switch(receivedControlData.state){
        // Controle para subida
        case PITCH_UP:
          FlapLeftServo.setAngle(90);         // flap esquerdo neutro
          FlapRightServo.setAngle(90);        // flap direito neutro
          AileronLeftServo.setAngle(90);      // aileron esquerdo neutro
          AileronRightServo.setAngle(90);     // aileron direito neutro
          ElevatorLeftServo.setAngle(120);    // profundor esquerdo para cima
          ElevatorRightServo.setAngle(120);   // profundor direito para cima
          RudderServo.setAngle(90);           // leme neutro
          break;
        
        // Controle para descida
        case PITCH_DOWN:
          FlapLeftServo.setAngle(90);         // flap esquerdo neutro
          FlapRightServo.setAngle(90);        // flap direito neutro
          AileronLeftServo.setAngle(90);      // aileron esquerdo neutro
          AileronRightServo.setAngle(90);     // aileron direito neutro
          ElevatorLeftServo.setAngle(60);     // profundor esquerdo para baixo
          ElevatorRightServo.setAngle(60);    // profundor direito para baixo
          RudderServo.setAngle(90);           // leme neutro
          break;
        
        // Controle para virar à direita
        case YAW_RIGHT:
          FlapLeftServo.setAngle(90);         // flap esquerdo neutro
          FlapRightServo.setAngle(90);        // flap direito neutro
          AileronLeftServo.setAngle(90);      // aileron esquerdo neutro
          AileronRightServo.setAngle(90);     // aileron direito neutro
          ElevatorLeftServo.setAngle(90);     // profundor esquerdo neutro
          ElevatorRightServo.setAngle(90);    // profundor direito neutro
          RudderServo.setAngle(75);           // leme para a direita
          break;
        
        // Controle para virar à esquerda
        case YAW_LEFT:
          FlapLeftServo.setAngle(90);         // flap esquerdo neutro
          FlapRightServo.setAngle(90);        // flap direito neutro
          AileronLeftServo.setAngle(90);      // aileron esquerdo neutro
          AileronRightServo.setAngle(90);     // aileron direito neutro
          ElevatorLeftServo.setAngle(90);     // profundor esquerdo neutro
          ElevatorRightServo.setAngle(90);    // profundor direito neutro
          RudderServo.setAngle(105);          // leme para a esquerda
          break;
        
        default:
          break;
      }
    }
  }
}