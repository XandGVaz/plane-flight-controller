
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
#define FLAP_RIGHT_SERVO_PIN 0
#define FLAP_RIGHT_SERVO_CHANNEL 1

// Pinos servo ailerons
#define AILERON_LEFT_SERVO_PIN 26
#define AILERON_LEFT_SERVO_CHANNEL 2
#define AILERON_RIGHT_SERVO_PIN 25
#define AILERON_RIGHT_SERVO_CHANNEL 3

// Pinos servo profundor
#define ELEVATOR_LEFT_SERVO_PIN 18
#define ELEVATOR_LEFT_SERVO_CHANNEL 4
#define ELEVATOR_RIGHT_SERVO_PIN 19
#define ELEVATOR_RIGHT_SERVO_CHANNEL 5

// Pino servo leme
#define RUDDER_SERVO_PIN 5
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

// Instanciação do logger do cartão SD
SDCardLogger SdLogger(SD_CS);

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
  CRUISE = 0U,
  PITCH_UP,
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

// Macro para converter valor analógico de leitura do joystick em ângulo de variação
#define analogReadToAngleVariation(value) (value > 2048 ? map(value, 2049, 4095, 0, 45) : map(value, 0, 2047, -45, 0))

// Textos para estados de voo
String flightStatesTexts[] = {
  "CRUZEIRO",
  "SUBIDA",
  "DESCIDA",
  "DIREITA",
  "ESQUERDA"
};

/*===============================================================================*/

void setup() {
  // Inicialização da serial para debug
  Serial.begin(115200);
  delay(1000);
  Serial.println("Iniciando o controlador de voo...");

  // Inicialização do display
  if(!Display.setup()){
    Serial.println("Erro na inicialização do display I2C!");
    while(1);
  } 

  // Inicialização do cartão SD
  if(!SdLogger.begin()){
    Serial.println("Erro na inicialização do cartão SD!");
    // while(1);
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


  // Criação das filas
  xFlightStateToDisplay = xQueueCreate(1, sizeof(flightState));
  xFlightStateToSDCard = xQueueCreate(1, sizeof(flightState));
  xFlightStateToControl = xQueueCreate(5, sizeof(controlData));

  // Criação dos software timers
  xJoystickReadTimerHandle = xTimerCreate("JOYSTICK_READ_TIMER", pdMS_TO_TICKS(10), pdTRUE, NULL, vJoystickReadTimerCallback);
  if(xJoystickReadTimerHandle == NULL){
    Serial.println("Erro na criação do timer de leitura do joystick!");
    while(1);
  }

  // Criação das tasks
  xTaskCreatePinnedToCore(vDisplayWriteTask, "DISPLAY_WRITE_TASK", configMINIMAL_STACK_SIZE + (1024 * 10), NULL, 1, &xDisplayWriteTaskHandle, PRO_CPU_NUM);
  if(xDisplayWriteTaskHandle == NULL){
    Serial.println("Erro na criação da task de escrita no display!");
    while(1);
  }
  xTaskCreatePinnedToCore(vSDCardSaveTask, "SDCARD_SAVE_TASK", configMINIMAL_STACK_SIZE + (1024 * 10), NULL, 1, &xSDCardSaveTaskHandle, APP_CPU_NUM);
  if(xSDCardSaveTaskHandle == NULL){
    Serial.println("Erro na criação da task de salvamento no cartão SD!");
    while(1);
  }
  xTaskCreatePinnedToCore(vJoystickReadTask, "JOYSTICK_READ_TASK", configMINIMAL_STACK_SIZE + (1024 * 10), NULL, 1, &xJoystickReadTaskHandle, APP_CPU_NUM);
  if(xJoystickReadTaskHandle == NULL){
    Serial.println("Erro na criação da task de leitura do joystick!");
    while(1);
  }
  xTaskCreatePinnedToCore(vControlTask, "CONTROL_TASK", configMINIMAL_STACK_SIZE + (1024 * 10), NULL, 1, &xControlTaskHandle, PRO_CPU_NUM);
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
    Display.writeMessage(flightStatesTexts[(int)receivedFlightState],1);
    
    // Aguarda 100 ms antes da próxima exibição
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

// Task de salvamento no cartão SD
void vSDCardSaveTask(void *pvParameters){
  // Variável para receber o estado de voo da fila
  flightState receivedFlightState;

  while(1){
    // Espera por um estado de voo na fila
    xQueueReceive(xFlightStateToSDCard, &receivedFlightState, 0);
    
    // Formata a linha de dados para salvar
    String dataLine = String(millis()) + "," + flightStatesTexts[receivedFlightState];
    
    // Salva a linha no cartão SD
    SdLogger.writeLine(dataLine);

    // Aguarda um tempo antes da próxima iteração
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Task de leitura do joystick
void vJoystickReadTask(void *pvParameters){
  // Inicialização do timer de leitura do joystick
  xTimerStart(xJoystickReadTimerHandle, 0);

  // Variável para receber dado de controle do joystick
  controlData controlData;
  flightState currentFlightState;

  while(1){
    // Espera pela notificação do timer
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Leitura dos valores do joystick
    controlData.xValue = analogRead(JOYSTICK_ADC_X_PIN);
    controlData.yValue = analogRead(JOYSTICK_ADC_y_PIN);

    // Determinação do estado de voo baseado nos valores lidos
    if(controlData.yValue > 3200){
      currentFlightState  = PITCH_UP;
    } 
    else if(controlData.yValue < 2400){
      currentFlightState  = PITCH_DOWN;
    } 
    else if(controlData.xValue > 3200){
      currentFlightState  = YAW_LEFT;
    } 
    else if(controlData.xValue < 2400){
     currentFlightState  = YAW_RIGHT;
    }
    else{
      currentFlightState  = CRUISE;
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
        /*Regra de controle:
            - Flaps neutros
            - Profundores para cima baseado na leitura do joystick
            - Leme neutro
        */
        case PITCH_UP:
          FlapLeftServo.setAngle(90);                                                               
          FlapRightServo.setAngle(90);                                                              
          AileronLeftServo.setAngle(90);                                                            
          AileronRightServo.setAngle(90);                                                           
          ElevatorLeftServo.setAngle(90 + analogReadToAngleVariation(receivedControlData.yValue));  
          ElevatorRightServo.setAngle(90 + analogReadToAngleVariation(receivedControlData.yValue)); 
          RudderServo.setAngle(90);                                                                 
          break;
        
        // Controle para descida
        /*Regra de controle:
            - Flaps neutros
            - Profundores para baixo baseado na leitura do joystick
            - Leme neutro
        */
        case PITCH_DOWN:

          FlapLeftServo.setAngle(90);                                                                
          FlapRightServo.setAngle(90);                                                               
          AileronLeftServo.setAngle(90);                                                             
          AileronRightServo.setAngle(90);                                                            
          ElevatorLeftServo.setAngle(90 + analogReadToAngleVariation(receivedControlData.yValue));   
          ElevatorRightServo.setAngle(90 + analogReadToAngleVariation(receivedControlData.yValue));  
          RudderServo.setAngle(90);                                                                 
          break;
        
        // Controle para virar à direita
        /*Regra de controle:
            - Flaps neutros
            - Ailerons neutros
            - Profundores neutros
            - Leme para a direita baseado na leitura do joystick
        */
        case YAW_RIGHT:
          FlapLeftServo.setAngle(90);                                                         
          FlapRightServo.setAngle(90);                                                        
          AileronLeftServo.setAngle(90);                                                      
          AileronRightServo.setAngle(90);                                                     
          ElevatorLeftServo.setAngle(90);                                                     
          ElevatorRightServo.setAngle(90);                                                    
          RudderServo.setAngle(90 + analogReadToAngleVariation(receivedControlData.xValue));  
          break;
        
        // Controle para virar à esquerda
        /*Regra de controle:
            - Flaps neutros
            - Ailerons neutros
            - Profundores neutros
            - Leme para a esquerda baseado na leitura do joystick
        */
        case YAW_LEFT:
          FlapLeftServo.setAngle(90);                                                        
          FlapRightServo.setAngle(90);                                                       
          AileronLeftServo.setAngle(90);                                                     
          AileronRightServo.setAngle(90);                                                    
          ElevatorLeftServo.setAngle(90);                                                    
          ElevatorRightServo.setAngle(90);                                                   
          RudderServo.setAngle(90 + analogReadToAngleVariation(receivedControlData.xValue)); 
          break;
        
        default:
          break;
      }
    }
  }
}