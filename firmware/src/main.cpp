/*
 *      Created on: dezembro, 10, 2025
 *      @author: Vitor Alexandre (XandGVaz)
 *
 *  Description:
 *      Este arquivo implementa o firmware para um controlador de voo de aeromodelo (plane flight controller),
 * utilizando a plataforma PlatformIO com a biblioteca do Arduino, rodando em um microcontrolador ESP32.
 * O código integra controle de servomotores, exibição de estado de voo em um display 16x2 e guarda estados
 * de voo em um cartão SD.
 *
 *      O firmware é responsável por ler os comandos de um receptor de rádio (RC), processar os dados dos sensores
 * para estabilizar o voo e controlar os servos das superfícies de comando (aileron, profundor, leme).
 * Além disso, ele gerencia o registro de dados de voo em um cartão SD, para análise pós-voo e exibe informações 
 * críticas em um display. O sistema pode operar em diferentes modos de voo.
 *
 *      A utilização das bibliotecas do Arduino e a estrutura do PlatformIO tornam este código específico para
 * este ecossistema, sendo necessário adaptá-lo para outras plataformas de hardware ou software.
 */

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
#define JOYSTICK_BUTTON_PIN 27

/*===============================================================================*/
// Instanciação dos módulos

// Display
Display16x2 Display(DISPLAY_SDA_PIN, DISPLAY_SCL_PIN);

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

// Queues FreeRTOS
xQueueHandle xFlightStateToDisplay = NULL;
xQueueHandle xFlightStateToSDCard = NULL;
xQueueHandle xFlightStateToControl = NULL;
xQueueHandle xChangeYawOrRollToControl = NULL;

// Timers FreeRTOS
xTimerHandle xJoystickReadTimerHandle = NULL;

/*===============================================================================*/
// Prototypes de callbacks e ISRs

// Prototype de callback de timer de leitura do joystick
void vJoystickReadTimerCallback(TimerHandle_t xTimer);

// Prototype de callback para mudança de YAW para ROLL
void vIsrYawToRollChangeCallback();

/*===============================================================================*/
// Estados de voo

// Enum de possíveis estados de voo
typedef enum{
  CRUISE = 0U,
  PITCH_UP,
  PITCH_DOWN,
  YAW_RIGHT,
  YAW_LEFT,
  ROLL_RIGHT,
  ROLL_LEFT
}flightState;

// Enum para estado de movimentação do joystick em X
typedef enum{
  YAW_MOVIEMENT = 0U,
  ROLL_MOVIEMENT
}xMoviementState;

// Dado de controle com estado e angulatura do controle
typedef struct{
  flightState state;
  xMoviementState xMoviement;
  uint16_t xValue;
  uint16_t yValue;
}controlData;

// Macro para converter valor analógico de leitura do joystick em ângulo de variação
#define analogReadToAngleVariation(value) (value > 2048 ? map(value, 2049, 4095, 0, 45) : map(value, 0, 2047, -45, 0))

// Textos para estados de voo
String flightStatesTexts[] = {
  "CRUZEIRO",
  "PITCH UP",
  "PITCH DOWN",
  "YAW RIGHT",
  "YAW LEFT",
  "ROLL RIGHT",
  "ROLL LEFT"
};

/*===============================================================================*/

void setup() {
  // Inicialização da serial para debug
  Serial.begin(115200);
  delay(1000);
  Serial.println("Iniciando o controlador de voo...");

  // Configuração dos pinos do joystick
  pinMode(JOYSTICK_ADC_X_PIN, INPUT);
  pinMode(JOYSTICK_ADC_y_PIN, INPUT);
  pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);

  // Configuração da interrupção externa do botão do joystick
  attachInterrupt(digitalPinToInterrupt(JOYSTICK_BUTTON_PIN), vIsrYawToRollChangeCallback, FALLING);

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
  xFlightStateToDisplay = xQueueCreate(5, sizeof(flightState));
  xFlightStateToSDCard = xQueueCreate(5, sizeof(flightState));
  xFlightStateToControl = xQueueCreate(5, sizeof(controlData));
  xChangeYawOrRollToControl = xQueueCreate(1, sizeof(xMoviementState));

  // Criação dos software timers
  xJoystickReadTimerHandle = xTimerCreate("JOYSTICK_READ_TIMER", pdMS_TO_TICKS(50), pdTRUE, NULL, vJoystickReadTimerCallback);
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
  xTaskCreatePinnedToCore(vJoystickReadTask, "JOYSTICK_READ_TASK", configMINIMAL_STACK_SIZE + (1024 * 20), NULL, 2, &xJoystickReadTaskHandle, APP_CPU_NUM);
  if(xJoystickReadTaskHandle == NULL){
    Serial.println("Erro na criação da task de leitura do joystick!");
    while(1);
  }
  xTaskCreatePinnedToCore(vControlTask, "CONTROL_TASK", configMINIMAL_STACK_SIZE + (1024 * 20), NULL, 2, &xControlTaskHandle, PRO_CPU_NUM);
  if(xControlTaskHandle == NULL){
    Serial.println("Erro na criação da task de controle dos servos!");
    while(1);
  }

}

void loop() {
  vTaskDelete(NULL);
}

/*===============================================================================*/
// Definição da callbacks e ISRs

// Callback do timer de leitura do joystick
void vJoystickReadTimerCallback(TimerHandle_t xTimer){
  // Notificação para a task de controle dos servos
  vTaskNotifyGiveFromISR(xJoystickReadTaskHandle, NULL);
}

// Callback para mudança de movimentação de YAW para ROLL
void vIsrYawToRollChangeCallback(){
  // Variável estática para alternar entre YAW e ROLL
  static xMoviementState state = YAW_MOVIEMENT;

  // Alterna o estado de movimentação
  state = (state == YAW_MOVIEMENT) ? ROLL_MOVIEMENT : YAW_MOVIEMENT;

  // Atualiza o estado atual
  xQueueSendFromISR(xChangeYawOrRollToControl, &state, NULL);
}

/*===============================================================================*/
// Definição das tasks FreeRTOS

// Task de escrita no display
void vDisplayWriteTask(void *pvParameters){
  // Variável para receber o estado de voo da fila
  flightState receivedFlightState;

  // Estado de voo antigo para evitar escritas repetidas
  flightState oldFlightState = CRUISE;
  
  while(1){
    // Espera por um estado de voo na fila
    xQueueReceive(xFlightStateToDisplay, &receivedFlightState, 0);
    
    // Escreve no display apenas se o estado de voo mudou
    if(receivedFlightState != oldFlightState){  
      Display.clear();
      Display.writeMessage("Estado Voo:", 0);
      Display.writeMessage(flightStatesTexts[receivedFlightState],1);

      // Atualiza o estado de voo antigo
      oldFlightState = receivedFlightState;
    }

    // Aguarda 100 ms antes da próxima exibição
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

// Task de salvamento no cartão SD
void vSDCardSaveTask(void *pvParameters){
  // Variável para receber o estado de voo da fila
  flightState receivedFlightState;
  
  // Estado de voo antigo para evitar escritas repetidas
  flightState oldFlightState = CRUISE;

  // Variável de contagem de linhas salvas
  uint8_t lineCount = 0;

  while(1){
    // Espera por um estado de voo na fila
    xQueueReceive(xFlightStateToSDCard, &receivedFlightState, 0);
    
    // Formata a linha de dados para salvar
    String dataLine = String(millis()) + "," + flightStatesTexts[receivedFlightState];
    
    if(receivedFlightState != oldFlightState){ 
      // Salva a linha no cartão SD
      SdLogger.writeLine(dataLine);
      lineCount++;

      // Flush dos dados após 10 linhas escritas para garantir salvamento
      if(lineCount >= 10){
        lineCount = 0;
        SdLogger.FlushPackage();
      }

      // Atualiza o estado de voo antigo
      oldFlightState = receivedFlightState;
    }

    // Aguarda 100ms antes da próxima iteração
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Task de leitura do joystick
void vJoystickReadTask(void *pvParameters){
  // Inicialização do timer de leitura do joystick
  xTimerStart(xJoystickReadTimerHandle, 0);

  // Variável para receber dado de controle do joystick
  flightState currentFlightState = CRUISE;
  xMoviementState currentXMoviementState = YAW_MOVIEMENT;
  controlData controlData = {currentFlightState, currentXMoviementState, 2048, 2048};

  while(1){
    // Espera pela notificação do timer
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Verifica se houve notificação de mudança de movimentaçãoe entre YAW e ROLL
    if(xQueueReceive(xChangeYawOrRollToControl, &currentXMoviementState, 0) == pdTRUE){
      controlData.xMoviement = currentXMoviementState;
    }

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
      currentFlightState  = currentXMoviementState ? ROLL_LEFT : YAW_LEFT;
    } 
    else if(controlData.xValue < 2400){
     currentFlightState  = currentXMoviementState ? ROLL_RIGHT : YAW_RIGHT;
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

  // Pacote de controle antigo para evitar movimentações desnecessárias
  controlData oldControlData = {CRUISE, YAW_MOVIEMENT, 2048, 2048};

  while(1){
    // Espera por um estado de voo na fila
    xQueueReceive(xFlightStateToControl, &receivedControlData, portMAX_DELAY);

    // Evita movimentações desnecessárias dos servos
    if( 
        (receivedControlData.state == oldControlData.state) &&
        (receivedControlData.xMoviement == oldControlData.xMoviement) &&
        (receivedControlData.xValue == oldControlData.xValue) &&
        (receivedControlData.yValue == oldControlData.yValue)
    ){
      continue;
    }

    // Atualiza o pacote de controle antigo
    oldControlData = receivedControlData;
    
    // Seta todos os servos na posição neutra se o avião estiver em cruzeiro
    if(receivedControlData.state == CRUISE){
      AileronLeftServo.setAngle(90);
      AileronRightServo.setAngle(90);
      ElevatorLeftServo.setAngle(90);
      ElevatorRightServo.setAngle(90);
      RudderServo.setAngle(90);
      continue;
    }

    // Calcula variações de ângulo baseadas nas leituras do joystick
    int16_t angleVariationX = analogReadToAngleVariation(receivedControlData.xValue);
    int16_t angleVariationY = analogReadToAngleVariation(receivedControlData.yValue);

    // Ajusta os servos dos profundores com base nas variações calculadas 
    ElevatorLeftServo.setAngle(90 + angleVariationY);
    ElevatorRightServo.setAngle(90 + angleVariationY);

    // Ajusta os servos do leme com base nas variações calculadas
    if(receivedControlData.xMoviement == YAW_MOVIEMENT){
      RudderServo.setAngle(90 + angleVariationX);
    }
    
    // Ajusta os servos dos ailerons com base nas variações calculadas
    if(receivedControlData.xMoviement == ROLL_MOVIEMENT){
      AileronLeftServo.setAngle(90 - angleVariationX);
      AileronRightServo.setAngle(90 + angleVariationX);
    }
  }
}