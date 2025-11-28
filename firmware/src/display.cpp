#define DEF_MOD_DISPLAY
#include "display.hpp"

/*===============================================================================*/
// Definição de métodos do display

// Método privado responssável por testar a conexão de um endereço (verificando se ele existe)
bool Display16x2::i2cAddrTest(byte addr){
  // Começa transmissão de barramento com o endereço passado por parâmetro
  Wire.beginTransmission(addr);
  
  // Se a transmissão ocorrer automaticamente, o endereço existe
  if(Wire.endTransmission() == 0);
    return true;
  
  // Caso contrário (demorou mais tempo para término da transmissão), o endereço não existe
  return false;
}

// Construtor da classe display16x2
Display16x2::Display16x2(uint8_t sdaPin, uint8_t sclPin){
  // Atribuição de pinos do display
  this->sdaPin = sdaPin;
  this->sclPin = sclPin;

  // Instanciação do objeto Display
  this->display = new LiquidCrystal_I2C(CI_ADDR1, 16/*colunas*/, 2/*linhas*/);
}

// Método público de configuração do display
bool  Display16x2::setup(){

  // Verifica instanciações de display
  if(display == NULL) return false;

  // Barramento de dados + controle
  Wire.begin(sdaPin, sclPin); // necessário para o funcionamento da biblioteca LiquidCrystal_I2C 

  // Caso o endereço do CI não exista no Display, então trocamos de endereço
  if(!i2cAddrTest(CI_ADDR1)){
    delete display;
    display = new LiquidCrystal_I2C(CI_ADDR2, 16, 2);
  }

  // Iniciação do driver do Display
  display->init();

  // Abre luz de fundo do Display
  display->backlight();

  // Retorna sucesso
  return true;
}

// Método público de atualização da mensagem do display
void Display16x2::updateMessage(String value){
  // Limpa tela do Display
  display->clear();

  // Seta a posição de impressão no Display no início da primeira linha
  display->setCursor(0,0);

  // Impressão de dado que cabe na primeira linha do Display
  if(value.length() <= 10){         
    display->print(value);
    return ;
  }

  // Impressão de dados que exigem o uso de duas linhas
  display->print(value.substring(0,11));
  display->setCursor(0,1);
  display->print(value.substring(11,27));
}

void Display16x2::writeMessage(String value, uint8_t line){
  // Seta a posição de impressão no Display na linha selecionada
  display->setCursor(0,line);

  // Impressão de dados na linha selecionada
  display->print(value);
}

void Display16x2::clear(){
  // Limpa caracteres presentes no display
  display->clear();
}