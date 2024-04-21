/**
**************************************************************************************************************
* @file main.cpp
* @author João Vitor Silva <joaovitor_s2015@ufu.br>
* @version V0.1.0
* @date 18-Abr-2024
* @brief code for Oximeter project - Instrumentação Biomedica 2.
*************************************************************************************************************
*/

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include <Arduino.h>

/* Constants ---------------------------------------------------------------------------------------------------------*/

/* Pin numbers -------------------------------------------------------------------------------------------------------*/
#define AC_OX_PIN GPIO_NUM_0
#define DC_OX_PIN GPIO_NUM_1

#define RED_LED_PIN GPIO_NUM_3
#define IFR_LED_PIN GPIO_NUM_4


/* Private variables -------------------------------------------------------------------------------------------------*/
bool ledState = true;

const uint16_t readings = 1000;

uint16_t acOxValueRED[readings];
uint16_t acOxValueIFR[readings];

uint16_t dcOxValueRED[readings];
uint16_t dcOxValueIFR[readings];

uint16_t acOxValueReadingRED = 0;
uint16_t acOxValueReadingIFR = 0;
uint16_t dcOxValueReadingIFR = 0;
uint16_t dcOxValueReadingRED = 0;
uint16_t acOxValueReading = 0;
uint32_t count = 0;



hw_timer_t *setTimer = NULL;
uint16_t setFreq = 50;

uint8_t preScaler = 80;

uint32_t timerFrequency = 80000000 / preScaler;
uint32_t timerPeriod = timerFrequency / setFreq;

/* Private functions -------------------------------------------------------------------------------------------------*/
void LedControlTask(void *pvParameters);
// void IRAM_ATTR LED_Control();

/* Main Application --------------------------------------------------------------------------------------------------*/
void setup() {

  setCpuFrequencyMhz(80);

  Serial.begin(115200);
  
  pinMode(IFR_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  analogReadResolution(12);

  // setTimer = timerBegin(0, preScaler, true);
  // timerAttachInterrupt(setTimer, &LED_Control, true);
  // timerAlarmWrite(setTimer, timerPeriod, true);
  // timerAlarmEnable(setTimer);

  for (int i = 0; i < readings; i++) {
    acOxValueRED[i] = 0;
  }

  xTaskCreate(LedControlTask, "LEDs Control Task", 4096, NULL, 1, NULL);

}

void loop() { }

void LedControlTask(void *pvParameters) {
  while(1) {
    ledState = !ledState;

    digitalWrite(IFR_LED_PIN, false);
    digitalWrite(RED_LED_PIN, true);

    vTaskDelay(5/portTICK_PERIOD_MS);

    if(ledState){
      acOxValueIFR[count] = analogRead(AC_OX_PIN);
      dcOxValueIFR[count] = analogRead(DC_OX_PIN);

      Serial.print(">SignalACIFR:");
      Serial.println(acOxValueIFR[count]);

      Serial.print(">SignalDCIFR:");
      Serial.println(dcOxValueIFR[count]);
    }

    else{
      acOxValueRED[count] = analogRead(AC_OX_PIN);
      dcOxValueRED[count] = analogRead(DC_OX_PIN);

      Serial.print(">SignalACRED:");
      Serial.println(acOxValueRED[count]);

      Serial.print(">SignalDCRED:");
      Serial.println(dcOxValueRED[count]);
    }

  }
}


// int ind = 0;              // Índice atual do array
// int total = 0;              // Soma total das leituras
// int average = 0;

// bool isStable(int valueNow, int Mean);

// unsigned long lastInterruptTime = 0;

// void IRAM_ATTR LED_Control(){
  
//   // Calcula o tempo decorrido desde a última interrupção
//   // unsigned long currentMillis = millis();
//   // unsigned long elapsedTime = currentMillis - lastInterruptTime;
  
//   // lastInterruptTime = currentMillis;

//   // // Verifica se a interrupção ocorreu no intervalo desejado
//   // Serial.print("Tempo decorrido entre interrupções: ");
//   // Serial.print(elapsedTime);
//   // Serial.println(" ms");

//   if(ledState){
//     acOxValueReading = analogRead(AC_OX_PIN);
//     total -= acOxValueIFR[ind];
//     acOxValueIFR[ind] = acOxValueReading;
//     total += acOxValueReading;
//     ind = (ind + 1) % readings;
//     average = total / readings;
//   }

//   if(!ledState){
//     acOxValueReading = analogRead(AC_OX_PIN);
//     total -= acOxValueRED[ind];
//     acOxValueRED[ind] = acOxValueReading;
//     total += acOxValueReading;
//     ind = (ind + 1) % readings;
//     average = total / readings;
//   }

//   // if (isStable(acOxValueReading, average) and acOxValueReading < 2500 and acOxValueReading > 1800) {

//     if(ledState){
//       acOxValueIFR[count] = analogRead(AC_OX_PIN);
//       dcOxValueIFR[count] = analogRead(DC_OX_PIN);

//       Serial.print(">SignalACIFR:");
//       Serial.println(acOxValueIFR[count]);

//       Serial.print(">SignalDCIFR:");
//       Serial.println(dcOxValueIFR[count]);
//     }
//     else{
//       acOxValueRED[count] = analogRead(AC_OX_PIN);
//       dcOxValueRED[count] = analogRead(DC_OX_PIN);

//       Serial.print(">SignalACRED:");
//       Serial.println(acOxValueRED[count]);

//       Serial.print(">SignalDCRED:");
//       Serial.println(dcOxValueRED[count]);
//     }
//   // }
  
//   Serial.println("Média: " + String(average) + " || Valor atual: " + String(acOxValueReading) + " || Diferença: " + String(isStable(acOxValueReading, average)));

//   // ledState = !ledState;
//   // digitalWrite(IFR_LED_PIN, ledState);
//   // digitalWrite(RED_LED_PIN, !ledState);
// }

// // Função para verificar se os dados estão estáveis
// bool isStable(int valueNow, int Mean) {
//   int threshold = 50;
//   return abs(valueNow - Mean) < threshold;
// }