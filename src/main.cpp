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
// #include <Filters.h>
// #include <AH/Timing/MillisMicrosTimer.hpp>
// #include <Filters/Butterworth.hpp>

/* Constants ---------------------------------------------------------------------------------------------------------*/
#define winSize 10

// #define ORDER 2
// #define CUTOFF_FREQUENCY 10.0
// #define SAMPLING_FREQUENCY 300.0
// #define NORMALIZEDFREQ 2 * CUTOFF_FREQUENCY / SAMPLING_FREQUENCY

/* Pin numbers -------------------------------------------------------------------------------------------------------*/
#define AC_OX_PIN GPIO_NUM_0
#define DC_OX_PIN GPIO_NUM_1

#define RED_LED_PIN GPIO_NUM_3
#define IFR_LED_PIN GPIO_NUM_4

/* Instances -------------------------------------------------------------------------------------------------------*/
// auto filter = butter<6>(NORMALIZEDFREQ);

/* Private variables -------------------------------------------------------------------------------------------------*/
bool ledState = true;

uint8_t windowAcRED = 0;
uint8_t windowDcRED = 0;

uint8_t windowAcIFR = 0;
uint8_t windowDcIFR = 0;

uint32_t arrayAcOxRED[winSize];
uint32_t arrayDcOxRED[winSize];

uint32_t arrayAcOxIFR[winSize];
uint32_t arrayDcOxIFR[winSize];

uint16_t meanAcOxIFR = 0;
uint16_t meanDcOxIFR = 0;

uint16_t meanAcOxRED = 0;
uint16_t meanDcOxRED = 0;

uint16_t acOxValueReading = 0;
uint16_t dcOxValueReading = 0;

uint32_t allValuesAcIFR = 0;
uint32_t allValuesDcIFR = 0;
uint32_t allValuesAcRED = 0;
uint32_t allValuesDcRED = 0;

#define RANGE 1750

uint16_t signalForAnalyzeAcRED[RANGE];
uint16_t signalForAnalyzeDcRED[RANGE];

uint16_t signalForAnalyzeAcIFR[RANGE];
uint16_t signalForAnalyzeDcIFR[RANGE];

uint32_t count = 0;

hw_timer_t *setTimer = NULL;
uint16_t setFreq = 350;

uint8_t preScaler = 80;

uint32_t timerFrequency = 80000000 / preScaler;
uint32_t timerPeriod = timerFrequency / setFreq;

/* Private functions -------------------------------------------------------------------------------------------------*/
void LedControlTask(void *pvParameters);
void IRAM_ATTR LED_Control();

void MovingAverageFilter(uint32_t *allValues, uint32_t *arrayValues, uint8_t *window, uint16_t *meanValue, const uint8_t windowSize, const uint8_t PIN);

/* Main Application --------------------------------------------------------------------------------------------------*/
void setup() {

  setCpuFrequencyMhz(80);

  Serial.begin(115200);
  
  pinMode(IFR_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  analogReadResolution(12);

  setTimer = timerBegin(0, preScaler, true);
  timerAttachInterrupt(setTimer, &LED_Control, true);
  timerAlarmWrite(setTimer, timerPeriod, true);
  timerAlarmEnable(setTimer);

}


#define WINDOW_SIZE 100
#define MIN_VARIATION 10
#define STABLE_THRESHOLD 70



int values[WINDOW_SIZE];
int indice = 0;
bool isStable = false;


void loop() { }

unsigned long lastInterruptTime = 0;

void IRAM_ATTR LED_Control(){

  if(ledState){

    values[indice] = analogRead(AC_OX_PIN);
    indice = (indice + 1) % WINDOW_SIZE;

    int sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) 
      sum += values[i];
    
    float mean = sum / (float)WINDOW_SIZE;

    float sq_sum = 0;

    for (int i = 0; i < WINDOW_SIZE; i++) 
      sq_sum += pow(values[i] - mean, 2);
    
    float std_dev = sqrt(sq_sum / WINDOW_SIZE);

    Serial.print(">SignalStdIFR:");
    Serial.println(std_dev);

    Serial.print(">Count:");
    Serial.println(count);

    if(count >= RANGE and std_dev > MIN_VARIATION and std_dev < STABLE_THRESHOLD){
        // Serial.printf("String: %d \n", signalForAnalyzeAcIFR);
    }
    else if (count <= RANGE and std_dev > MIN_VARIATION and std_dev < STABLE_THRESHOLD) {
      
        MovingAverageFilter(&allValuesAcIFR, arrayAcOxIFR, &windowAcIFR, &meanAcOxIFR, 10, AC_OX_PIN);
        MovingAverageFilter(&allValuesDcIFR, arrayDcOxIFR, &windowDcIFR, &meanDcOxIFR, 10, DC_OX_PIN);

        signalForAnalyzeAcIFR[count] = meanAcOxIFR;
        signalForAnalyzeDcIFR[count] = meanDcOxIFR;

        Serial.print("> SignalACIFR:");
        Serial.println(signalForAnalyzeAcIFR[count]);

        count++;

      if (!isStable) 
        isStable = true;
      
    } else{
      isStable = false;
      count = 0;
    }
}


  else{
    // MovingAverageFilter(&allValuesAcRED, arrayAcOxRED, &windowAcRED, &meanAcOxRED, 10);
    // MovingAverageFilter(&allValuesDcRED, arrayDcOxRED, &windowDcRED, &meanDcOxRED, 10);

    // Serial.print(">SignalACRED:");
    // Serial.println(meanAcOxRED);

    // Serial.print(">SignalDCRED:");
    // Serial.println(meanDcOxRED);
  }

  ledState = !ledState;
  digitalWrite(IFR_LED_PIN, ledState);
  digitalWrite(RED_LED_PIN, !ledState);
}
  
void MovingAverageFilter(uint32_t *allValues, uint32_t *arrayValues, uint8_t *window, uint16_t *meanValue, const uint8_t windowSize, const uint8_t PIN){
  
    uint16_t OxValueReading = analogRead(PIN);

    *allValues -= arrayValues[*window];
    *allValues += OxValueReading;
    arrayValues[*window] = OxValueReading;
    *window = (*window + 1) % windowSize;
    *meanValue = *allValues / windowSize;

}
