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
#define winSize 10

/* Pin numbers -------------------------------------------------------------------------------------------------------*/
#define AC_OX_PIN GPIO_NUM_0
#define DC_OX_PIN GPIO_NUM_1

#define RED_LED_PIN GPIO_NUM_3
#define IFR_LED_PIN GPIO_NUM_4

#define WINDOW_SIZE 100
#define MIN_VARIATION 10
#define STABLE_THRESHOLD 70

#define RANGE 800
#define WINDOW_SIZE_PEAK 50



#define NUM_SAMPLES 1750
#define SAMPLE_RATE 170 // Hz
#define CUTOFF_FREQUENCY 2 // Hz
#define NUM_TAPS 51


float data[NUM_SAMPLES];
volatile uint16_t filtered_data[NUM_SAMPLES];

float filter_taps[NUM_TAPS];

/* Instances -------------------------------------------------------------------------------------------------------*/



/* Private variables -------------------------------------------------------------------------------------------------*/
bool ledState = true;
bool processData = false;

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


//166 Hz - 6ms
//1,66 Hz - 600ms

volatile uint16_t signalForAnalyzeAcRED[RANGE];
volatile uint16_t signalForAnalyzeDcRED[RANGE];

volatile uint16_t signalForAnalyzeAcIFR[RANGE];
volatile uint16_t signalForAnalyzeDcIFR[RANGE];


uint16_t peakCount = 0;
uint16_t valleyCount = 0;

uint16_t peaks[RANGE];
uint16_t valleys[RANGE];


uint32_t count = 0;


hw_timer_t *setTimer = NULL;
uint16_t setFreq = 200;

uint8_t preScaler = 80;

uint32_t timerFrequency = 80000000 / preScaler;
uint32_t timerPeriod = timerFrequency / setFreq;

/* Private functions -------------------------------------------------------------------------------------------------*/
void LedControlTask(void *pvParameters);
void IRAM_ATTR LED_Control();

void MovingAverageFilter(uint32_t *allValues, uint32_t *arrayValues, uint8_t *window, uint16_t *meanValue, const uint8_t windowSize, const uint8_t PIN);
void detectPeaksAndValleys(volatile uint16_t arrayAcOx[]);

void generateHighPassFilter();
void applyFilter(volatile uint16_t arrayAcOx[]);

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


  xTaskCreate(LedControlTask, "LEDs Control Task", 4096, NULL, 1, NULL);

}

int values[WINDOW_SIZE];
int indice = 0;
bool isStable = false;


void loop() { }

void LedControlTask(void *pvParameters) {
    while(1) {
        if(processData){
            timerAlarmDisable(setTimer);    
            for(uint16_t count = 0; count < RANGE; count++){
              Serial.print("> SignalACIFR:");
              Serial.println(signalForAnalyzeAcIFR[count]);
              vTaskDelay(1 / portTICK_PERIOD_MS);
            }

            // generateHighPassFilter();
            // applyFilter(signalForAnalyzeAcIFR);

            processData = false;
        }
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

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

    // Serial.print(">SignalStdIFR:");
    // Serial.println(std_dev);

    Serial.print(">Count:");
    Serial.println(count);

    if(count >= RANGE and std_dev > MIN_VARIATION and std_dev < STABLE_THRESHOLD){
      processData = true;
    }
    else if (count <= RANGE and std_dev > MIN_VARIATION and std_dev < STABLE_THRESHOLD) {
        processData = false;

        MovingAverageFilter(&allValuesAcIFR, arrayAcOxIFR, &windowAcIFR, &meanAcOxIFR, winSize, AC_OX_PIN);
        MovingAverageFilter(&allValuesDcIFR, arrayDcOxIFR, &windowDcIFR, &meanDcOxIFR, winSize, DC_OX_PIN);

        signalForAnalyzeAcIFR[count] = meanAcOxIFR;
        signalForAnalyzeDcIFR[count] = meanDcOxIFR;

        Serial.print("> SignalACIFRBRUTO:");
        Serial.println(signalForAnalyzeAcIFR[count]);

        count++;

      if (!isStable) 
        isStable = true;
      
    } else{
      processData = false;
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

// void detectPeaksAndValleys(volatile uint16_t arrayAcOx[]) {
    
//   uint16_t PeakSum = 0;
//   uint16_t ValleySum = 0;

//   for (uint16_t i = 750; i < RANGE - WINDOW_SIZE_PEAK; i++) {

//     uint16_t maxVal = arrayAcOx[i];
//     uint16_t minVal = arrayAcOx[i];

//     int maxPos = i;
//     int minPos = i;


//     for (int j = i; j < i + WINDOW_SIZE_PEAK; j++) {
//       if (arrayAcOx[j] > maxVal) {
//         maxVal = arrayAcOx[j];
//         maxPos = j;
//       }
//       if (arrayAcOx[j] < minVal) {
//         minVal = arrayAcOx[j];
//         minPos = j;
//       }
//     }

//     if (maxPos == i) {
//       peaks[peakCount] = maxVal;
//       PeakSum += maxVal;
//       peakCount++;
//     }
//     if (minPos == i) {
//       valleys[valleyCount] = minVal;
//       ValleySum += minVal;
//       valleyCount++;
//     }
//   }
//   Serial.printf("Peaks: %d || Valley: %d || valleyCount: %d || peakCount: %d \n", PeakSum, ValleySum, valleyCount, peakCount);
// }


// void generateHighPassFilter() {
//   int middle = NUM_TAPS / 2;
//   for (uint16_t i = 0; i < NUM_TAPS; i++) {
//     if (i == middle) {
//       filter_taps[i] = 1 - 2 * CUTOFF_FREQUENCY / SAMPLE_RATE;
//     } else {
//       filter_taps[i] = -sin(2 * PI * CUTOFF_FREQUENCY * (i - middle) / SAMPLE_RATE) / (PI * (i - middle));
//     }
//   }
// }

// void applyFilter(volatile uint16_t arrayAcOx[]) {

//   uint16_t start_index = NUM_TAPS / 2;
//   uint16_t end_index = NUM_SAMPLES - NUM_TAPS / 2;

//   for (uint16_t i = start_index; i < end_index; i++) {
//     volatile uint16_t filtered_value = 0;
//     for (uint16_t j = 0; j < NUM_TAPS; j++) {
//       filtered_value += arrayAcOx[i + j] * filter_taps[j];
//     }
//     filtered_data[i] = filtered_value;
//   }

//   for(uint16_t i = 0; i < RANGE; i++){
//     Serial.print(">FilteredSignalACIFRTODO:");
//     Serial.println(filtered_data[i]);
//   }

//   for(uint16_t i = 200; i < 600; i++){
//     Serial.print(">FilteredSignalACIFRCORTADO:");
//     Serial.println(filtered_data[i]);
//   }

//   detectPeaksAndValleys(filtered_data);
// }