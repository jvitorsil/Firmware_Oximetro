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

/* Pin numbers -------------------------------------------------------------------------------------------------------*/
#define AC_OX_PIN GPIO_NUM_0
#define DC_OX_PIN GPIO_NUM_1

#define RED_LED_PIN GPIO_NUM_3
#define IFR_LED_PIN GPIO_NUM_4

#define FBCK_LED_AZUL_PIN GPIO_NUM_10
#define FBCK_LED_VERD_PIN GPIO_NUM_9

/* Instances -------------------------------------------------------------------------------------------------------*/

/* Constants ---------------------------------------------------------------------------------------------------------*/

// Filtro de media
#define WINDOW_AVERAGE 10

// Estabilização de dados
#define WINDOW_STABILIZE 150
#define STABILITY_THRESHOLD 200 // Limite para o desvio padrão que define estabilidade
#define STABLE_READINGS_THRESHOLD 10 // Número de leituras consecutivas para considerar o sinal estável
#define UNSTABLE_READINGS_THRESHOLD 5 // Número de leituras consecutivas para desconsiderar o sinal

// Analise de dados
#define SIGNAL_SIZE 1750
#define WINDOW_SIZE_MOVE 50

// Interrução
#define INT_SET_FREQ 250
#define INT_PRE_SCALER 80
#define INT_TIM_FREQ  80000000/INT_PRE_SCALER
#define INT_TIM_PERI  INT_TIM_FREQ/INT_SET_FREQ


/* Private variables -------------------------------------------------------------------------------------------------*/
bool flagLedState = true;
bool flagReadData = false;
bool flagStableRED = false;
bool flagStableIFR = false;

int values[WINDOW_STABILIZE];
int indice = 0;


uint8_t windowAc = 0;
uint8_t windowDc = 0;

uint16_t arrayAcOxRED[WINDOW_AVERAGE];
uint16_t arrayDcOxRED[WINDOW_AVERAGE];

uint16_t arrayAcOxIFR[WINDOW_AVERAGE];
uint16_t arrayDcOxIFR[WINDOW_AVERAGE];

uint16_t acOxValueReading = 0;
uint16_t dcOxValueReading = 0;

uint16_t sumValuesAcIFR = 0;
uint16_t sumValuesDcIFR = 0;
uint16_t sumValuesAcRED = 0;
uint16_t sumValuesDcRED = 0;

uint16_t signalForAnalyzeAcRED[SIGNAL_SIZE];
uint16_t signalForAnalyzeDcRED[SIGNAL_SIZE];

uint16_t signalForAnalyzeAcIFR[SIGNAL_SIZE];
uint16_t signalForAnalyzeDcIFR[SIGNAL_SIZE];


uint16_t peakCount = 0;
uint16_t valleyCount = 0;

uint16_t peaks[SIGNAL_SIZE];
uint16_t valleys[SIGNAL_SIZE];


float data[SIGNAL_SIZE];
volatile uint16_t filtered_data[SIGNAL_SIZE];

uint16_t count = 0;


// Definição da Interrupção
hw_timer_t *setTimer = NULL;


/* Private functions -------------------------------------------------------------------------------------------------*/
void LedControlTask(void *pvParameters);
void IRAM_ATTR LED_Control();

void MovingAverageFilter(uint16_t *sumValues, uint16_t *arrayValues, uint8_t *window, uint16_t *meanValue, const uint8_t windowSize, const uint8_t PIN);
void detectPeaksAndValleys(volatile uint16_t arrayAcOx[]);

void generateHighPassFilter();
void applyFilter(volatile uint16_t arrayAcOx[]);

bool checkStability(int* values, int currentIndex);


/* Main Application --------------------------------------------------------------------------------------------------*/
void setup() {

  setCpuFrequencyMhz(80);

  Serial.begin(115200);
  
  pinMode(IFR_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  pinMode(FBCK_LED_AZUL_PIN, OUTPUT);
  pinMode(FBCK_LED_VERD_PIN, OUTPUT);

  analogReadResolution(12);


  setTimer = timerBegin(0, INT_PRE_SCALER, true);
  timerAttachInterrupt(setTimer, &LED_Control, true);
  timerAlarmWrite(setTimer, INT_TIM_PERI, true);
  timerAlarmEnable(setTimer);

  xTaskCreate(LedControlTask, "LEDs Control Task", 4096, NULL, 1, NULL);

}

void loop() { }

void LedControlTask(void *pvParameters) {
    while(1) {
        if(flagReadData){
            Serial.print("AQUIIIIIIIIIIIIIIII:");
            timerAlarmDisable(setTimer);    
            for(uint16_t count = 0; count < SIGNAL_SIZE; count++){
              
                Serial.print("> SignalACIFR:");
                Serial.println(signalForAnalyzeAcIFR[count]);

                digitalWrite(FBCK_LED_AZUL_PIN, LOW);
                digitalWrite(FBCK_LED_VERD_PIN, HIGH);

                vTaskDelay(1/portTICK_PERIOD_MS);
            }

            // generateHighPassFilter();
            // applyFilter(signalForAnalyzeAcIFR);
            count = 0;
            flagReadData = false;
            timerAlarmEnable(setTimer);
        }
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void IRAM_ATTR LED_Control(){

  if(flagLedState){

    values[indice] = analogRead(AC_OX_PIN);
    indice = (indice + 1) % WINDOW_STABILIZE;

    flagStableRED = checkStability(values, WINDOW_STABILIZE);

    Serial.print(">Count:");
    Serial.println(count);

    Serial.print("> SignalACIFRBRUTO:");
    Serial.println(values[indice]);

    if(count >= SIGNAL_SIZE)
    {
      // flagReadData = true;
      count = 0;
    }
    else if (count <= SIGNAL_SIZE) 
    {
        MovingAverageFilter(&sumValuesAcIFR, arrayAcOxIFR, &windowAc, &signalForAnalyzeAcIFR[count], WINDOW_AVERAGE, AC_OX_PIN);
        MovingAverageFilter(&sumValuesDcIFR, arrayDcOxIFR, &windowDc, &signalForAnalyzeDcIFR[count], WINDOW_AVERAGE, DC_OX_PIN);

        Serial.print("> SignalACIFFiltradoMV:");
        Serial.println(signalForAnalyzeAcIFR[count]);

        digitalWrite(FBCK_LED_AZUL_PIN, HIGH);
        digitalWrite(FBCK_LED_VERD_PIN, LOW);

        count++;
        flagReadData = false;

      if (!flagStableRED) 
        flagStableRED = true;
    } 
    else
    {
      flagReadData = false;
      flagStableRED = false;
      count = 0;
    }
}
  else{
    // MovingAverageFilter(&sumValuesAcRED, arrayAcOxRED, &windowAc, &meanAcOxRED, 10);
    // MovingAverageFilter(&sumValuesDcRED, arrayDcOxRED, &windowDc, &meanDcOxRED, 10);

    // Serial.print(">SignalACRED:");
    // Serial.println(meanAcOxRED);

    // Serial.print(">SignalDCRED:");
    // Serial.println(meanDcOxRED);
  }

  flagLedState = !flagLedState;
  digitalWrite(IFR_LED_PIN, flagLedState);
  digitalWrite(RED_LED_PIN, !flagLedState);
}
  
void MovingAverageFilter(uint16_t *sumValues, uint16_t *arrayValues, uint8_t *window, uint16_t *meanValue, const uint8_t windowSize, const uint8_t PIN){
  
  uint16_t OxValueReading = analogRead(PIN);

  *sumValues -= arrayValues[*window];
  *sumValues += OxValueReading;
  arrayValues[*window] = OxValueReading;
  *window = (*window + 1) % windowSize;
  *meanValue = *sumValues / windowSize;
}



int stableReadingsCount = 0;
int unstableReadingsCount = 0;
float previousStdDev = 0.0;
float stdDev = 0.0;

bool checkStability(int* values, int currentIndex) {
    
    float sum = 0.0;
    float mean = 0.0;

    // Calculate mean
    for (int i = currentIndex - WINDOW_STABILIZE + 1; i <= currentIndex; i++) {
        sum += values[i];
    }
    mean = sum / WINDOW_STABILIZE;

    // Calculate standard deviation
    sum = 0.0;
    for (int i = currentIndex - WINDOW_STABILIZE + 1; i <= currentIndex; i++)
        sum += pow(values[i] - mean, 2);
    
    previousStdDev = stdDev;

    stdDev = sqrt(sum / WINDOW_STABILIZE);

    // Check stability
    if (stdDev < STABILITY_THRESHOLD and abs(stdDev - previousStdDev) < 0.1) {
        stableReadingsCount++;
        unstableReadingsCount = 0;
    } else {
        unstableReadingsCount++;
        stableReadingsCount = 0;
    }


    Serial.print("> abs: ");
    Serial.println(abs(stdDev - previousStdDev));

    Serial.print("> StdDev: ");
    Serial.println(stdDev);

    if (stableReadingsCount >= STABLE_READINGS_THRESHOLD)
      return true;

    else if (unstableReadingsCount >= UNSTABLE_READINGS_THRESHOLD)
      return false;
  
}


// void detectPeaksAndValleys(volatile uint16_t arrayAcOx[]) {
    
//   uint16_t PeakSum = 0;
//   uint16_t ValleySum = 0;

//   for (uint16_t i = 750; i < SIGNAL_SIZE - WINDOW_SIZE_MOVE; i++) {

//     uint16_t maxVal = arrayAcOx[i];
//     uint16_t minVal = arrayAcOx[i];

//     int maxPos = i;
//     int minPos = i;


//     for (int j = i; j < i + WINDOW_SIZE_MOVE; j++) {
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



// void applyFilter(volatile uint16_t arrayAcOx[]) {

//   uint16_t start_index = FILT_NUM_TAPS / 2;
//   uint16_t end_index = SIGNAL_SIZE - FILT_NUM_TAPS / 2;

//   for (uint16_t i = start_index; i < end_index; i++) {
//     volatile uint16_t filtered_value = 0;
//     for (uint16_t j = 0; j < FILT_NUM_TAPS; j++) {
//       filtered_value += arrayAcOx[i + j] * filter_taps[j];
//     }
//     filtered_data[i] = filtered_value;
//   }

//   for(uint16_t i = 0; i < SIGNAL_SIZE; i++){
//     Serial.print(">FilteredSignalACIFRTODO:");
//     Serial.println(filtered_data[i]);
//   }

//   for(uint16_t i = 200; i < 600; i++){
//     Serial.print(">FilteredSignalACIFRCORTADO:");
//     Serial.println(filtered_data[i]);
//   }

//   detectPeaksAndValleys(filtered_data);
// }