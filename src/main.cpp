/**
**************************************************************************************************************
* @file main.cpp
* @author João Vitor Silva
* @version V0.1.0
* @date 18-Abr-2024
* @brief code for Oximeter project - Instrumentação Biomedica 2.
* 
* Fs = 8ms => tempo entre uma interrupção e outra interrupção que da 125 Hz
* Coletando 10 segundos de dados => 1250 dados
*************************************************************************************************************
*/

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include <Arduino.h>
#include "ble_config.h"
#include "led_state.h"

/* Pin numbers -------------------------------------------------------------------------------------------------------*/
#define AC_OX_PIN GPIO_NUM_0
#define DC_OX_PIN GPIO_NUM_1

/* Instances -------------------------------------------------------------------------------------------------------*/
BLEConfig ble;

/* Constants ---------------------------------------------------------------------------------------------------------*/

// Filtro de media
#define WINDOW_AVERAGE 10

// Estabilização de dados
#define WINDOW_STABILIZE 150
#define STABILITY_THRESHOLD 190 // Limite para o desvio padrão que define estabilidade
#define STABLE_READINGS_THRESHOLD 10 // Número de leituras consecutivas para considerar o sinal estável
#define UNSTABLE_READINGS_THRESHOLD 5 // Número de leituras consecutivas para desconsiderar o sinal

// Analise de dados
#define SIGNAL_SIZE 1250
#define WINDOW_SIZE_MOVE 50

// Configuração da interrupção
#define INT_SET_FREQ 250
#define INT_PRE_SCALER 80
#define INT_TIM_FREQ  80000000/INT_PRE_SCALER
#define INT_TIM_PERI  INT_TIM_FREQ/INT_SET_FREQ

// Definição de janelas para detecção de picos e vales
#define WINDOW_SIZE 15
#define MIN_DISTANCE_BETWEEN_VALLEYS 50
#define MIN_DISTANCE_BETWEEN_PEAKS 50
#define PEAK_WINDOW_SIZE 65

/* Private variables -------------------------------------------------------------------------------------------------*/
volatile bool flagLedState = true;
volatile bool flagReadData = false;
volatile bool flagStable = false;
volatile bool flagBPM = true;

// Variáveis para verificar estabilidade dos dados
uint16_t verifyStabilizeData[WINDOW_STABILIZE];
uint8_t countWinSize = 0;

uint8_t windowAcRED = 0;
uint8_t windowDcRED = 0;
uint8_t windowAcIFR = 0;
uint8_t windowDcIFR = 0;

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

uint16_t peaksIFR[SIGNAL_SIZE];
uint16_t valleysIFR[SIGNAL_SIZE];

uint16_t peaksRED[SIGNAL_SIZE];
uint16_t valleysRED[SIGNAL_SIZE];

float data[SIGNAL_SIZE];
volatile uint16_t filtered_data[SIGNAL_SIZE];

int stableReadingsCount = 0;
int unstableReadingsCount = 0;
float previousStdDev = 0.0;
float stdDev = 0.0;

uint16_t count = 0;

// Definição da Interrupção
hw_timer_t *setTimer = NULL;

/* Private functions -------------------------------------------------------------------------------------------------*/

void MovingAverageFilter(uint16_t *sumValues, uint16_t *arrayValues, uint8_t *window, uint16_t *meanValue, const uint8_t windowSize, const uint8_t PIN);
void find_valleys_and_peaks(bool calBPM, uint16_t* signalAC, uint16_t* signalDC, uint16_t* valleys, uint16_t* peaks);
bool checkStability(uint16_t* values, int currentIndex);
void processDataTask(void *pvParameters);
void readData(int8_t acPin, int8_t dcPin, uint16_t* acArray, uint16_t* dcArray, uint16_t* signalAc, uint16_t* signalDc, uint16_t* sumValuesAc, uint16_t* sumValuesDc, uint8_t* windowAc, uint8_t* windowDc, bool isRedLed);
float calculateRatio(uint16_t* acRed, uint16_t* dcRed, uint16_t* acIR, uint16_t* dcIR, uint16_t start, uint16_t end);
void IRAM_ATTR LED_Control();

/* Main Application --------------------------------------------------------------------------------------------------*/
void setup() {
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  analogReadResolution(12);
  ble.init();
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  setTimer = timerBegin(0, INT_PRE_SCALER, true);
  timerAttachInterrupt(setTimer, &LED_Control, true);
  timerAlarmWrite(setTimer, INT_TIM_PERI, true);
  timerAlarmEnable(setTimer);

  xTaskCreate(Leds_task, "Feedback Leds Task", 8 * 1024, NULL, 4, NULL);
  xTaskCreate(processDataTask, "LEDs Control Task", 8 * 1024, NULL, 1, NULL);
}

void loop() { }

void IRAM_ATTR LED_Control() {
  static bool isRedLed = false;

  if (isRedLed) {
    readData(AC_OX_PIN, DC_OX_PIN, arrayAcOxRED, arrayDcOxRED, signalForAnalyzeAcRED, signalForAnalyzeDcRED, &sumValuesAcRED, &sumValuesDcRED, &windowAcRED, &windowDcRED, isRedLed);
  } else {
    readData(AC_OX_PIN, DC_OX_PIN, arrayAcOxIFR, arrayDcOxIFR, signalForAnalyzeAcIFR, signalForAnalyzeDcIFR, &sumValuesAcIFR, &sumValuesDcIFR, &windowAcIFR, &windowDcIFR, isRedLed);
  }

  digitalWrite(IFR_LED_PIN, isRedLed);
  digitalWrite(RED_LED_PIN, !isRedLed);  
  isRedLed = !isRedLed;
}


void readData(int8_t acPin, int8_t dcPin, uint16_t* acArray, uint16_t* dcArray, uint16_t* signalAc, uint16_t* signalDc, uint16_t* sumValuesAc, uint16_t* sumValuesDc, uint8_t* windowAc, uint8_t* windowDc, bool isRedLed) {
  static uint16_t countRed = 0;
  static uint16_t countIFR = 0;
  uint16_t* count = isRedLed ? &countRed : &countIFR;

  verifyStabilizeData[countWinSize] = analogRead(acPin);
  countWinSize = (countWinSize + 1) % WINDOW_STABILIZE;

  flagStable = checkStability(verifyStabilizeData, WINDOW_STABILIZE);

  if (isRedLed) {
    Serial.print(">Count RED:"); Serial.println(*count);
    Serial.print("> Signal AC RED Raw:"); Serial.println(verifyStabilizeData[countWinSize]);
  } else {
    Serial.print(">Count IF:"); Serial.println(*count);
    Serial.print("> Signal AC IFR Raw:"); Serial.println(verifyStabilizeData[countWinSize]);
  }

  if (*count >= SIGNAL_SIZE && flagStable) {
    *count = 0;
    flagReadData = true;
    Leds_setColorAndMode(GREEN, CONTINUOUS);
  } else if (*count < SIGNAL_SIZE && flagStable) {
    MovingAverageFilter(sumValuesAc, acArray, windowAc, &signalAc[*count], WINDOW_AVERAGE, acPin);
    MovingAverageFilter(sumValuesDc, dcArray, windowDc, &signalDc[*count], WINDOW_AVERAGE, dcPin);

    if (isRedLed) {
      Serial.print("> Signal AC RED Filt:"); Serial.println(signalAc[*count]);
      Serial.print("> Signal DC RED Filt:"); Serial.println(signalDc[*count]);
    } else {
      Serial.print("> Signal AC IFR Filt:"); Serial.println(signalAc[*count]);
      Serial.print("> Signal DC IFR Filt:"); Serial.println(signalDc[*count]);
    }

    (*count)++;
    flagReadData = false;
    Leds_setColorAndMode(BLUE, CONTINUOUS);
  } else {
    *count = 0;
    flagReadData = false;
    Leds_setColor(NO_COLOR);
  }
}

void processDataTask(void *pvParameters) {
  while (1) {
    if (flagReadData) {
      timerAlarmDisable(setTimer);

      for (uint16_t count = 200; count < SIGNAL_SIZE - 200; count++) {
        // uint16_t value = map(*(&signalForAnalyzeAcRED[count]), 2250, 2360, 0, 255);
        Serial.print("> plot RED:"); Serial.println(signalForAnalyzeAcRED[count]);
        Serial.print("> plot IFR:"); Serial.println(signalForAnalyzeAcIFR[count]);

        // ble.getCurveCharacteristic()->setValue(value);
        // ble.getCurveCharacteristic()->notify();
        vTaskDelay(20 / portTICK_PERIOD_MS);
      }

      find_valleys_and_peaks(flagBPM, signalForAnalyzeAcIFR, signalForAnalyzeDcIFR, valleysIFR, peaksIFR);
      find_valleys_and_peaks(!flagBPM, signalForAnalyzeAcRED, signalForAnalyzeDcRED, valleysRED, peaksRED);

      // Calcular SpO2 usando a janela de dados de 525 até SIGNAL_SIZE - 525
      uint16_t start = 200;
      uint16_t end = SIGNAL_SIZE - 200;
      float ratio = calculateRatio(signalForAnalyzeAcRED, signalForAnalyzeDcRED, signalForAnalyzeAcIFR, signalForAnalyzeDcIFR, start, end);
      uint16_t spo2 = 2497.06 * ratio - 2397.84;
      Serial.print("ratio: "); Serial.println(ratio, 8);
      Serial.print("  SpO2: "); Serial.println(spo2);

      count = 0;
      flagReadData = false;
      timerAlarmEnable(setTimer);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void MovingAverageFilter(uint16_t *sumValues, uint16_t *arrayValues, uint8_t *window, uint16_t *meanValue, const uint8_t windowSize, const uint8_t PIN) {
  uint16_t OxValueReading = analogRead(PIN);
  *sumValues -= arrayValues[*window];
  *sumValues += OxValueReading;
  arrayValues[*window] = OxValueReading;
  *window = (*window + 1) % windowSize;
  *meanValue = *sumValues / windowSize;
}

bool checkStability(uint16_t* values, int currentIndex) {
  float sum = 0.0;
  float mean = 0.0;

  // Calculate mean
  for (int i = currentIndex - WINDOW_STABILIZE + 1; i <= currentIndex; i++)
    sum += values[i];
  mean = sum / WINDOW_STABILIZE;

  // Calculate standard deviation
  sum = 0.0;
  for (int i = currentIndex - WINDOW_STABILIZE + 1; i <= currentIndex; i++)
    sum += pow(values[i] - mean, 2);
  stdDev = sqrt(sum / WINDOW_STABILIZE);

  // Check stability
  if (stdDev < STABILITY_THRESHOLD) {
    stableReadingsCount++;
    unstableReadingsCount = 0;
  } else {
    unstableReadingsCount++;
    stableReadingsCount = 0;
  }

  if (stableReadingsCount >= STABLE_READINGS_THRESHOLD)
    return true;
  else if (unstableReadingsCount >= UNSTABLE_READINGS_THRESHOLD)
    return false;

  return false;
}

void find_valleys_and_peaks(bool calBPM, uint16_t* signalAC, uint16_t* signalDC, uint16_t* valleys, uint16_t* peaks) {
  int last_valley_index = -MIN_DISTANCE_BETWEEN_VALLEYS;
  int last_peak_index = -MIN_DISTANCE_BETWEEN_PEAKS;

  for (uint16_t i = 200 + WINDOW_SIZE; i < SIGNAL_SIZE - WINDOW_SIZE - 200; i++) {
    bool is_valley = true;
    for (int j = i - WINDOW_SIZE; j <= i + WINDOW_SIZE; j++) {
      if (signalAC[j] < signalAC[i]) {
        is_valley = false;
        break;
      }
    }

    if (is_valley && i - last_valley_index >= MIN_DISTANCE_BETWEEN_VALLEYS) {
      *valleys++ = signalAC[i];
      last_valley_index = i;

      uint16_t peak = signalAC[i];
      uint16_t peakIndex = i;

      for (int j = i + 1; j < i + PEAK_WINDOW_SIZE && j < SIGNAL_SIZE; j++) {
        if (signalAC[j] > peak && j - last_peak_index >= MIN_DISTANCE_BETWEEN_PEAKS) {
          peak = signalAC[j];
          peakIndex = j;
        }
      }

      if (peakIndex != i) {
        *peaks++ = peak;
        last_peak_index = peakIndex;
      }
    }
  }

  if (calBPM) {
    uint16_t bpm = 5100 / (0.68 * ((last_peak_index - last_valley_index) * 1.4));
    uint16_t SPO2 = random(95, 101);

    Serial.print("  bpm: "); Serial.println(bpm);

    // ble.getCharacteristic()->setValue(bpm);
    // ble.getCharacteristic()->notify();

    // ble.getCharacteristic()->setValue(SPO2);
    // ble.getCharacteristic()->notify();

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void CalculateTheBPM() {
  // Função vazia, pode ser implementada conforme necessário
}

float calculateRatio(uint16_t* acRed, uint16_t* dcRed, uint16_t* acIR, uint16_t* dcIR, uint16_t start, uint16_t end) {
  float sumAcRed = 0, sumDcRed = 0, sumAcIR = 0, sumDcIR = 0;
  uint16_t count = end - start;

  for (uint16_t i = start; i < end; i++) {
    sumAcRed += acRed[i];
    sumDcRed += dcRed[i];
    sumAcIR += acIR[i];
    sumDcIR += dcIR[i];
  }

  float meanAcRed = sumAcRed / count;
  float meanDcRed = sumDcRed / count;
  float meanAcIR = sumAcIR / count;
  float meanDcIR = sumDcIR / count;

  Serial.print("meanAcRed: "); Serial.println(meanAcRed);
  Serial.print("meanDcRed: "); Serial.println(meanDcRed);
  Serial.print("meanAcIR: "); Serial.println(meanAcIR);
  Serial.print("meanDcIR: "); Serial.println(meanDcIR);

  if (meanDcRed == 0 || meanDcIR == 0) {
    Serial.println("Error: DC mean value is zero, cannot calculate ratio.");
    return 1.0; // Return a default value to avoid division by zero
  }

  float ratio = (meanAcRed / meanDcRed) / (meanAcIR / meanDcIR);
  return ratio;
}