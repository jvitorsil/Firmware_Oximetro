/**
**************************************************************************************************************
* @file main.cpp
* @author João Vitor Silva <joaovitor_s2015@ufu.br>
* @version V0.1.0
* @date 18-Abr-2024
* @brief code for Oximeter project - Instrumentação Biomedica 2.
* 
*
* Fs = 8ms => tempo entre uma interrupção e outra interrupção que da 125 Hz
* Coletando 10 segundos de dados => 1250 dados

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
#define STABILITY_THRESHOLD 155 // Limite para o desvio padrão que define estabilidade
#define STABLE_READINGS_THRESHOLD 10 // Número de leituras consecutivas para considerar o sinal estável
#define UNSTABLE_READINGS_THRESHOLD 5 // Número de leituras consecutivas para desconsiderar o sinal

// Analise de dados
#define SIGNAL_SIZE 1250
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

// Variaveis para verificar estabilidade dos dados
uint16_t verifyStabilizeData[WINDOW_STABILIZE];
uint8_t countWinSize = 0;


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
void processDataTask(void *pvParameters);
void IRAM_ATTR LED_Control();

void MovingAverageFilter(uint16_t *sumValues, uint16_t *arrayValues, uint8_t *window, uint16_t *meanValue, const uint8_t windowSize, const uint8_t PIN);
void detectPeaksAndValleys(volatile uint16_t arrayAcOx[]);

void generateHighPassFilter();
void applyFilter(volatile uint16_t arrayAcOx[]);

bool checkStability(uint16_t* values, int currentIndex);

void find_valleys_and_peaks(uint16_t* signal, uint16_t* valleys, uint16_t* peaks);
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

  xTaskCreate(processDataTask, "LEDs Control Task", 4096, NULL, 1, NULL);
}

void loop() { }

void processDataTask(void *pvParameters) {
    while(1) {
        if(flagReadData){
            Serial.print("AQUIIIIIIIIIIIIIIII:");
            timerAlarmDisable(setTimer);    
            for(uint16_t count = 210; count < SIGNAL_SIZE - 190; count++){
              
                Serial.print("> SignalACIFR:");
                Serial.println(signalForAnalyzeAcIFR[count]);

                digitalWrite(FBCK_LED_AZUL_PIN, LOW);
                digitalWrite(FBCK_LED_VERD_PIN, HIGH);

                vTaskDelay(1/portTICK_PERIOD_MS);
            }
            find_valleys_and_peaks(signalForAnalyzeAcIFR, valleys, peaks);

            // generateHighPassFilter();
            // applyFilter(signalForAnalyzeAcIFR);
            count = 0;
            flagReadData = false;
            timerAlarmEnable(setTimer);
        }
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

unsigned long lastInterruptTime = 0;

void IRAM_ATTR LED_Control(){

  if(flagLedState){ // Se a flag LED for verdadeira entra na condição e lê os dados para o IFR

    //Verifica se os dados estabilizaram
    verifyStabilizeData[countWinSize] = analogRead(AC_OX_PIN);
    countWinSize = (countWinSize + 1) % WINDOW_STABILIZE;

    flagStableRED = checkStability(verifyStabilizeData, WINDOW_STABILIZE);

    Serial.print(">Count:");
    Serial.println(count);

    // Se os dados estabilizaram e preencheram todo o array de dados entra nessa condição para envio de dados 
    if(count >= SIGNAL_SIZE and flagStableRED)
    {
      flagReadData = true;
      count = 0;
    }

    // Se os dados estabilizaram e o array de dados ainda n está cheio entra na condição para preencher ele
    else if (count <= SIGNAL_SIZE and flagStableRED) 
    {
      MovingAverageFilter(&sumValuesAcIFR, arrayAcOxIFR, &windowAc, &signalForAnalyzeAcIFR[count], WINDOW_AVERAGE, AC_OX_PIN);
      MovingAverageFilter(&sumValuesDcIFR, arrayDcOxIFR, &windowDc, &signalForAnalyzeDcIFR[count], WINDOW_AVERAGE, DC_OX_PIN);

      Serial.print("> SignalACIFFiltradoMV:");
      Serial.println(signalForAnalyzeAcIFR[count]);

      digitalWrite(FBCK_LED_AZUL_PIN, HIGH);
      digitalWrite(FBCK_LED_VERD_PIN, LOW);

      count++;
      flagReadData = false;
    } 


    else
    {
      flagReadData = false;
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

bool checkStability(uint16_t* values, int currentIndex) {
    
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
    
    stdDev = sqrt(sum / WINDOW_STABILIZE);

    // Check stability
    if (stdDev < STABILITY_THRESHOLD) {
        stableReadingsCount++;
        unstableReadingsCount = 0;
    } else {
        unstableReadingsCount++;
        stableReadingsCount = 0;
    }

    Serial.print("> StdDev: ");
    Serial.println(stdDev);

    if (stableReadingsCount >= STABLE_READINGS_THRESHOLD)
      return true;

    else if (unstableReadingsCount >= UNSTABLE_READINGS_THRESHOLD)
      return false;

  return false;
}


#define WINDOW_SIZE 10
#define MIN_DISTANCE_BETWEEN_VALLEYS 30
#define PEAK_WINDOW_SIZE 120



void find_valleys_and_peaks(uint16_t* signal, uint16_t* valleys, uint16_t* peaks) {

    int last_valley_index = -MIN_DISTANCE_BETWEEN_VALLEYS;
    int valleys_count = 0;

    for (uint16_t i = 200 + WINDOW_SIZE; i < SIGNAL_SIZE - WINDOW_SIZE - 200; i++) {
        bool is_valley = true;
        for (int j = i - WINDOW_SIZE; j <= i + WINDOW_SIZE; j++) {
            if (signal[j] < signal[i]) {
                is_valley = false;
                break;
            }
        }
        if (is_valley && i - last_valley_index >= MIN_DISTANCE_BETWEEN_VALLEYS) {
            *valleys++ = signal[i];
            last_valley_index = i;
            valleys_count++;

            uint16_t peak = signal[i];
            for (int j = i + 1; j < i + PEAK_WINDOW_SIZE && j < SIGNAL_SIZE; j++) {
                if (signal[j] > peak) {
                    peak = signal[j];
                }
            }
            
            *peaks++ = peak;

            Serial.print("> signal[i]: ");
            Serial.println(signal[i]);

            Serial.print("> peak: ");
            Serial.println(peak);
        }
    }
}