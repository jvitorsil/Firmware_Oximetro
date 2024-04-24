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
#include <NimBLEDevice.h>


/* Pin numbers -------------------------------------------------------------------------------------------------------*/
#define AC_OX_PIN GPIO_NUM_0
#define DC_OX_PIN GPIO_NUM_1

#define RED_LED_PIN GPIO_NUM_3
#define IFR_LED_PIN GPIO_NUM_4

#define FBCK_LED_AZUL_PIN GPIO_NUM_10
#define FBCK_LED_VERD_PIN GPIO_NUM_9

/* Instances -------------------------------------------------------------------------------------------------------*/

NimBLEServer* pServer;
NimBLECharacteristic* pCharacteristic;
NimBLECharacteristic* pNewCharacteristic;
NimBLECharacteristic* pWriteCharacteristic;
NimBLECharacteristic* pCurveCharacteristic;

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

// Interrução
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
bool flagLedState = true;
bool flagReadData = false;
bool flagStable = false;
bool flagBPM = true;
// Variaveis para verificar estabilidade dos dados
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
void IRAM_ATTR LED_Control();




/* Main Application --------------------------------------------------------------------------------------------------*/
void setup() {

  setCpuFrequencyMhz(80);

  Serial.begin(115200);
  
  pinMode(IFR_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  pinMode(FBCK_LED_AZUL_PIN, OUTPUT);
  pinMode(FBCK_LED_VERD_PIN, OUTPUT);

  analogReadResolution(12);

  NimBLEDevice::init("OxiMed <3"); // Nome do Bluetooth
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); // Define o tipo de pareamento
  NimBLEDevice::setSecurityAuth(true, true, true); // Habilita o pareamento com autenticação e solicitação de PIN

  pServer = NimBLEDevice::createServer();
  NimBLEService* pService = pServer->createService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
  pWriteCharacteristic = pService->createCharacteristic(
                   "f14a557e-a13d-4b00-9353-913027a5cd89",
                   NIMBLE_PROPERTY::READ |
                   NIMBLE_PROPERTY::WRITE |
                   NIMBLE_PROPERTY::NOTIFY
                 );

  pCharacteristic = pService->createCharacteristic(
                   "beb5483e-36e1-4688-b7f5-ea07361b26a8",
                   NIMBLE_PROPERTY::READ |
                   NIMBLE_PROPERTY::WRITE |
                   NIMBLE_PROPERTY::NOTIFY
                 );

  pNewCharacteristic = pService->createCharacteristic(
                   "26e2b12b-85f0-4f3f-9fdd-91d114270e6e",
                   NIMBLE_PROPERTY::READ |
                   NIMBLE_PROPERTY::WRITE |
                   NIMBLE_PROPERTY::NOTIFY
                 );

  pCurveCharacteristic = pService->createCharacteristic(
                   "26e2b12b-85f0-4f3f-9fdd-91d114270ea5",
                   NIMBLE_PROPERTY::READ |
                   NIMBLE_PROPERTY::WRITE |
                   NIMBLE_PROPERTY::NOTIFY
                 );
  
  pService->start();
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
  pAdvertising->start();


  setTimer = timerBegin(0, INT_PRE_SCALER, true);
  timerAttachInterrupt(setTimer, &LED_Control, true);
  timerAlarmWrite(setTimer, INT_TIM_PERI, true);
  timerAlarmEnable(setTimer);

  xTaskCreate(processDataTask, "LEDs Control Task", 4096, NULL, 1, NULL);

}

void loop() {
}

void processDataTask(void *pvParameters) {
    while(1) {
        if(flagReadData){

            timerAlarmDisable(setTimer);    

            for(uint16_t count = 525; count < SIGNAL_SIZE - 525; count++){
              pCurveCharacteristic->setValue(signalForAnalyzeAcIFR[count]);
              pCurveCharacteristic->notify(); 
              // Serial.print("Curve: ");Serial.println(signalForAnalyzeAcIFR[count]);
              vTaskDelay(80/portTICK_PERIOD_MS);
            }
            
            find_valleys_and_peaks(flagBPM, signalForAnalyzeAcIFR, signalForAnalyzeDcIFR, valleysIFR, peaksIFR);
            find_valleys_and_peaks(!flagBPM, signalForAnalyzeAcRED, signalForAnalyzeDcRED, valleysRED, peaksRED);

            count = 0;
            flagReadData = false;
            timerAlarmEnable(setTimer);
        }
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


void IRAM_ATTR LED_Control(){

  if(flagLedState){ // Se a flag LED for verdadeira entra na condição e lê os dados para o IFR

    //Verifica se os dados estabilizaram
    verifyStabilizeData[countWinSize] = analogRead(AC_OX_PIN);
    countWinSize = (countWinSize + 1) % WINDOW_STABILIZE;

    flagStable = checkStability( verifyStabilizeData, WINDOW_STABILIZE);

    Serial.print(">Count:");
    Serial.println(count);

    Serial.print("> Signal AC IFR Raw:");
    Serial.println(verifyStabilizeData[countWinSize]);

    // Se os dados estabilizaram e preencheram todo o array de dados entra nessa condição para envio de dados 
    if(count >= SIGNAL_SIZE and flagStable)
    {
      flagReadData = true;
      count = 0;
      digitalWrite(FBCK_LED_AZUL_PIN, LOW);
      digitalWrite(FBCK_LED_VERD_PIN, HIGH);
    }

    // Se os dados estabilizaram e o array de dados ainda n está cheio entra na condição para preencher ele
    else if (count <= SIGNAL_SIZE and flagStable) 
    {
      MovingAverageFilter(&sumValuesAcIFR, arrayAcOxIFR, &windowAcIFR, &signalForAnalyzeAcIFR[count], WINDOW_AVERAGE, AC_OX_PIN);
      MovingAverageFilter(&sumValuesDcIFR, arrayDcOxIFR, &windowDcIFR, &signalForAnalyzeDcIFR[count], WINDOW_AVERAGE, DC_OX_PIN);

      Serial.print("> Signal AC IFR Filt:");
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
      digitalWrite(FBCK_LED_AZUL_PIN, LOW);
      digitalWrite(FBCK_LED_VERD_PIN, LOW);
    }
}
  else{

    if (count <= SIGNAL_SIZE and flagStable) 
    {
      MovingAverageFilter(&sumValuesAcRED, arrayAcOxRED, &windowAcRED, &signalForAnalyzeAcRED[count], WINDOW_AVERAGE, AC_OX_PIN);
      MovingAverageFilter(&sumValuesDcRED, arrayDcOxRED, &windowDcRED, &signalForAnalyzeDcRED[count], WINDOW_AVERAGE, DC_OX_PIN);

      Serial.print("> Signal AC RED Filt:");
      Serial.println(signalForAnalyzeAcRED[count]);

      count++;
      flagReadData = false;
    } 
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

    Serial.print("> Desvio Padrão: ");
    Serial.println(stdDev);

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

                Serial.print(" || Valley: ");
                Serial.print(signalAC[last_valley_index]);
                Serial.print(" || Peak: ");
                Serial.print(signalAC[last_peak_index]);
                Serial.print("  || last_valley_index: ");
                Serial.print(last_valley_index);
                Serial.print(" || last_peak_index: ");
                Serial.println(last_peak_index);
                vTaskDelay(1/portTICK_PERIOD_MS);

            }
        }
    }

    if(calBPM){
      uint16_t bpm = 5100/(0.68*((last_peak_index - last_valley_index)*1.6));
      uint16_t SPO2 = random(95, 101);

      Serial.print("  bpm: ");
      Serial.println(bpm);

      pCharacteristic->setValue(bpm);
      pCharacteristic->notify();
    
      pNewCharacteristic->setValue(SPO2);
      pNewCharacteristic->notify();

      vTaskDelay(1/portTICK_PERIOD_MS);
    }
}


void CalculateTheBPM(){
}