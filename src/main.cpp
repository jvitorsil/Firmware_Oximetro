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
#include <WiFi.h>
#include <HTTPClient.h>

/* Pin numbers -------------------------------------------------------------------------------------------------------*/
#define AC_OX_PIN GPIO_NUM_0
#define DC_OX_PIN GPIO_NUM_1

#define RED_LED_PIN GPIO_NUM_3
#define IFR_LED_PIN GPIO_NUM_4


/* Instances -------------------------------------------------------------------------------------------------------*/
HTTPClient http;

/* Constants ---------------------------------------------------------------------------------------------------------*/
#define WINDOW_AVERAGE 10


#define WINDOW_STABILIZE 100
#define MIN_STD_VAR 1900
#define MAX_STD_VAR 2100


#define SIGNAL_SIZE 800
#define WINDOW_SIZE_MOVE 50


#define FILT_SAMPLE_RATE 170 // Hz
#define FILT_CUTOFF_FREQUENCY 2 // Hz
#define FILT_NUM_TAPS 51


#define INT_SET_FREQ 200
#define INT_PRE_SCALER 80
#define INT_TIM_FREQ  80000000/INT_PRE_SCALER
#define INT_TIM_PERI  INT_TIM_FREQ/INT_SET_FREQ


#define SSID "Solano"
#define PASSWORD "solano1234"
#define SERVER_URL "http://172.20.10.7:5000/"


/* Private variables -------------------------------------------------------------------------------------------------*/
bool flagLedState = true;
bool flagReadData = false;
bool flagStable = false;


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

float filter_taps[FILT_NUM_TAPS];

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

void sendMeasurement(uint16_t bpmValue);
void connectToWiFi();


/* Main Application --------------------------------------------------------------------------------------------------*/
void setup() {

  setCpuFrequencyMhz(80);

  Serial.begin(115200);
  
  pinMode(IFR_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  analogReadResolution(12);

  // WiFi.begin(SSID, PASSWORD);
  
  // if (WiFi.status() != WL_CONNECTED)
  //   delay(1000);
  

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
            timerAlarmDisable(setTimer);    
            for(uint16_t count = 0; count < SIGNAL_SIZE; count++){

              // if (WiFi.status() != WL_CONNECTED)
              //   connectToWiFi();
              //  else
              //   sendMeasurement(signalForAnalyzeAcIFR[count]);
              
                Serial.print("> SignalACIFR:");
                Serial.println(signalForAnalyzeAcIFR[count]);

                vTaskDelay(1/portTICK_PERIOD_MS);
            }

            // generateHighPassFilter();
            // applyFilter(signalForAnalyzeAcIFR);

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

    uint16_t sum = 0;

    for (int i = 0; i < WINDOW_STABILIZE; i++) 
      sum += values[i];
    
    float mean = sum / (float)WINDOW_STABILIZE;

    float sq_sum = 0;

    for (int i = 0; i < WINDOW_STABILIZE; i++) 
      sq_sum += pow(values[i] - mean, 2);
    
    float std_dev = sqrt(sq_sum / WINDOW_STABILIZE);

    Serial.print(">SignalStdIFR:");
    Serial.println(std_dev);

    Serial.print(">Count:");
    Serial.println(count);

    Serial.print("> SignalACIFRBRUTO:");
    Serial.println(values[indice]);

    if(count >= SIGNAL_SIZE and std_dev > MIN_STD_VAR and std_dev < MAX_STD_VAR){
      flagReadData = true;
      count = 0;
    }
    else if (count <= SIGNAL_SIZE and std_dev > MIN_STD_VAR and std_dev < MAX_STD_VAR) {
        flagReadData = false;

        MovingAverageFilter(&sumValuesAcIFR, arrayAcOxIFR, &windowAc, &signalForAnalyzeAcIFR[count], WINDOW_AVERAGE, AC_OX_PIN);
        MovingAverageFilter(&sumValuesDcIFR, arrayDcOxIFR, &windowDc, &signalForAnalyzeDcIFR[count], WINDOW_AVERAGE, DC_OX_PIN);

        Serial.print("> SignalACIFFiltradoMV:");
        Serial.println(signalForAnalyzeAcIFR[count]);

        count++;

      if (!flagStable) 
        flagStable = true;
      
    } else{
      flagReadData = false;
      flagStable = false;
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


void sendMeasurement(uint16_t bpmValue) {

  String requestBody = "{\"dado\":\"" + String(bpmValue) + "\"}";
  String url = String(SERVER_URL) + "/dados";

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  int httpResponseCode = http.POST(requestBody);


  if (httpResponseCode > 0) {
    Serial.print("Resposta do servidor: ");
    Serial.println(http.getString());
  } else {
    Serial.print("Erro na requisição HTTP: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}


void connectToWiFi() {

  Serial.println("Conectando ao WiFi...");
  WiFi.begin(SSID, PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 5) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConectado ao WiFi");
  } else {
    Serial.println("\nFalha ao conectar ao WiFi");
  }
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


// void generateHighPassFilter() {
//   int middle = FILT_NUM_TAPS / 2;
//   for (uint16_t i = 0; i < FILT_NUM_TAPS; i++) {
//     if (i == middle) {
//       filter_taps[i] = 1 - 2 * FILT_CUTOFF_FREQUENCY / FILT_SAMPLE_RATE;
//     } else {
//       filter_taps[i] = -sin(2 * PI * FILT_CUTOFF_FREQUENCY * (i - middle) / FILT_SAMPLE_RATE) / (PI * (i - middle));
//     }
//   }
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