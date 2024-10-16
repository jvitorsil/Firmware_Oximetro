/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>

#include "led_state.h"
#include "esp32-hal-ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp32-hal-log.h"

/* Private Defines -----------------------------------------------------------*/

/* Private Variables ---------------------------------------------------------*/
static const char *TAG = "LEDS";
static int color = BLUE;
static int mode = BLINKING;

/* Exported Functions --------------------------------------------------------*/
void Leds_task(void* pvParameter)
{
    ESP_LOGI(TAG, "Starting leds task");

    pinMode(IFR_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);

    pinMode(FBCK_LED_AZUL_PIN, OUTPUT);
    pinMode(FBCK_LED_VERD_PIN, OUTPUT);

    while (1)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);

        switch (color)
        {
        case BLUE:
            digitalWrite(FBCK_LED_AZUL_PIN, HIGH);
            digitalWrite(FBCK_LED_VERD_PIN, LOW);
            break;

        case GREEN:
            digitalWrite(FBCK_LED_AZUL_PIN, LOW);
            digitalWrite(FBCK_LED_VERD_PIN, HIGH);
            break;

        default:
            digitalWrite(FBCK_LED_AZUL_PIN, LOW);
            digitalWrite(FBCK_LED_VERD_PIN, LOW);
            break;
        }

        if (mode == BLINKING)
        {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            
            digitalWrite(FBCK_LED_AZUL_PIN, LOW);
            digitalWrite(FBCK_LED_VERD_PIN, LOW);
        }
    }

    vTaskDelete(NULL);
}

void Leds_setColor(int _color)
{
    color = _color;
}

void Leds_setMode(int _mode)
{
    mode = _mode;
}

void Leds_setColorAndMode(int _color, int _mode)
{
    color = _color;
    mode = _mode;
}

int Leds_getColor()
{
    return color;
}

int Leds_getMode()
{
    return mode;
}