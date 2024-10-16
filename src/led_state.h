#ifndef LEDS_H_
#define LEDS_H_

/* Includes ------------------------------------------------------------------*/

#define RED_LED_PIN GPIO_NUM_3
#define IFR_LED_PIN GPIO_NUM_4

#define FBCK_LED_AZUL_PIN GPIO_NUM_10
#define FBCK_LED_VERD_PIN GPIO_NUM_9

enum LEDS_COLORS
{
    GREEN,
    BLUE,
    NO_COLOR
};

enum LEDS_MODE
{
    CONTINUOUS,
    BLINKING
};

/* Exported Functions --------------------------------------------------------*/
void Leds_task(void* pvParameter);
void Leds_setColor(int _color);
void Leds_setMode(int _mode);
void Leds_setColorAndMode(int _color, int _mode);
int Leds_getColor();
int Leds_getMode();

#endif /* LEDS_H_ */