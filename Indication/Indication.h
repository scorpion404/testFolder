#ifndef __INDICATION_H
#define __INDICATION_H


enum _Indication_modes
{
    INDICAT_OFF_MODE,
    INDICAT_PROG_MODE,
    INDICAT_ALARM_MODE,
    INDICAT_LIGHT_ON_MODE,
    INDICAT_LOOP_ALARM_MODE,          //3
    INDICAT_LOOP_SHORT_MODE,         //2
    INDICAT_LOOP_OPEN_MODE,          //1
    INDICAT_LOOP_BREAK_MODE,         //1
    INDICAT_RED_TEST_MODE,      // выполняется только один цикл этого режима
    INDICAT_GREEN_TEST_MODE,    // выполняется только один цикл этого режима
};

enum    _Indication_Led_States
{
    LED_OFF,
    LED_ON,
    LED_TOGGLE,
};

enum _Indication_bitfields      // if bit is 1 the led on in this beat, else led off
{
    IND_BF_OFF          = 0x00,
    IND_BF_ON           = 0xFF,
    IND_BF_BLINK_FAST   = 0xAA,
    IND_BF_BLINK_NORM   = 0x33,
    IND_BF_BLINK_SLOW   = 0xFF,
    IND_BF_1SHOT_LONG   = 0x07,
    IND_BF_1SHOT        = 0x01,
    IND_BF_2SHOT        = 0x01 + 0x04,
    IND_BF_3SHOT        = 0x01 + 0x04 + 0x10,
};

typedef struct
{
    char mode;
    char stepNum;               // bit number, step after every ~200 ms
    char redBitField;           // red led bitfield,
    char greenBitField;         // green led bitfield

    char modeTimeMulty;         // множитель для времени шага индикации. 10 - шаг индикации  в норме и равен
}INDICATION_Type;


void    INDICAT_Init        (void);
void    INDICAT_Reset       (void);
void    INDICAT_Set         (char indicatMode);
void    INDICAT_Step        (void);
void    INDICAT_RedLed      (char ledState);
void    INDICAT_GreenLed    (char ledState);
#endif
