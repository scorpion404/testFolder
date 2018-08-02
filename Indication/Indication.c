#include "Indication.h"

#include "Board.h"

PIN_Handle pinLedHandle;
PIN_State pinLedState;

const PIN_Config LedGpioInitTable[] =
{
 IRB_LED_GREEN       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 IRB_LED_RED         | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

INDICATION_Type indication;

char isInit = 0;

void INDICAT_Init(void)
{
    //-------------------------------------------------     pin init    ---------------------------
    pinLedHandle = PIN_open(&pinLedState, LedGpioInitTable);

    PIN_setOutputValue(pinLedHandle, TD_LED_GREEN_ID, 0);

    INDICAT_Reset();
    isInit = 1;
}



void INDICAT_Reset (void)
{
    indication.mode             = INDICAT_OFF_MODE;
    indication.redBitField      = IND_BF_OFF;
    indication.greenBitField    = IND_BF_OFF;
    indication.stepNum          = 0;
}

/**
 *
 */
void INDICAT_Set (char indicatMode)
{
    if(isInit == 0)
        return 0;

    indication.mode = indicatMode;

    indication.stepNum = 0;

    switch(indicatMode)
    {
        case INDICAT_OFF_MODE:
            INDICAT_Reset();
        break;

        case INDICAT_PROG_MODE:
            indication.greenBitField    = IND_BF_OFF;
            indication.redBitField      = IND_BF_BLINK_FAST;
        break;

        case INDICAT_ALARM_MODE:
            indication.greenBitField    = IND_BF_OFF;
            indication.redBitField      = IND_BF_OFF;
        break;

        case INDICAT_LIGHT_ON_MODE:
            indication.greenBitField    = IND_BF_BLINK_FAST;
            indication.redBitField      = IND_BF_OFF;
        break;

        case INDICAT_LOOP_ALARM_MODE:
            indication.greenBitField    = IND_BF_OFF;
            indication.redBitField      = IND_BF_3SHOT;
        break;

        case INDICAT_LOOP_SHORT_MODE:
            indication.greenBitField    = IND_BF_OFF;
            indication.redBitField      = IND_BF_2SHOT;
        break;

        case INDICAT_LOOP_OPEN_MODE:
        case INDICAT_LOOP_BREAK_MODE:
            indication.greenBitField    = IND_BF_OFF;
            indication.redBitField      = IND_BF_1SHOT;
        break;

        case INDICAT_RED_TEST_MODE:
            indication.greenBitField    = IND_BF_OFF;
            indication.redBitField      = IND_BF_1SHOT;
        break;

        case INDICAT_GREEN_TEST_MODE:
            indication.greenBitField    = IND_BF_1SHOT;
            indication.redBitField      = IND_BF_OFF;
        break;


        default:
            break;
    }
}

/**
 *
 */
void INDICAT_Step (void)
{
    if(isInit == 0)
            return 0;

    if(indication.redBitField & (1 << indication.stepNum))
    {
        // enable red led
        PIN_setOutputValue(pinLedHandle, TD_LED_RED_ID, 1);
    }
    else
    {
        // disable red led
        PIN_setOutputValue(pinLedHandle, TD_LED_RED_ID, 0);
    }

    if(indication.greenBitField & (1 << indication.stepNum))
    {
        // enable green led
        PIN_setOutputValue(pinLedHandle, TD_LED_GREEN_ID, 1);
    }
    else
    {
        // disable green led
        PIN_setOutputValue(pinLedHandle, TD_LED_GREEN_ID, 0);
    }

    indication.stepNum += 1;

    if(indication.stepNum > 7)
    {
        if((indication.mode == INDICAT_GREEN_TEST_MODE) || (indication.mode == INDICAT_RED_TEST_MODE))
        {
            indication.mode = INDICAT_OFF_MODE;
        }
        INDICAT_Set(indication.mode);
    }
}

/**
 *
 */
void INDICAT_RedLed (char ledState)
{
    if(isInit == 0)
            return 0;

    indication.stepNum = 0;

    if(ledState == LED_ON)
    {
        indication.redBitField      = IND_BF_1SHOT_LONG;
    }
    else if(ledState == LED_OFF)
    {
        indication.redBitField      = IND_BF_OFF;
    }
    else if(ledState == LED_TOGGLE)
    {
        if(indication.redBitField)
            indication.redBitField      = IND_BF_OFF;
        else
            indication.redBitField      = IND_BF_1SHOT_LONG;
    }
}

/**
 *
 */
void INDICAT_GreenLed (char ledState)
{
    if(isInit == 0)
            return 0;

    indication.stepNum = 0;

    if(ledState == LED_ON)
    {
        indication.greenBitField      = IND_BF_1SHOT_LONG;
    }
    else if(ledState == LED_OFF)
    {
        indication.greenBitField      = IND_BF_OFF;
    }
    else if(ledState == LED_TOGGLE)
    {
        if(indication.greenBitField)
            indication.greenBitField      = IND_BF_OFF;
        else
            indication.greenBitField      = IND_BF_1SHOT_LONG;
    }
}

/**
 *
 */
