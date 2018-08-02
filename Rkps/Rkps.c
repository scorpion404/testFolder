/*
 * Rkps.c
 *
 *  Created on: 26 апр. 2018 г.
 *      Author: Vladimir Bortnikov
 */

#include <string.h>

#include "Rkps.h"
#include "SensorControl_Kirill/source/scif.h"
#include "Indication/Indication.h"
#include "Crc/Crc.h"


#include <ti/drivers/rf/RF.h>
#include <ti/drivers/UART.h>



#include DeviceFamily_constructPath(driverlib/aux_adc.h)
#include DeviceFamily_constructPath(driverlib/osc.h)
#include DeviceFamily_constructPath(driverlib/flash.h)
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

#include "smartrf_settings/smartrf_settings.h"
#include "RFQueue.h"


#define BV(x) (1 << (x))

//SCIF_TASK_DATA_T *scData; // = (const) scifTaskData;

static unsigned short freqArr[][2] =    // массив частот радиоканалов
{
    {864, 025*65},          // 0
    {864, 075*65},          // 1
    {864, 125*65},          // 2
    {864, 175*65},          // 3
    {864, 225*65},          // 4
    {864, 275*65},          // 5
    {864, 325*65},          // 6
    {864, 375*65},          // 7
    {864, 425*65},          // 8
    {864, 475*65},          // 9
    {864, 525*65},          // 10
    {864, 575*65},          // 11
    {864, 625*65},          // 12
    {864, 675*65},          // 13
    {864, 725*65},          // 14
    {864, 775*65},          // 15
    {864, 825*65},          // 16
    {864, 875*65},          // 17
    {864, 925*65},          // 18
    {864, 975*65},          ///< programming channel            // index == 19, chanNumber == 20
        {868, 725*65},          // 20
        {868, 775*65},          // 21
        {868, 825*65},          // 22
        {868, 875*65},          // 23
        {868, 925*65},          // 24
        {868, 975*65},          // 25
        {869, 025*65},          // 26
        {869, 075*65},          // 27
        {869, 125*65},          // 28
        {869, 175*65},          // 29
    {868, 025*65},          // 30
    {868, 075*65},          // 31
    {868, 125*65},          // 32
    {868, 175*65}           // 33
};



static char rfRxDone = 0;
static char rfTxDone = 0;
static char CF[CONTROL_FRAME_OWN_QUANT];   ///< Control frame own array

functionPtr RKPS_OWN_Handler_Array[RKPS_HANDLER_QUANT]; //<---------------------- OWN handler function pointer array


//---------------------------------------   TIRTOS Pin struct and handler
PIN_Handle pinHandle;
PIN_State pinState;

//--------------------------------------    TIRTOS mailbox parameters
/* This buffer is not directly accessed by the application */
MailboxMsgObj mailboxBuffer[NUMMSGS];
Mailbox_Struct mbxStruct;
Mailbox_Handle mbxHandle;


//---------------------------------------   TIRTOS clock structures
Clock_Struct    clkLinkStruct;
Clock_Struct    clkProgStruct;
Clock_Struct    clkFreeStruct;
Clock_Struct    clkSyncStruct;
Clock_Struct    clkListenStruct;
Clock_Struct    clkKeyStruct;

Clock_Handle    clkLinkHandle;
Clock_Handle    clkProgHandle;
Clock_Handle    clkFreeHandle;
Clock_Handle    clkSyncHandle;
Clock_Handle    clkListenHandle;
Clock_Handle    clkKeyHandle;

//---------------------------------------   TIRTOS RF entity parameters
static RF_Object rfObject;
static RF_Handle rfHandle;
static RF_Params rfParams;
/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;

static uint8_t* packetDataPointer;
uint32_t curtime;


/* Packet RX Configuration */
#define NUM_DATA_ENTRIES       1  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * Max 30 payload bytes
                                   * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */

#pragma DATA_ALIGN (rxDataEntryBuffer, 4);
       static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                        PACK_LENGTH+NUM_APPENDED_BYTES,
                                                                NUM_APPENDED_BYTES)];
//----------------------------------------  RF Task parameters
#define TASKSTACKSIZE   512
Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

//----------------------------------    variables for delta time calculation
UInt32 startTime = 0;
UInt32 deltaTime = 0;

rfc_propRxOutput_t      rfOutStructure;

//----------------------------------    RKPS parameters
RKPS_Type               rkps;                    // rkps structure, in Flash
RML_Setting_Type        rmlSettings;             // in Flash
RML_State_Type          rmlState;                // rml states structure
RML_State_Flags_Type    rmlFlags;                // flags parameterf, in Flash

QUEUE_Type              rkpsQ;                   // rkps out message queue

//char LED_Indication = 0;

char rfPackIn[30];

//---------------------------------------------------------------   UART structure and params
char        input;
UART_Handle uart;
UART_Params uartParams;
char wasCallBack = 0;

unsigned long buttonsClkTime = 0;


//----------------------------------------------------------------    static function initilisation
static void             Rkps_FillCommon         (RKPS_Pack_Type* pMessage);
static void             Rkps_FillLink           (void);
static void             Rkps_FillProg0Ack       (void);
static void             Rkps_FillProgAckEnd    (void);
static void             Rkps_FillProg1Ack       (void);
static void             Rkps_FillMsg            (void);
static void             Rkps_ClockInit          (Clock_Params *clkParam, Clock_Handle * clkHandle, Clock_Struct* clkStruct);
static void             Rkps_StartClockEntity   (char ownHandlerNum, unsigned long tickQuant);
static void             Rkps_RF_RX_okIndicat    (RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
static void             Rkps_RF_TX_okIndicat    (RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
static unsigned long    Rkps_RetTimeInUs        (unsigned long tiNumber);
static void             Rkps_CalculMyOwnNumber  (void);
static unsigned short   Rkps_CalcDetlaOwn       (unsigned short curOwn, char nextOwnType);
static void             Rkps_CalculScenario     (void);
static void             Rkps_CalculShiftTime    (void);
static unsigned long    Rkps_CalculFreeOwnTime  (void);

static char             Rkps_FlashSavingHandle  (void);
static void             Rkps_RmlSettingsDefault (void);


static char             Rkps_SyncronizeFlags    (void);
static void             Rkps_SetFlagState       (char flagNum, char newState);
static char             Rkps_RetFlag            (char flagNum);

static char             Rkps_ReturnFlagState                (unsigned long flashValue);
static void             Rkps_ChangeFlagStateInFlashWord     (unsigned long *pFlashWord);


static char             Rkps_SaveRmlFlagsToFlash        (void);
static void             Rkps_ReadSettingsFromFlash      (void);
static void             Rkps_ReadFlagsFromFlash         (void);
static char             Rkps_SaveAllToFlash             (void);

//-------   RKPS CMD HANDLERS    -------------------------------------------  RKPS CMD HANDLERS
static void             Rkps_SetArmed_H     (char Off0_On1);
static void             Rkps_SetDisarmed_H  (char Off0_On1);
static void             Rkps_Indication_H   (char Off0_On1);
//static void             Rkps_HwVer_H        (char Off0_On1);
static void             Rkps_SwVer_H        (char Off0_On1);
static void             Rkps_Blocking_H     (char Off0_On1);
static void             Rkps_CmdHandle          (void);

//--------    OUT MESSAGE QUEUE     ----------------------------------------  OUT MESSAGE QUEUE
static void     RkpsQ_Free              (void);
static char     RkpsQ_Clean             (void);
static char     RkpsQ_Push              (char msgId, char param1, char param2);
static char     RkpsQ_IsEmpty           (void);
static char     RkpsQ_IsHaveMsg         (char msgId, char param1, char param2);
static char     RkpsQ_PullOut           (char* msgId, char* param1, char* param2);
static char     RkpsQ_Restore           (void);
static char     RkpsQ_HaveSomeMsg       (void);


static void     UART_SomeCallBack       (void);
static void     Rkps_MakePcPacket       (SCIF_TASK_DATA_T *pScifTaskData, char *pcPack, char *pOutSize);
static uint8_t  _crc8                   (uint8_t* addr, int size);
static uint8_t  _crc_ibutton_update     (uint8_t crc, uint8_t data);

void RFS_RML_SetArmed(char Off0_On1);
void RFS_RML_SetDisarmed(char Off0_On1);

static double log_x (double x);
static short TemperatureCalcul(unsigned long adcValue);


Void                    clkSwitchRkpsFxn        (UArg arg0);
Void                    clkKeybuttonFxn        (UArg arg0);
//--------------------------------------------------------------------  rkps handlers array
const CMD_EVENT_Handlers_type   cmdHandlers[] =
{
    {CMD_ARMED,         Rkps_SetArmed_H     },
    {CMD_DISARMED,      Rkps_SetDisarmed_H  },
    {CMD_INDON,         Rkps_Indication_H   },
    //{CMD_LSENSE,        Rkps_LoopSense_H    },
//    {CMD_HWV,           Rkps_HwVer_H        },
    {CMD_SWV,           Rkps_SwVer_H        },
    {CMD_BLOCK,         Rkps_Blocking_H     },
};


//--------------------------------------------------------------------  function definition section

void RKPS_Init(void)
{
    unsigned int modeButtonState = 0;
//    unsigned int i = 0;

    Clock_Params clkParams0;
    Clock_Params clkParams1;
    Clock_Params clkParams2;
    Clock_Params clkParams3;
    Clock_Params clkParams4;
    Clock_Params clkParams5;

    Mailbox_Params mbxParams;

    //---------------------------------------     mailbox init    ---------------------------
    /* Construct a Mailbox instance */
    Mailbox_Params_init(&mbxParams);
    mbxParams.buf = (Ptr)mailboxBuffer;
    mbxParams.bufSize = sizeof(mailboxBuffer);
    Mailbox_construct(&mbxStruct, sizeof(MsgObj), NUMMSGS, &mbxParams, NULL);
    mbxHandle = Mailbox_handle(&mbxStruct);

    //------------------------------------------    clock init  ------------------------------------
    Rkps_ClockInit(&clkParams0, &clkLinkHandle, &clkLinkStruct);
    Rkps_ClockInit(&clkParams1, &clkProgHandle, &clkProgStruct);
    Rkps_ClockInit(&clkParams2, &clkFreeHandle, &clkFreeStruct);
    Rkps_ClockInit(&clkParams3, &clkSyncHandle, &clkSyncStruct);
    Rkps_ClockInit(&clkParams4, &clkListenHandle, &clkListenStruct);

//    Rkps_ClockInit(&clkParams5, &clkKeyHandle, &clkKeyStruct);

    buttonsClkTime = Rkps_RetTimeInUs(23*8) / 10 *2;

    Clock_construct(&clkKeyStruct, clkKeybuttonFxn, 0, &clkParams5);

    clkKeyHandle = Clock_handle(&clkKeyStruct);

    Clock_setPeriod(clkKeyHandle, buttonsClkTime /*Rkps_RetTimeInUs(23*8) / 10 *2*/);


    //------------------------------------------------  task init   ------------------------------
    /* Construct BIOS Objects */
    Task_Params taskParams;
    //Mailbox_Params mbxParams;


    /* Construct read and writer tasks */
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)RKPS_Task, &taskParams, NULL);


    //-------------------------------------------------     pin init    ---------------------------
    pinHandle = PIN_open(&pinState, BoardGpioInitTable);

//    PIN_setOutputValue(pinHandle, TD_LED_RED_ID, 0);
//    PIN_setOutputValue(pinHandle, TD_LED_GREEN_ID, 0);
    //PIN_setOutputValue(pinHandle, TD_TCXO_ID, 1);               // TCXO enable    // enable only when RF-core power On

//    PIN_setConfig(pinHandle, PIN_BM_ALL, TD_MODE_ID|PIN_GPIO_OUTPUT_DIS|PIN_INPUT_EN|PIN_PULLUP);           // MODE_Button out buffer disable;
//    PIN_setConfig(pinHandle, PIN_BM_INPUT_MODE, TD_MODE_ID|PIN_INPUT_EN|PIN_PULLUP);         // MODE_Button switch to input;

    //-------------------------------------------------     RF parameters init   ----------------------
    RF_Params_init(&rfParams);

    RF_cmdPropTx.pktLen = PACK_LENGTH;
    RF_cmdPropTx.pPkt = (uint8_t *)&rkps.txArray.F_Code;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    RF_cmdPropTx.startTime = 0;
    RF_cmdPropTx.pktConf.bVarLen = 0;

    RF_cmdPropRx.endTime = 4000000; //4000000 RF_ticks = 1 S
    /* Modify CMD_PROP_RX command for application needs */
    RF_cmdPropRx.pQueue = &dataQueue;           /* Set the Data Entity queue for received data */
    RF_cmdPropRx.pOutput = (uint8_t *) &rfOutStructure;
    RF_cmdPropRx.maxPktLen = PACK_LENGTH;        /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.endTrigger.triggerType = TRIG_REL_START;
    RF_cmdPropRx.endTrigger.bEnaCmd      = 0;   //1;

    //---------------------------------------------------   RKPS handlers array initialisation
    RKPS_OWN_Handler_Array[RKPS_LINK_HANDLER]     = RKPS_Link_Handler;
    RKPS_OWN_Handler_Array[RKPS_LISTEN_HANDLER]   = RKPS_Listen_handler;
    RKPS_OWN_Handler_Array[RKPS_PROG_HANDLER]     = RKPS_Prog_handler;
    RKPS_OWN_Handler_Array[RKPS_SYNC_HANDLER]     = RKPS_Sync_handler;
    RKPS_OWN_Handler_Array[RKPS_MESSAGE_HANDLER]  = RKPS_Message_handler;



    //--------------------------------------------------    Read states and settings from mcu-flash  ---------
    modeButtonState = sizeof(rkps);
    asm("   nop");
    modeButtonState = sizeof(rmlState);
    asm("   nop");
    modeButtonState = sizeof(rmlSettings);
    asm("   nop");

    Rkps_RmlSettingsDefault();

    rmlState.mode = MODE_NO_MODE;

    rmlState.testIndicTimeStart = 0;

    rmlState.packErrCount   = 0;
    rmlState.packCount      = 0;
    rmlState.maxPackErrCount  = 0;
    //------------------------------------------------   saving data to the mcu flash

    Rkps_ReadSettingsFromFlash();

    rmlSettings.name = DEVICE_NAME;
    rmlSettings.type = DEVICE_TYPE;

    Rkps_ReadFlagsFromFlash();

    if(rmlState.flags.indon)
        rmlState.flags.indon = 0;

    PIN_setConfig(pinHandle, PIN_BM_ALL, TD_MODE_ID|PIN_GPIO_OUTPUT_DIS|PIN_INPUT_EN|PIN_PULLUP);           // MODE_Button out buffer disable;
    modeButtonState = PIN_getInputValue(TD_MODE_ID);
    if(modeButtonState == 0)
    {
        rmlState.flags.proged = 0;
    }

    PIN_setConfig(pinHandle, PIN_BM_ALL, TD_TAMPER_ID|PIN_GPIO_OUTPUT_DIS|PIN_INPUT_EN|PIN_PULLUP);           // TAMPER_Button out buffer disable;
    modeButtonState = PIN_getInputValue(TD_TAMPER_ID);
    if(modeButtonState == 0)
    {
        rmlState.flags.proged = 1;
        // switch to the PIR-control mode
        rmlState.mode = MODE_PIR_TEST;
    }


    //-------------------------------------------------------------------   Crc calculation test    -----------------------
    char buf[] = "123456789";
    long crc = Crc_16(buf, sizeof(buf)-1);
    if(crc)
        asm("   nop");
    else
        asm("   nop");
    //------------------------------------------------------------------    Fixed point calculation test    -------------------
    //float fTI = 32000.0/32768.0 * 1000;
    unsigned long long  number = 0x1676000000, // 22,4609375 в виде числа с фикс запятой 32 на целое, 32 на дробную часть //(unsigned long long)(fNum * (1LL << 32)),    //
                        temp = Rkps_RetTimeInUs(23*320),
                        mult = 10000000,
                        TI = Rkps_RetTimeInUs(4);
    temp = number * mult;

    if (temp)
    {
        asm("   nop");
    }
    temp = TI * mult;
    temp *= 23;                 // OWN time

    asm("   nop");

    //------------------------------------------------------    scenario calculation
    Rkps_CalculMyOwnNumber();
    Rkps_CalculScenario();
    rkps.shiftTime = 0;
    rkps.shiftTimeMinus = 0;
    rkps.haveCmd = 0;
    //------------------------------------------------------    rkps message queue initialization
    RkpsQ_Free();

    if(rmlState.mode & MODE_PIR_TEST)
    {
        UART_init();
        wasCallBack = 1;
    }
    else
    {
        INDICAT_Init();
    }
}


/**
 *
 */
void RKPS_Task(UArg arg0, UArg arg1)
{
    MsgObj msg;
    UArg arg = RKPS_PROG_HANDLER;

    if(rmlState.flags.proged)
    {
        arg = RKPS_SYNC_HANDLER;
    }

    if(rmlState.mode & MODE_PIR_TEST)
    {
        // Create a UART with data processing off.
        UART_Params_init(&uartParams);
        uartParams.writeDataMode = UART_DATA_BINARY;
        uartParams.readDataMode = UART_DATA_BINARY;
        uartParams.readReturnMode = UART_RETURN_FULL;
        uartParams.readEcho = UART_ECHO_OFF;
        uartParams.baudRate = 57600;    //115200;   //9600;
        uartParams.writeMode = UART_MODE_CALLBACK;
        uartParams.writeCallback = UART_SomeCallBack;
        // Open an instance of the UART drivers
        uart = UART_open(Board_UART0, &uartParams);
        if (uart == NULL) {
          // UART_open() failed
          while (1);
        }
    }


    //------------------------------------------------------    Sensor controller config
    scifOsalInit();
    scifInit(&scifDriverSetup);
    uint32_t rtc_Hz = /*10*/ 50;
    scifStartRtcTicksNow(0x00010000 / rtc_Hz);
//    scifTaskData.irSensor.input.lightLine = 2000;         //3900
//    scifTaskData.irSensor.input.isLedIndication = 0;

    if(rmlState.mode & MODE_PIR_TEST)
        scifTaskData.analyseIk.input.isLogMode = 1;
    else
        scifTaskData.analyseIk.input.isLogMode = 0;

    if(rmlState.flags.armed)
        scifTaskData.analyseIk.input.isArmed = 1;
    else
        scifTaskData.analyseIk.input.isArmed = 0;

    scifOsalRegisterTaskAlertCallback(SC_AlertCallback);                    // register SC callback
	scifStartTasksNbl(BV(SCIF_ANALYSE_IK_TASK_ID));                          // run SC-task

	Rkps_StartClockEntity(arg, 100000);                                     // run handler carousel

	Clock_start(clkKeyHandle);
	
    while(1)
    {
        Mailbox_pend(mbxHandle, &msg, BIOS_WAIT_FOREVER);
        msg.val = mailboxBuffer[0].obj.val;

        PIN_setOutputValue(pinHandle, TD_TCXO_ID, 1);                       // TCXO enable
        /* Request access to the radio */
        rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);


        if( RFQueue_defineQueue(&dataQueue, rxDataEntryBuffer, sizeof(rxDataEntryBuffer), NUM_DATA_ENTRIES, PACK_LENGTH + NUM_APPENDED_BYTES))
        {
            /* Failed to allocate space for all data entries */
            while(1);
        }

        //--------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------
        //---------------------------   OWN handler    -----------------------------------
                            RKPS_OWN_Handler_Array[msg.val]();
        //--------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------


        if(rfHandle)
            RF_close(rfHandle);

        // TCXO disable
        PIN_setOutputValue(pinHandle, TD_TCXO_ID, 0);

        Rkps_CmdHandle();

        msg.val = 0;
    }
}


/**
 *
 */
void SC_AlertCallback(void)
{
    unsigned long timeout = 0;

    char pcPack[40];        // pc packet
    char packSize = 0;

    unsigned long adcData = 0;  //scifTaskData.irSensor.output.thermo;

    char nextHandlerId = RKPS_MESSAGE_HANDLER;
    // Clear the ALERT interrupt source
    scifClearAlertIntSource();

//    PIN_setOutputValue(pinHandle, TD_X12_ID, 1);

    if (rmlState.mode & MODE_PIR_TEST)
    {
        if(wasCallBack)
        {
            wasCallBack = 0;

            Rkps_MakePcPacket(&scifTaskData, pcPack, &packSize);
            UART_write(uart, pcPack, packSize);
//            textLen = snprintf(text, sizeof(text), "PIR = %d\n\r", scifTaskData.analyseIk.output.Upiri);
//            UART_write(uart, text, textLen);
        }
    }
    else
    {
        //if(scifTaskData.irSensor.state.alarmState)
        if(scifTaskData.analyseIk.output.PirPulseCounter == 2)
        {
            if((rmlState.flags.armed) && (rmlState.mode & MODE_LINKED))
            {
                RkpsQ_Push(DEVICE_ALARM, 1, 0);
                rmlState.flags.alarm = 1;
            }
            rmlState.sens[ALARM_SENSOR].state = 1;
        }
        else
        {
            if((rmlState.flags.armed) && (rmlState.mode & MODE_LINKED))
            {
                RkpsQ_Push(DEVICE_ALARM, 0, 0);
            }
            rmlState.sens[ALARM_SENSOR].state = 0;
            rmlState.flags.alarm = 0;
        }

        if ((rmlState.flags.armed == 0) && (rmlState.mode & MODE_TEST_INDON))
        {
           INDICAT_Set(INDICAT_RED_TEST_MODE);
           RKPS_ButtonClkChangePeriodRestart(30);
        }

        timeout = Rkps_CalculFreeOwnTime();

        if(RkpsQ_HaveSomeMsg())
            Rkps_StartClockEntity(RKPS_MESSAGE_HANDLER, timeout);

    }
    // Acknowledge the ALERT event
    scifAckAlertEvents();
}

/**
 *
 */
Void clkSwitchRkpsFxn(UArg arg0)
{
    MsgObj msg1;

    msg1.val = arg0;
    Mailbox_post(mbxHandle, &msg1, BIOS_WAIT_FOREVER);
    mailboxBuffer[0].obj.val = arg0;
    startTime = Clock_getTicks();
}



/**
 * Ф-ция проверяет кнопки PROG и TAMP на нажатие
 * Ф-ция вызывается в RML окне другого датчика, т.е. где данному RML нет необюходиомости что-то передавать или принимать
 * ф-ция вызывается за 4 мс до конца окна. Т.о. 2мс на обработку кнопок и 2 мс на ЗИ.
 * ЗИ необходимо выдержать, чтобы конец обработки кнопок приходился на конец окна
 * и механизм передачи сообщений смог синхронизироваться.
 */
Void clkKeybuttonFxn(UArg arg0)
{
    static char keyMode = 0, keyTamp = 0;
    char    stateIndx = 0,
            isChangedIndx = 4,
            haveMsg = 0;
    unsigned int curState = 0;
    unsigned long   timeout = 0,
                    adcAdjVal = 0,
                    curTime = Clock_getTicks();

    PIN_setOutputValue(pinHandle, TD_X12_ID, 1);

    PIN_setConfig(pinHandle, PIN_BM_ALL, TD_TAMPER_ID   |PIN_GPIO_OUTPUT_DIS|PIN_INPUT_EN|PIN_PULLUP);           //
    PIN_setConfig(pinHandle, PIN_BM_ALL, TD_MODE_ID     |PIN_GPIO_OUTPUT_DIS|PIN_INPUT_EN|PIN_PULLUP);           //

//    while((Clock_getTicks() - curTime) < (Rkps_RetTimeInUs(1) / 10)); // жду 1 ТИ
    //-------------------------------------------------------------------------------
    curState = PIN_getInputValue(TD_MODE_ID);
    if(curState != (keyMode & (1 << stateIndx)))
    {
        if(keyMode & (1 << stateIndx))
            keyMode &= ~(1 << stateIndx);
        else
            keyMode |= (1 << stateIndx);
        keyMode |= (1 << isChangedIndx);
    }
    else
        keyMode &= ~(1 << isChangedIndx);
    if(keyMode & (1 << isChangedIndx))
    {
        if(keyMode & (1 << stateIndx))          // MODE button is unpressed
        {

        }
        else                                    // MODE button is pressed
        {

        }
    }

    //-------------------------------------------------------------------------------
    curState = PIN_getInputValue(TD_TAMPER_ID);
    if(curState != (keyTamp & (1 << stateIndx)))
    {
        if(keyTamp & (1 << stateIndx))
            keyTamp &= ~(1 << stateIndx);
        else
            keyTamp |= (1 << stateIndx);
        keyTamp |= (1 << isChangedIndx);
    }
    else
        keyTamp &= ~(1 << isChangedIndx);

    if(keyTamp & (1 << isChangedIndx))
    {
        if(keyTamp & (1 << stateIndx))
        {
            RkpsQ_Push(OPEN_CASE, 1, 0);
        }
        else
        {
            RkpsQ_Push(OPEN_CASE, 0, 0);
            if(rmlState.flags.armed == 0)
            {
                rmlState.mode |= MODE_TEST_INDON;
                rmlState.testIndicTimeStart = Clock_getTicks();
                scifTaskData.analyseIk.input.isTestMode = 1;
            }
        }
        haveMsg = 1;
    }
    if((Clock_getTicks() - rmlState.testIndicTimeStart > 6000000) && (rmlState.flags.armed == 0))
    {
        rmlState.mode &= ~MODE_TEST_INDON;
        scifTaskData.analyseIk.input.isTestMode = 0;
        RKPS_ButtonClkChangePeriodRestart(100);
    }
    if(haveMsg)
    {
        timeout = Rkps_CalculFreeOwnTime();
        Rkps_StartClockEntity(RKPS_MESSAGE_HANDLER, timeout);
    }

//    Clock_setTimeout(clkKeyHandle, timeout/10);
//    Clock_start(clkKeyHandle);


    PIN_setConfig(pinHandle, PIN_BM_ALL, TD_TAMPER_ID   | PIN_GPIO_OUTPUT_EN | PIN_PUSHPULL | PIN_GPIO_LOW);           //
    PIN_setConfig(pinHandle, PIN_BM_ALL, TD_MODE_ID     | PIN_GPIO_OUTPUT_EN | PIN_PUSHPULL | PIN_GPIO_LOW);           //

//    adcAdjVal = AUXADCAdjustValueForGainAndOffset(scifTaskData.irSensor.output.battery,
//                                                  AUXADCGetAdjustmentGain(AUXADC_REF_VDDS_REL),
//                                                  AUXADCGetAdjustmentOffset(AUXADC_REF_VDDS_REL));
//    rmlState.batVoltage1 = adcAdjVal * 1800 * 22 / 4096 / 10;
//
//    adcAdjVal = AUXADCAdjustValueForGainAndOffset(scifTaskData.irSensor.output.thermo,
//                                                 AUXADCGetAdjustmentGain(AUXADC_REF_VDDS_REL),
//                                                 AUXADCGetAdjustmentOffset(AUXADC_REF_VDDS_REL));

    rmlState.curTemper1 = TemperatureCalcul(adcAdjVal);

    Rkps_FlashSavingHandle();

    INDICAT_Step();
//    if(LED_Indication == 1)
//    {
//    }
//    else if(LED_Indication == 2)
//    {
//    }
}

/**
 *
 */
void RKPS_Link_Handler(void)
{
    char nextHandlerId;
    unsigned long   timeout = 0,
                    testInd = TD_LED_GREEN_ID;
    unsigned short deltaOwn = 0;
    unsigned long curTime = Clock_getTicks();

    rfRxDone = 0;
    rfTxDone = 0;

    if(rmlState.flags.armed)
        testInd = TD_LED_RED_ID;

    rkps.cfCount++;

    RF_cmdPropRx.endTime = 4000 * 20;     // 20 ms rxTimeout
    RF_cmdFs.frequency = freqArr[rmlSettings.freqChanIndexAr[0]-1][0];
    RF_cmdFs.fractFreq = freqArr[rmlSettings.freqChanIndexAr[0]-1][1];
    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);         // was post, now run

    PIN_setOutputValue(pinHandle, TD_X12_ID, 1);

    RF_cmdPropTx.pPkt = (uint8_t *)&rkps.txArray.F_Code;
    if(!RkpsQ_IsEmpty())
    {
        RkpsQ_Restore();
//        Rkps_FillMsg();
    }

    Rkps_FillLink();

    while((Clock_getTicks() - curTime) < (Rkps_RetTimeInUs(2) / 10)); // жду 2 ТИ

    PIN_setOutputValue(pinHandle, TD_X12_ID, 0);

//    PIN_setOutputValue(pinHandle, testInd, 1);

    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, &Rkps_RF_TX_okIndicat, IRQ_TX_DONE);            // was run
    while(!rfTxDone);

    rmlState.packCount++;
    if(rmlState.packCount == 255)
    {
        rmlState.packCount      = 0;
        rmlState.packErrCount   = 0;
    }

    PIN_setOutputValue(pinHandle, TD_X12_ID, 1);

    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &Rkps_RF_RX_okIndicat, IRQ_RX_ENTRY_DONE);                     // was run
    while(!rfRxDone);

    if(rfRxDone == 2)               // havn`t receive the prog-packet
    {
        deltaOwn = 10;
        timeout = Rkps_RetTimeInUs(23*deltaOwn);  //~230 ms Timeout

        rmlState.packErrCount++;

        if(rmlState.packErrCount > rmlState.maxPackErrCount)
            rmlState.maxPackErrCount = rmlState.packErrCount;

        nextHandlerId = RKPS_SYNC_HANDLER;
    }
    else if (rfRxDone == 1)
    {
        rmlState.mode |= MODE_LINKED;

        rmlState.Lq[RFS_TRANS_1][RFS_FREQ_1].Down = rfOutStructure.lastRssi;

        Rkps_CalculShiftTime();

        if(rkps.rxArray.Pack1.id != ACK)
            rkps.haveCmd = 1;
        else
            rkps.haveCmd = 0;

        if((rmlState.flags.armed) && (rkps.haveCmd == 0))
        {
            deltaOwn = 320*8 - 2;                                 //link in 8 CF
        }
        else
        {
            if(rkps.haveCmd == 1)
                deltaOwn = 320 - 2;                                    // link in next CF
            else
            {
                deltaOwn = 320-1;                                 //listen in ACK-own
                rkps.curOwnNum = rkps.myOwnNum+1;
                timeout = Rkps_RetTimeInUs(23*deltaOwn + 2 + 1/*4*/);
                if(rkps.shiftTimeMinus)
                   timeout -= rkps.shiftTime * 10 / 16;
                else
                   timeout += rkps.shiftTime * 10 / 16;

                nextHandlerId = RKPS_LISTEN_HANDLER;
                Rkps_StartClockEntity(nextHandlerId, timeout);

                deltaOwn = 320*16 - 2;
            }
        }
        rkps.curOwnNum = rkps.myOwnNum+1;
        timeout = Rkps_RetTimeInUs(23*deltaOwn + 2 + 1 /*4*/);
        if(rkps.shiftTimeMinus)
           timeout -= rkps.shiftTime * 10;
        else
           timeout += rkps.shiftTime * 10;
        if(rkps.shiftTime == 0)                      // если первый заход в Link и попытка попасть в следующий Link точно
           timeout += Rkps_RetTimeInUs(1);
        nextHandlerId = RKPS_LINK_HANDLER;

        if(Clock_isActive(clkKeyHandle))
            Clock_stop(clkKeyHandle);

        Clock_setTimeout(clkKeyHandle, Rkps_RetTimeInUs(23/**2*/ +20 +3 +4)/10);    // попасть в окно другого РМЛ с опозданием 4ti
        Clock_start(clkKeyHandle);
    }
    PIN_setOutputValue(pinHandle, TD_X12_ID, 0);
//    PIN_setOutputValue(pinHandle, testInd, 0);
    Rkps_StartClockEntity(nextHandlerId, timeout);

    rkps.cfTime = Clock_getTicks() + 318;// +3180 uS    // + (Rkps_RetTimeInUs(2)/10); //  /10 because sysTick = 10 uS
}


/**
 *
 */
void RKPS_Listen_handler(void)
{
    char nextHandlerId;
    unsigned long timeout = 0;
    unsigned long curTime = Clock_getTicks();

    static char in = 0;

    rfRxDone = 0;
    rfTxDone = 0;

    in++;
    RF_cmdPropRx.endTime = 4000 * 6;     // 6 ms rxTimeout
    RF_cmdFs.frequency = freqArr[rmlSettings.freqChanIndexAr[0]-1][0];
    RF_cmdFs.fractFreq = freqArr[rmlSettings.freqChanIndexAr[0]-1][1];
    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);             // was post

    PIN_setOutputValue(pinHandle, TD_X12_ID, 1);

    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &Rkps_RF_RX_okIndicat, IRQ_RX_ENTRY_DONE);                              // was run
    while(!rfRxDone);

    rkps.curOwnTime =  Clock_getTicks() - curTime;
    if(rfRxDone == 2)               // havn`t receive a packet
    {
        if(RkpsQ_HaveSomeMsg())
        {
            // жду конца окна - 2ТИ, т.к. после этого надо будет передавать сообщение.
            // Эта строка поможет синхронизоваться механизму отправки сообщений
            while(Clock_getTicks() - curTime < Rkps_RetTimeInUs(23-2)/10);

            rkps.curOwnTime =  Clock_getTicks() - curTime;
        }
        if(in == 15)
            timeout = 0;//Rkps_RetTimeInUs(23 * (320 - 1)) - 7800;                // 319 OWN - 6 mS
        else
            timeout = Rkps_RetTimeInUs(23 * 320 /*- 2*/) - rkps.curOwnTime*10 - 400 /*6100*/ /*6000*/ /*7600*/;                       // 320 OWN - 2TI - 6 mS

        if(Clock_isActive(clkKeyHandle))
           Clock_stop(clkKeyHandle);

       //Clock_setTimeout(clkKeyHandle, Rkps_RetTimeInUs(23*2 + 20 + 3)/10);
        Clock_setTimeout(clkKeyHandle, Rkps_RetTimeInUs(23/**2*/ +20 +3 +4)/10);    // попасть в окно другого РМЛ с опозданием 4ti
       Clock_start(clkKeyHandle);
    }
    else
    {
       in = 15;  // if some pack is received, then out in MY_RML_OWN
       timeout = Rkps_RetTimeInUs(23 * Rkps_CalcDetlaOwn(rkps.myOwnNum+1, OW_MY_RML) + 2);

       rkps.haveCmd = 1;                // need check if RML have cmd.
    }
    nextHandlerId = RKPS_LISTEN_HANDLER;
    if(in == 15)
    {
        in = 0;
        nextHandlerId = RKPS_LINK_HANDLER;
    }
    PIN_setOutputValue(pinHandle, TD_X12_ID, 0);

    rkps.curOwnTime =  Clock_getTicks() - curTime;
    rkps.curOwnNum = rkps.myOwnNum+1;
    rkps.cfCount++;

    if(timeout)
        Rkps_StartClockEntity(nextHandlerId, timeout);
}


/** change call function __ period.
 * peiodMulti in percents from "buttonsClkTime" value
 */
void RKPS_ButtonClkChangePeriodRestart(unsigned short periodMulti)
{
    unsigned long   newPeriodVal = buttonsClkTime * periodMulti;
                    newPeriodVal /= 100;

    Clock_stop(clkKeyHandle);
    Clock_setPeriod(clkKeyHandle, newPeriodVal);
    Clock_start(clkKeyHandle);
}

void RKPS_Prog_handler(void)
{
    static char stage = 0;

    char nextHandlerId;
    char indicationPhase = 0;
    unsigned long indicationTime = 10000;
    unsigned long timeout = 0;
    unsigned short deltaOwn = 0;
    unsigned long curTime = 0;

    rfRxDone = 0;
    rfTxDone = 0;

    rmlState.mode |= MODE_PROGRAMM;
    rmlState.mode &= ~MODE_TEST_INDON;
    RKPS_ButtonClkChangePeriodRestart(100);

    INDICAT_Set(INDICAT_PROG_MODE);

    if(stage == 3)               // если in > 10 то можно уходить в бесконечный сон, программирование РМЛ не состоялось
    {
        stage = 0;
    }

    // if sc is started, then stop it
//    if(scifGetActiveTaskIds() & BV(SCIF_IR_SENSOR_TASK_ID))   // if sc is started, then stop it
//    {
//        scifStopTasksNbl(BV(SCIF_IR_SENSOR_TASK_ID));
//    }



    RF_cmdFs.frequency = freqArr[FREQ_PROG_CHAN][0];
    RF_cmdFs.fractFreq = freqArr[FREQ_PROG_CHAN][1];

    RF_cmdPropRx.endTime = 4000000 * 6;                     // RfRx-end_time = 6 S (6с, чтобы прослушать 2 кадра контроля)
    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);                                             // was post


//    PIN_setOutputValue(pinHandle, TD_X12_ID, 1);
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &Rkps_RF_RX_okIndicat, IRQ_RX_ENTRY_DONE);       // was run
    curTime = Clock_getTicks();
    while(!rfRxDone);


//    PIN_setOutputValue(pinHandle, TD_X12_ID, 0);
    curTime = Clock_getTicks();

//    PIN_setOutputValue(pinHandle, TD_LED_GREEN_ID, 0);

    if (rfRxDone == 2)               // havn`t receive the prog-packet
    {
        //stage = 0;
        nextHandlerId = RKPS_PROG_HANDLER;
        timeout = Rkps_RetTimeInUs(23 * 10);                // 10 OW s
    }
    else if (rfRxDone == 1)         // have received prog packet
    {
        if(rkps.rxArray.Pack1.id == CMD_RML_PROGRAM0)
        {
            if(stage == 2)
                stage = 3;
            else
                stage = 1;

            if (rmlSettings.type == rkps.rxArray.Pack1.data.prog0.P1Type_P2LoopCtrl)                 // if packet type-parameter match with device type, then...
            {
                rmlSettings.addr = rkps.rxArray.Pack1.data.prog0.P1Addr_P2AesByte;                     // get address
            }
            if(rkps.rxArray.Pack2.data.prog0.P1Type_P2LoopCtrl == 1)                                // if packet control loop == 1, then...
                rmlSettings.loop.isControl = 1;                                             // set loop control flag
            else
                rmlSettings.loop.isControl = 0;                                         // else: reset loop control flag

            rmlSettings.aes128Key[0] = rkps.rxArray.Pack2.data.prog0.P1Addr_P2AesByte;  // get AES128 Byte

            if(stage == 3)
                Rkps_FillProgAckEnd();
            else
            {
                Rkps_FillProg0Ack();                                                        // rkps.txArray: fill the ACK-packet for the first PROG-pack

                nextHandlerId = RKPS_PROG_HANDLER;

                deltaOwn = 150; //158;//Rkps_CalcDetlaOwn(unsigned short curOwn, char nextOwnType);
                timeout = Rkps_RetTimeInUs(23*(deltaOwn) + 2);             // сразу после приема прог-пакета ставлю на 160-2 окна и +2ТИ
            }
        }
        else if (rkps.rxArray.Pack1.id == CMD_RML_PROGRAM1)
        {
            if(stage == 1)
                stage = 3;
            else
                stage = 2;
            rmlSettings.systemMac = ((unsigned short)rkps.rxArray.Pack1.data.prog1.P1MAC2_P2Freq2 << 8) + rkps.rxArray.Pack1.data.prog1.P1MAC1_P2Freq1;

            rmlSettings.freqChanIndexAr[RFS_FREQ_1] = rkps.rxArray.Pack2.data.prog1.P1MAC1_P2Freq1;  // get radio channels indexes
            rmlSettings.freqChanIndexAr[RFS_FREQ_2] = rkps.rxArray.Pack2.data.prog1.P1MAC2_P2Freq2;

            if(stage == 3)
                Rkps_FillProgAckEnd();
            else
            {
                Rkps_FillProg1Ack();                                                        // rkps.txArray: fill the ACK-packet for the second PROG-pack
                nextHandlerId = RKPS_PROG_HANDLER;
                deltaOwn = 150; //158;//Rkps_CalcDetlaOwn(unsigned short curOwn, char nextOwnType);
                timeout = Rkps_RetTimeInUs(23*(deltaOwn) + 2);
            }
        }

        if(stage == 3)              // this is the programm mode end
        {
            nextHandlerId = RKPS_SYNC_HANDLER;
            deltaOwn = 2;
            timeout = Rkps_RetTimeInUs(23*deltaOwn + 2);                     // ждать два окна и ловить синхропакет :)
        }

        while(Clock_getTicks() - curTime < Rkps_RetTimeInUs(4) / 10);            // wait 2 TI
//        PIN_setOutputValue(pinHandle, TD_X12_ID, 1);

        RF_cmdPropTx.pPkt = (uint8_t *)&rkps.txArray.F_Code;

        RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, &Rkps_RF_TX_okIndicat, IRQ_TX_DONE);            // was run
        while(!rfTxDone);


//        PIN_setOutputValue(pinHandle, TD_X12_ID, 0);
    }

    if(stage == 3)
    {
        //PIN_setOutputValue(pinHandle, TD_LED_RED_ID, 1);
        stage = 0;                  // device have all data RKPS net
        Rkps_CalculMyOwnNumber();
        Rkps_CalculScenario();

        Rkps_SetFlagState(RML_FL_PROGED, 1);
        Rkps_SetFlagState(RML_FL_ARMED, 0);
        rmlState.flags.inFlash = 1;

        Rkps_FlashSavingHandle();

        rmlState.mode &= ~MODE_PROGRAMM;

        SysCtrlSystemReset ();                          // system reset!!!!
    }
    Rkps_StartClockEntity(nextHandlerId, timeout);
}



void RKPS_Sync_handler(void)
{
    char nextHandlerId = 0;
    unsigned long timeout = 0;              //Rkps_RetTimeInUs(23*30 /*38*/);
    char attCounter = 0;                    // счетчик попыток синхронизироваться с сетью
    unsigned short  deltaOwn = 0;

    rfRxDone = 0;
    rfTxDone = 0;

    rkps.shiftTime       = 0;
    rkps.shiftTimeMinus  = 0;
    rkps.curOwnTime      = 0;
    rkps.haveCmd         = 0;

    RF_cmdPropRx.endTime = 4000 * 2000;     // 2000 ms rxTimeout
    RF_cmdFs.frequency = freqArr[rmlSettings.freqChanIndexAr[0]-1][0];
    RF_cmdFs.fractFreq = freqArr[rmlSettings.freqChanIndexAr[0]-1][1];
    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);                 // was post

    while((rfRxDone == 0) && (attCounter < 120))
    {
        PIN_setOutputValue(pinHandle, TD_X12_ID, 1);

        RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &Rkps_RF_RX_okIndicat, IRQ_RX_ENTRY_DONE);                  // now post
        while(!rfRxDone);

//        PIN_setOutputValue(pinHandle, TD_X12_ID, 0);

        if(rfRxDone == 2)               // havn`t receive the sync-packet
        {
            deltaOwn = 10;
            nextHandlerId = RKPS_SYNC_HANDLER;
            timeout = Rkps_RetTimeInUs(23*deltaOwn);  //~230 ms Timeout

            rmlState.mode &= ~MODE_LINKED;
        }
        else if (rfRxDone == 1)
        {
            if(rkps.rxArray.Pack1.id == SYNC)        // check if it synchro-pack
            {
                rmlState.mode |= MODE_LINKED;

                //syncOwnNum = rkps.rxArray.Pack1.data.sync.syncNum;
                rmlState.Lq[RFS_TRANS_1][RFS_FREQ_1].Down = rfOutStructure.lastRssi;
                nextHandlerId = RKPS_LINK_HANDLER;
                deltaOwn = Rkps_CalcDetlaOwn(rkps.rxArray.Pack1.data.sync.syncNum, OW_MY_RML);
                timeout = Rkps_RetTimeInUs(23*deltaOwn + 2 + 1);

//                if( !(scifGetActiveTaskIds() & BV(SCIF_IR_SENSOR_TASK_ID)) )  // if sc isn`t started, then start it
//                {
//                    scifStartTasksNbl(BV(SCIF_IR_SENSOR_TASK_ID));
//                }
            }
            else
            {
                nextHandlerId = RKPS_SYNC_HANDLER;
                timeout = Rkps_RetTimeInUs(23 * 10);  //~230 ms Timeout
            }
        }
        attCounter++ ;
        PIN_setOutputValue(pinHandle, TD_X12_ID, 0);
    }
    rkps.rxArray.Pack1.id = CMD_RMM_QUANT;
    Rkps_StartClockEntity(nextHandlerId, timeout);
}



void RKPS_Message_handler(void)
{
    unsigned long curTime = Clock_getTicks();
    static char in = 0;

    rfRxDone = 0;
    rfTxDone = 0;

    RF_cmdPropRx.endTime = 4000 * 20;     // 20 ms rxTimeout
    RF_cmdFs.frequency = freqArr[rmlSettings.freqChanIndexAr[0]-1][0];
    RF_cmdFs.fractFreq = freqArr[rmlSettings.freqChanIndexAr[0]-1][1];
    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);                          // was post

    PIN_setOutputValue(pinHandle, TD_X12_ID, 1);
    RF_cmdPropTx.pPkt = (uint8_t *)&rkps.txArray.F_Code;

    Rkps_FillMsg(); //<<<<< fill pack insert here

    while(Clock_getTicks() - curTime < Rkps_RetTimeInUs(2) / 10); // жду 2 ТИ

    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, &Rkps_RF_TX_okIndicat, IRQ_TX_DONE);            // was run
    while(!rfTxDone);

    rmlState.packCount++;

    if(rmlState.packCount == 255)
    {
        rmlState.packCount      = 0;
        rmlState.packErrCount   = 0;
    }

    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &Rkps_RF_RX_okIndicat, IRQ_RX_ENTRY_DONE);                              // was run
    while(!rfRxDone);

    if(rfRxDone == 2)               // havn`t receive the prog-packet
    {
        Rkps_StartClockEntity(RKPS_SYNC_HANDLER, 10000);
        rmlState.packErrCount++;
    }
    else if (rfRxDone == 1)
    {
        rmlState.mode |= MODE_LINKED;

        rmlState.Lq[RFS_TRANS_1][RFS_FREQ_1].Down = rfOutStructure.lastRssi;
        RkpsQ_Clean();

        if(rkps.rxArray.Pack1.id != ACK)
            rkps.haveCmd = 1;
        else
            rkps.haveCmd = 0;
    }
    rkps.curOwnNum = rkps.msgOwnNum + 1;
    PIN_setOutputValue(pinHandle, TD_X12_ID, 0);
}




//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
//----------------------------------------------------------    static section  -----------------
//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
static void UART_SomeCallBack(void)
{
    wasCallBack = 1;
}



static void Rkps_ClockInit(Clock_Params *clkParam, Clock_Handle * clkHandle, Clock_Struct * clkStruct)
{
    Clock_Params_init(clkParam);
    clkParam->period = 0;                        // period = 0, there is no periodic call
    clkParam->startFlag = FALSE /*TRUE*/;                               // wait start
    clkParam->arg = 0x0;
    clkParam->period = 0;
    clkParam->startFlag = FALSE;
    /* Construct a periodic Clock Instance */
    Clock_construct(clkStruct, (Clock_FuncPtr)clkSwitchRkpsFxn, 0, clkParam);

    *clkHandle = Clock_handle(clkStruct);
}



static void Rkps_StartClockEntity(char ownHandlerNum, unsigned long tickQuant)
{
    Clock_Handle clkEntityHandler;
    char error = 0;
    switch (ownHandlerNum)
    {
        case RKPS_LINK_HANDLER:
            clkEntityHandler = clkLinkHandle;
            break;
        case RKPS_LISTEN_HANDLER:
            clkEntityHandler = clkListenHandle;
            break;
        case RKPS_MESSAGE_HANDLER:
            clkEntityHandler = clkFreeHandle;
            break;
        case RKPS_SYNC_HANDLER:
            clkEntityHandler = clkSyncHandle;
            break;
        case RKPS_PROG_HANDLER:
            clkEntityHandler = clkProgHandle;
            break;
        default :
            error = 1;
            break;
    }

    if(error)
        while (1);                              // error!!!!

    Clock_setFunc(clkEntityHandler, clkSwitchRkpsFxn, ownHandlerNum);
    Clock_setTimeout(clkEntityHandler, tickQuant / 10);
    Clock_start(clkEntityHandler);
}



static void Rkps_RF_RX_okIndicat(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    rfc_dataEntryGeneral_t* currentDataEntry;
    if (e & RF_EventRxEntryDone)
    {
        if(rfOutStructure.nRxNok)
        {
            rfOutStructure.nRxNok = 0;
            rfRxDone = 2;
        }
        else if (rfOutStructure.nRxOk)
        {
            rfOutStructure.nRxOk = 0;
            /* Get current unhandled data entry */
            currentDataEntry = RFQueue_getDataEntry();
            packetDataPointer = (uint8_t*)(&currentDataEntry->data);

            /* Copy the payload + the status byte to the packet variable */
            memcpy((char *)&rkps.rxArray.F_Code, packetDataPointer, PACK_LENGTH);
        }
        RFQueue_nextEntry();

        if(!rfRxDone)
            rfRxDone = 1;
    }
    else
    {
        if(!rfRxDone)
            rfRxDone = 2;                                 //RfRx error
    }
}


static void Rkps_RF_TX_okIndicat(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    rfc_dataEntryGeneral_t* currentDataEntry;
    if (e & RF_EventTxDone)
    {
        rfTxDone = 1;
    }
    else
    {
        if(!rfTxDone)
            rfTxDone = 2;                                 //RfRx error
    }
}

/**
 * calculation based on rkps.rxArr.Pack1.data.ack.shTime
 */
static void Rkps_CalculShiftTime(void)
{
    unsigned long temp = 0;
    unsigned char inShTime = rkps.rxArray.Pack1.data.ack.shTime;

    char shiftTimeMinus = 0;

    if(inShTime >= 128)
    {
        temp = (inShTime - 128) * 64;
        shiftTimeMinus = 1;
    }
    else
    {
        temp = inShTime * 64;
    }

    if((temp % 10) > 5)
    {
        temp = (temp / 10) + 1;
    }
    else
    {
        temp = (temp / 10);
    }
    if(shiftTimeMinus)
    {
        if(rkps.shiftTime < temp)
        {
            rkps.shiftTime = temp - rkps.shiftTime;
            rkps.shiftTimeMinus = 1;
        }
        else
        {
            rkps.shiftTime = rkps.shiftTime - temp;
            rkps.shiftTimeMinus = 0;
        }
    }
    else
    {
        rkps.shiftTime += temp;
        rkps.shiftTimeMinus = 0;
    }
}

/**
*/
static void Rkps_FillCommon(RKPS_Pack_Type* pMessage)
{
    pMessage->F_Code.Type               = 0;
    pMessage->F_Code.Direction          = 0;
    pMessage->F_Code.Description        = 3;
    pMessage->F_Code.RetransType        = 0;
    pMessage->F_Code.Subordination      = 0;

    pMessage->C_Data.LinkQuality1       = 0;
    pMessage->C_Data.LinkQuality2       = 0;
    pMessage->C_Data.MsgReplay          = 0;
    pMessage->C_Data.TransNum           = 0;

    pMessage->SenderAddr                = rmlSettings.addr;
    pMessage->ReceiverAddr              = 0;
    pMessage->SystemMAC                 = rmlSettings.systemMac;

    pMessage->noise                     = rkps.noiseCounter++;

    pMessage->RetransAddr1              = 0;
    pMessage->RetransAddr2              = 0;
}


static void Rkps_FillProg0Ack(void)
{
    Rkps_FillCommon(&rkps.txArray);
    rkps.txArray.Pack1.id                   = ACK;
    rkps.txArray.Pack1.data.ack.linkBudget  = 0x28/*rmlSettings.type*/;

    rkps.txArray.Pack1.data.ack.shTime      = 0x92;

    //--------- pack2 is empty
    rkps.txArray.Pack2.id       = EMPTY;
}

static void Rkps_FillProg1Ack(void)
{
    Rkps_FillCommon(&rkps.txArray);
    rkps.txArray.Pack1.id                       = ACK;
    rkps.txArray.Pack1.data.ack.linkBudget      = 0x77;

    rkps.txArray.Pack1.data.ack.shTime  = 0x78;

    //--------- pack2 is empty
    rkps.txArray.Pack2.id       = EMPTY;
}

static void Rkps_FillProgAckEnd(void)
{
    Rkps_FillCommon(&rkps.txArray);
    rkps.txArray.Pack1.id                       = ACK;
    rkps.txArray.Pack1.data.ack.linkBudget      = 0x70;

    rkps.txArray.Pack1.data.ack.shTime  = 0x70;

    //--------- pack2 is empty
    rkps.txArray.Pack2.id       = EMPTY;
}


/**
 * Function return time interval in uS
 * input parameter is quntity of TI-tlements (TI-TimeInterval, minimum time in RKPS)
*/
static unsigned long Rkps_RetTimeInUs(unsigned long tiNumber)
{
    unsigned long long ti = 0x3D090000000,
                        retval = (ti * tiNumber) >> 32;
    return (unsigned long) retval;
}

static void Rkps_CalculMyOwnNumber(void)
{
    unsigned short temp = (rmlSettings.addr - 4) % 8;
    unsigned short slot = (rmlSettings.addr - 4) / 8;

    rkps.myOwnNum = slot*40 + temp*4 + 5;
}

/**
 * Function return quantity owns from curOwn and own with type = nextOwnType;
 * curOwn - number of just have ended OWN
 * Call function after radio action in own.
 * quantity owns is equals next needed own number - curOwn - 1
 */
static unsigned short Rkps_CalcDetlaOwn(unsigned short curOwn, char nextOwnType)
{
    unsigned short  deltaOwn = 0,
                    iteration = 0,
                    ownIndex = curOwn;
    while((iteration < CONTROL_FRAME_OWN_QUANT) || (CF[ownIndex] != nextOwnType))
    {
        ownIndex++;
        iteration++;
        if(ownIndex == CONTROL_FRAME_OWN_QUANT)
            ownIndex = 0;
    }
    if(CF[ownIndex] == nextOwnType)
    {
        ownIndex++;             // make own_number from own_index
        if(ownIndex > curOwn)
            deltaOwn = ownIndex - curOwn - 1;
        else
            deltaOwn = CONTROL_FRAME_OWN_QUANT - curOwn + ownIndex - 1;
    }
    return deltaOwn;
}


//------------------------------------------------------------------------------
static uint8_t _crc_ibutton_update(uint8_t crc, uint8_t data)
{
  int i;
  crc = crc ^ data;
  for (i = 0; i < 8; i++)
  {
    if (crc & 0x01)
      crc = (crc >> 1) ^ 0x8C;
    else
      crc >>= 1;
  }
  return crc;
}

//------------------------------------------------------------------------------
static uint8_t _crc8(uint8_t* addr, int size)
{
  int i;
  uint8_t crc = 0;
  for(i=0; i < size; i++)
  {
    crc = _crc_ibutton_update(crc, addr[i]);
  }
  return crc;
}

/**
 *
 */
static void Rkps_MakePcPacket(SCIF_TASK_DATA_T *pScifTaskData, char *pPcPack, char *pOutSize)
{
    char byteIndex = 0;
    char ix = 0;
    char tmpPack[40];
    char size = 0;

    tmpPack[byteIndex++] = 0xC0;        // FEND
    tmpPack[byteIndex++] = 0x40;        // packet type IK
    tmpPack[byteIndex++] = 15; //19;          // packet size
    tmpPack[byteIndex++] = (pScifTaskData->analyseIk.output.Upiri & 0xFF00) >> 8;
    tmpPack[byteIndex++] = pScifTaskData->analyseIk.output.Upiri & 0x00FF;
    tmpPack[byteIndex++] = (pScifTaskData->analyseIk.output.Upirav & 0xFF00) >> 8;
    tmpPack[byteIndex++] = pScifTaskData->analyseIk.output.Upirav & 0x00FF;
    tmpPack[byteIndex++] = (pScifTaskData->analyseIk.output.Uph & 0xFF00) >> 8;
    tmpPack[byteIndex++] = pScifTaskData->analyseIk.output.Uph & 0x00FF;
    tmpPack[byteIndex++] = (pScifTaskData->analyseIk.output.Upl & 0xFF00) >> 8;
    tmpPack[byteIndex++] = pScifTaskData->analyseIk.output.Upl & 0x00FF;
    tmpPack[byteIndex++] = (pScifTaskData->analyseIk.output.Uledi & 0xFF00) >> 8;
    tmpPack[byteIndex++] = pScifTaskData->analyseIk.output.Uledi & 0x00FF;
    tmpPack[byteIndex++] = (pScifTaskData->analyseIk.output.dUled & 0xFF00) >> 8;
    tmpPack[byteIndex++] = pScifTaskData->analyseIk.output.dUled & 0x00FF;
    tmpPack[byteIndex++] = (pScifTaskData->analyseIk.output.LightThrCounter10 & 0xFF00) >> 8;
    tmpPack[byteIndex++] = pScifTaskData->analyseIk.output.LightThrCounter10 & 0x00FF;
    tmpPack[byteIndex++] = pScifTaskData->analyseIk.output.PirPulseCounter * 3;
    tmpPack[byteIndex]  = _crc8(tmpPack, byteIndex);     /*   CRC8    */;

    size = byteIndex+1;

    pPcPack[0] = tmpPack[0];
    ix = 1;
    for(byteIndex = 1; byteIndex < size; byteIndex++)
    {
        switch(tmpPack[byteIndex])
        {
            case 0x0C:  //FEND:
                pPcPack[ix++] = 0xDB;   //(FESC);
                pPcPack[ix++] = 0xDC;   //(FESC_FEND);
                break;
            case 0xDB:  //FESC:
                pPcPack[ix++] = 0xDB;   //(FESC);
                pPcPack[ix++] = 0xDD;   //(FESC_FESC);
                break;
            default:
                pPcPack[ix++] = tmpPack[byteIndex];
                break;
        }
    }
    *pOutSize = ix;
}
/**
 *Function must be called after function Rkps_CalculMyOwnNumber()
 */
static void Rkps_CalculScenario(void)
{
    unsigned short i;

    for(i = 0; i < CONTROL_FRAME_OWN_QUANT / 2; i++)
    {
        CF[i*2] = OW_FREE;
        CF[i*2 + 1] = OW_ACK;
    }
    for(unsigned short j = 0; j < CONTROL_FRAME_OWN_QUANT; j += 40)
    {
        for(i = 4; i <= 11; i++)
        {
            CF[j + 4*(i-3)] = OW_RML;
            CF[j + 4*(i-3) + 1] = OW_RML_ACK;
        }

        CF[0+j] = OW_RMU_SYNC;
        CF[1+j] = OW_RMU_SYNC;

        CF[36+j] = OW_RMU_LINK;
    }
    CF[156] = OW_PROG;
    CF[316] = OW_PROG;

    CF[rkps.myOwnNum-1] = OW_MY_RML;
    CF[rkps.myOwnNum]   = OW_MY_ACK;

    CF[rkps.myOwnNum-1 - 2] = OW_NO_TYPE;
    CF[rkps.myOwnNum-1 + 2] = OW_NO_TYPE;


//    tI = 64;                                            // tOW / 23;
//    tOW = tI * 23;                                  // or cfTime / (8*20*2);
//    zI = tI * 2;                                    // tI * 2;
//    cfTime = 320 * tOW;                         // 320 * tOW or commonPeriodOw / 8;
}

static void Rkps_FillLink(void)
{
    static char in = 0;
    in++;                                   // select TEMP_BAT or LINK_QUALITY
    Rkps_FillCommon(&rkps.txArray);
    rkps.txArray.Pack1.id = LINK;

    if(rmlState.flags.armed)
        rkps.txArray.Pack1.data.link.ackCmd = DEV_ARMED;
    else
        rkps.txArray.Pack1.data.link.ackCmd = DEV_DISARMED;

    rkps.txArray.Pack1.data.link.loop1 = rmlState.sens[ALARM_SENSOR].state;
    rkps.txArray.Pack1.data.link.loop2 = rmlState.sens[SABOT_SENSOR].state;

    if(in % 2)
    {
        rkps.txArray.Pack2.id = TEMP_BAT;
        rkps.txArray.Pack2.data.batteryParams.battery = (char)(rmlState.batVoltage1 / 10 - 200);
        rkps.txArray.Pack2.data.batteryParams.temperature = (char)rmlState.curTemper1;
    }
    else
    {
        rkps.txArray.Pack2.id = LINK_QUALITY;
        rkps.txArray.Pack2.data.linkQuality.rssi = rmlState.packErrCount;   //rmlState.Lq[RFS_TRANS_1][RFS_FREQ_1].Down;
        rkps.txArray.Pack2.data.linkQuality.loop1 = rmlState.sens[LOOP_SENSOR].state;
        rkps.txArray.Pack2.data.linkQuality.loop2 = 0;
    }
}

static void Rkps_FillMsg(void)
{
    char    msgId = 0,
            param1 = 0,
            param2 = 0;

    if(RkpsQ_PullOut(&msgId, &param1, &param2))
    {
        Rkps_FillCommon(&rkps.txArray);
        rkps.txArray.Pack1.id = msgId;
        rkps.txArray.Pack1.data.commPack.param1 = param1;
        rkps.txArray.Pack1.data.commPack.param2 = param2;

        rkps.txArray.Pack2.id = EMPTY;
    }
}


static unsigned long Rkps_CalculFreeOwnTime(void)
{
    unsigned short  curOwnNum,
                    deltaOwn1,
                    deltaOwn2;

    unsigned long   timeout,
                    additionalTime,
                    deltaTime = Clock_getTicks() - rkps.cfTime;

    deltaOwn1 = ((deltaTime*10) / OWN_TIME_US /*Rkps_RetTimeInUs(23)*/) + 2;        // текущее время относительно старта прошлого Link-кадра. +2 т.к. rkps.cfTime это время конца ACK после Link-пакета

    additionalTime = (deltaTime * 10) % OWN_TIME_US;//Rkps_RetTimeInUs(23)
    curOwnNum = ((deltaOwn1 + rkps.myOwnNum) % 320)/* +1 +1*/;

    deltaOwn2 = Rkps_CalcDetlaOwn(curOwnNum, OW_FREE);

    timeout = Rkps_RetTimeInUs(23 * (deltaOwn2 + 1)) - additionalTime;

    return timeout;
}
/**
 * Function handle command, that comes from RKPS-net (from RMM)
 * command in rkps.rxArray
 *
 */
static void Rkps_CmdHandle(void)
{
    unsigned long timeout = 0;
    unsigned short deltaOwn = 0;

    if((rkps.rxArray.Pack1.id < CMD_RMM_QUANT) && (rkps.haveCmd))
    {
        for(char i = 0; i < sizeof(cmdHandlers); i++)
        {
            if(rkps.rxArray.Pack1.id == cmdHandlers[i].cmdId)
            {
                cmdHandlers[i].CmdHandler(rkps.rxArray.Pack1.data.commPack.param1);
                break;
            }
        }
        rkps.haveCmd = 0;
    }
    if(RkpsQ_HaveSomeMsg() )
    {
        deltaOwn = Rkps_CalcDetlaOwn(rkps.curOwnNum, OW_FREE);
        timeout = Rkps_RetTimeInUs(23*deltaOwn + 2);
        rkps.msgOwnNum = rkps.curOwnNum + deltaOwn + 1;
        Rkps_StartClockEntity(RKPS_MESSAGE_HANDLER, timeout);
    }
    rkps.rxArray.Pack1.id = CMD_RMM_QUANT;
}


/**
 *
 */
static void Rkps_ReadSettingsFromFlash (void)
{
    unsigned long flashAddr = FlashSizeGet() - FlashSectorSizeGet() * 2;    // + sizeof(rkps);

    memcpy((char *)&rmlSettings, (char *)flashAddr, sizeof(rmlSettings));

    if(Crc_8((char *)&rmlSettings, sizeof(rmlSettings)-1) != rmlSettings.CRC8)
    {
        Rkps_RmlSettingsDefault();
        rmlState.flags.inFlash = 1;
    }
}

/**
 *
 */
static void Rkps_ReadFlagsFromFlash (void)
{
    unsigned long flashAddr = FlashSizeGet() - FlashSectorSizeGet()*2 + sizeof(rmlSettings);

    memcpy((char *)&rmlFlags, (char *)flashAddr, sizeof(rmlFlags));

    if(rmlFlags.signat == 0x55)
        Rkps_SyncronizeFlags();
    else
    {
        rmlFlags.signat = 0x55;

        rmlFlags.alarmState = 0xffffffff;   // 0xfff...ff is mean 0
        rmlFlags.armState   = 0xffffffff;
        rmlFlags.progState  = 0xffffffff;
        rmlFlags.blockState = 0xffffffff;
        rmlFlags.loopPresState = 0xffffffff;
        rmlFlags.loopRestoreState = 0xffffffff;
        rmlState.flags.inFlash = 1;
        Rkps_SyncronizeFlags();
    }
}


/**
Фукция только устанавливает биты в инверсное состояние.
Проверку на текущее состонияние не выполняет. Предполагается,
что проверка выполнена в ф-ции Rnet_ApplyDynamicFlag(), т.е.
при вызове Rnet_ChangeFlagStateInFlashWord() известно, что состояние надо менять.
*/
static void Rkps_ChangeFlagStateInFlashWord(unsigned long *pFlashWord)
{
    char bitIndex = 0;

    unsigned long tmpWord = *pFlashWord;

    if(tmpWord == 0x00000000)               // was 0 state
    {
        tmpWord = 0xffffffff-2;             // 111111,,,,101b is 1 state
    }
    else
    {
        // find the start of 1-value bit position
        for(bitIndex = 0; bitIndex < sizeof(tmpWord)*8; bitIndex++)
        {
            if(tmpWord & (1 << bitIndex))
                break;
        }
        if(tmpWord & (1 << (bitIndex+1)))       // this bit-configuration is mean 0, (0...0111,,,b) mean 0
            tmpWord &= ~(1 << (bitIndex+1));        // set bit-configuration to 1, (0...0101,,,b) mean 1
        else                                                // (0...0101,,,b) mean 1
            tmpWord &= ~(1 << (bitIndex));      // set bit-configuration to 0, (0...0001,,,b) mean 0
    }
    *pFlashWord = tmpWord;
}

/**
Return flag state based on value from flash
*/
static char Rkps_ReturnFlagState(unsigned long flashValue)
{
    char retval = 0;
    //here bitfield handler
  if(flashValue == 0)
  {
    retval = 1;
  }
  else if(flashValue == 0xffffffff)
  {
    retval = 0;
  }
  else
  {
    for(char i = 0; i < 32; i++)
    {
      if(flashValue & (1 << i))
      {
        if( !(flashValue & (1 << (i+1))) )  // 0 bit value after 1 bit value  => 1
          retval = 1;
        else                        // 1 bit value after 1 bit value => 0
          retval = 0;
        break;
      }
    }
  }
  return retval;
}

/**
apply dynamic flags (armed or alarm) and save in Flash
flagName value from FlagNumbers
*/
static void Rkps_SetFlagState(char flagName, char flagState)
{
    char toFlash = 0;
    char needSave = 1;

    unsigned long *pFlagWord;
    char tmpFlState = 0;

    switch(flagName)
    {
    case RML_FL_ALARM:
            rmlState.flags.alarm = flagState;
            pFlagWord = &rmlFlags.alarmState;
            break;

        case RML_FL_ARMED:
            rmlState.flags.armed = flagState;
            pFlagWord    = &rmlFlags.armState;
            break;

        case RML_FL_PROGED:
            rmlState.flags.proged = flagState;
            pFlagWord    = &rmlFlags.progState;
            break;

        case RML_FL_LOOPPRES:
            rmlState.flags.loopPresent = flagState;
            pFlagWord    = &rmlFlags.loopPresState;
            break;

        case RML_FL_BLOCK:
           rmlState.flags.blocked = flagState;
           pFlagWord    = &rmlFlags.blockState;
           break;

        case RML_FL_LOOP_WAS_RESTORE:
           rmlState.flags.loopWasRestore = flagState;
           pFlagWord    = &rmlFlags.loopRestoreState;
           break;

        case RML_FL_INDON:
           rmlState.flags.indon = flagState;
           needSave    = 0;
           break;

        default:
            break;
    }
    if(needSave)
    {
        tmpFlState = Rkps_ReturnFlagState(*pFlagWord);
        if(flagState != tmpFlState)
        {
            // new zero state for armed flag to dynamic flags sector
            Rkps_ChangeFlagStateInFlashWord(pFlagWord);
            toFlash = 1;
            if(*pFlagWord == 0x00000000)
            {
                *pFlagWord = 0xffffffff;
                toFlash = 2;
            }
        }
        // if need saving to the Flash
        if(toFlash)
        {
            if(toFlash == 2)            // need to resave all flags to the Flash
            {
               rmlState.flags.inFlash = 1;
            }
            else if(toFlash == 1)
            {
                rmlState.flags.partInFlash = 1;
            }
        }
    }
}



static char Rkps_SaveAllToFlash (void)
{
    unsigned long flashAddr = FlashSizeGet() - FlashSectorSizeGet() * 2;
    char retval = 1;

    rmlSettings.CRC8 = Crc_8((uint8_t *)&rmlSettings, sizeof(rmlSettings)-1);       // recalculate CRC8 for rmlSettings

    FlashProtectionSet(flashAddr, FLASH_NO_PROTECT);

    if(FlashProtectionGet(flashAddr) == FLASH_NO_PROTECT)
    {
       FlashSectorErase(flashAddr);
       if(FlashProgram((uint8_t *)&rmlSettings, flashAddr, sizeof(rmlSettings)) == FAPI_STATUS_SUCCESS)
       {

       }
       flashAddr += sizeof(rmlSettings);
       if(FlashProgram((uint8_t *)&rmlFlags, flashAddr, sizeof(rmlFlags)) == FAPI_STATUS_SUCCESS)
       {

       }
    }
    return retval;
}


static char Rkps_SaveRmlFlagsToFlash (void)
{
    unsigned long flashAddr = FlashSizeGet() - FlashSectorSizeGet() * 2 + sizeof(rmlSettings);
    char retval = 0;
    if(FlashProtectionGet(flashAddr) == FLASH_NO_PROTECT)
    {
        //FlashSectorErase(flashAddr);
        if(FlashProgram((uint8_t *)&rmlFlags, flashAddr, sizeof(rmlFlags)) == FAPI_STATUS_SUCCESS)
        {
            retval = 1;
        }
    }
    return retval;
}

/**
 *
 */
static char Rkps_FlashSavingHandle(void)
{
    char retval = 0;
    char isSave = 0;
   // scifTaskData.mkSensorTask1.input.isFlashTime = 1;                   // SC isn`t generate alarm

    IntMasterDisable();

    if(rmlState.flags.inFlash)
    {
        retval = Rkps_SaveAllToFlash();
        if(retval)
            rmlState.flags.inFlash = 0;
        isSave = 1;
    }
    else if(rmlState.flags.partInFlash)
    {
        retval = Rkps_SaveRmlFlagsToFlash();
        if(retval)
            rmlState.flags.partInFlash = 0;
        isSave = 1;
    }
    IntMasterEnable();

    //scifTaskData.mkSensorTask1.input.isFlashTime = 0;                   // SC may generate alarm
    if(isSave)
        return retval;
    else
        return 0;
}
/**
*
*/
static void Rkps_SetArmed_H     (char Off0_On1)
{
    rmlState.flags.armed = 1;
    RkpsQ_Push(DEV_IS_ARM, 1, 0);
    //scifTaskData.irSensor.input.isLedIndication = 0;
    scifTaskData.analyseIk.input.isArmed = 1;
    Rkps_SetFlagState(RML_FL_ARMED, 1);
}

/**
*
*/
static void Rkps_SetDisarmed_H  (char Off0_On1)
{
    rmlState.flags.armed = 0;
    RkpsQ_Push(DEV_IS_DARM, 1, 0);
    Rkps_SetFlagState(RML_FL_ARMED, 0);
    scifTaskData.analyseIk.input.isArmed = 0;
}

/**
*
*/
static void Rkps_Indication_H   (char Off0_On1)
{
    if(Off0_On1)
    {
        INDICAT_Set(INDICAT_LIGHT_ON_MODE);
        rmlState.flags.indon = 1;
        RkpsQ_Push(DEV_INDICATION_IS_ON, 1, 0);
        Rkps_SetFlagState(RML_FL_INDON, 1);
    }
    else
    {
        INDICAT_Set(INDICAT_OFF_MODE);
        rmlState.flags.indon = 0;
        RkpsQ_Push(DEV_INDICATION_IS_ON, 0, 0);
        Rkps_SetFlagState(RML_FL_INDON, 0);
    }
}

/**
 *
 */
static void Rkps_RmlSettingsDefault(void)
{
    memcpy(rmlSettings.aes128Key, "123456789abcdefg", 16);

    rmlState.flags.armed =  0;
    rmlState.flags.alarm =  0;
    rmlState.flags.indon =  0;
    rmlState.flags.proged = 0;

    rmlSettings.name = DEVICE_NAME;             // Jupiter-5231, MSB=1 - then Jupiter, else Saturn
    rmlSettings.systemMac = 0x7788;
    rmlSettings.type = DEVICE_TYPE; //DEVICETYPE_MAGNET;   //DEVICETYPE_INFRARED_PASSIVE; //DEVICETYPE_WATER;
    rmlSettings.loop.isPresent = 1;
    rmlSettings.freqChanIndexAr[0] = 9; // 864,475 MHz
    rmlSettings.hwVer[0] = 0x00;
    rmlSettings.hwVer[1] = 'A';
    rmlSettings.swVer[0] = 0x00;
    rmlSettings.swVer[1] = 'A';
}

/**
 * Для каждого флага есть две переменные: __StIndex и __State.
 * ___StIndex - битовое поле, индексы битов со значением 0 соответствуют индексам битов-состояний в __State,
 * бит = 1 - состояние бита __State с этим индексом не определено
 * __State - битовое поле, хранит состояние флага, 1 - состояние соответвует названию флага, 0 - инвертированное состояние
 */
static char Rkps_SyncronizeFlags(void)
{
    rmlState.flags.alarm = 0;
    rmlState.flags.armed        = Rkps_RetFlag(RML_FL_ARMED);
    rmlState.flags.blocked      = Rkps_RetFlag(RML_FL_BLOCK);
    rmlState.flags.proged       = Rkps_RetFlag(RML_FL_PROGED);
    rmlState.flags.indon        = Rkps_RetFlag(RML_FL_INDON);
    rmlState.flags.loopPresent  = Rkps_RetFlag(RML_FL_LOOPPRES);

    return 0;
}
/**
*
*/
static void Rkps_SwVer_H        (char Off0_On1)
{
}

/**
*
*/
static void Rkps_Blocking_H     (char Off0_On1)
{
}


static char Rkps_RetFlag (char flagNum)
{
   char flState = 0;
   char bitIndex = 0;
   unsigned long *pFlState = 0;
   unsigned long *pFlStIndex = 0;

   switch (flagNum)
   {
   case RML_FL_ALARM:
       pFlState    = &rmlFlags.alarmState;
       break;

   case RML_FL_ARMED:
       pFlState    = &rmlFlags.armState;
       break;

   case RML_FL_PROGED:
       pFlState    = &rmlFlags.progState;
       break;

   case RML_FL_LOOPPRES:
       pFlState    = &rmlFlags.loopPresState;
       break;

   case RML_FL_BLOCK:
      pFlState    = &rmlFlags.blockState;
      break;
   default:
       break;
   }
   flState = Rkps_ReturnFlagState(*pFlState);

   return flState;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//---------------------------------------------------   RKPS out message queue section  -----------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

static void RkpsQ_Free(void)
{
    for(char i = 0; i < RFS_QUEUE_SIZE; i++)
    {
        memset((char *)&rkpsQ.elem[i], 0, sizeof(rkpsQ.elem[i]));
        rkpsQ.elem[i].state = Q_FREE;
    }
    rkpsQ.freeSpace = RFS_QUEUE_SIZE;
}


static char RkpsQ_PullOut(char* pMsgId, char* pParam1, char* pParam2)
{
    char allOk = 0;         // some error
    for(char i = 0; i < 2; i++)
    {
        if(rkpsQ.elem[i].state == Q_NOT_SENDED)
        {
            *pMsgId = rkpsQ.elem[i].msg.msgId;
            *pParam1 = rkpsQ.elem[i].msg.param1;
            *pParam2 = rkpsQ.elem[i].msg.param2;
            rkpsQ.elem[i].state = Q_SENDED;
            allOk = 1;
            break;
        }
    }
    return allOk;
}

static char RkpsQ_Push(char msgId, char param1, char param2)
{
    char allOk = 0;         // some error
    char freeIndex = 0;

    if(rkpsQ.freeSpace)
    {
        if (!RkpsQ_IsHaveMsg(msgId, param1, param2))
        {
            freeIndex = RFS_QUEUE_SIZE - rkpsQ.freeSpace;

            rkpsQ.elem[freeIndex].msg.msgId = msgId;
            rkpsQ.elem[freeIndex].msg.param1 = param1;
            rkpsQ.elem[freeIndex].msg.param2 = param2;
            rkpsQ.elem[freeIndex].state = Q_NOT_SENDED;

            rkpsQ.freeSpace--;
        }
        allOk = 1;
    }
    return allOk;
}

static char RkpsQ_IsHaveMsg(char msgId, char param1, char param2)
{
    char isHave = 0;
    for(char i = 0; i < RFS_QUEUE_SIZE - rkpsQ.freeSpace; i++)
    {
        if((rkpsQ.elem[i].state == Q_NOT_SENDED) &&
            (rkpsQ.elem[i].msg.msgId == msgId) &&
            (rkpsQ.elem[i].msg.param1 == param1) &&
            (rkpsQ.elem[i].msg.param2 == param2))
        {
            isHave = 1;
            break;
        }
    }
    return isHave;
}


static char RkpsQ_Clean(void)
{
    char allOk = 0;

    for(char j = 0; j < 2; j++)
    {
        if(rkpsQ.elem[j].state == Q_SENDED)
        {
            for(char i = 0; i < RFS_QUEUE_SIZE-1; i++)
            {
                memcpy((char *)&rkpsQ.elem[i], (char *)&rkpsQ.elem[i+1], sizeof(rkpsQ.elem[0]));
            }
            memset((char *)&rkpsQ.elem[RFS_QUEUE_SIZE-1], 0, sizeof(rkpsQ.elem[0]));
            rkpsQ.elem[RFS_QUEUE_SIZE-1].state = Q_FREE;
            allOk = 1;
            rkpsQ.freeSpace++;
        }
    }
    return allOk;
}

/**
 * Function restore messages from sended to not_sended.
 * Only for messages with numbers rkpsQ.num1 and rkpsQ.num2
 */
static char RkpsQ_Restore(void)
{
    char allOk = 0;

    rkpsQ.elem[0].state = Q_NOT_SENDED;

    return allOk;
}

static char RkpsQ_IsEmpty(void)
{
    char retval = 0;
    if(rkpsQ.freeSpace == RFS_QUEUE_SIZE)
        retval = 1;

    return retval;
}

static char RkpsQ_HaveSomeMsg(void)
{
    char retval = 0;
    if(rkpsQ.elem[0].state == Q_NOT_SENDED)
    {
        retval = 1;
    }
    return retval;
}

static short TemperatureCalcul(unsigned long adcValue)
{
    unsigned int Rtemp = 0;
    unsigned int mVolt = 0;
    unsigned long Rother = 100000;            // Other resistor value in Ohm
    double Rthermistor = 100000.0;                      // Thermistor value in Ohm with 25C
    double T = 0;
    double T0 = 25.0 + 273.15;
    double B = 4250.0;
    double R_R0 = 0;

    mVolt = 1800 * adcValue / 4096;
    Rtemp = Rother * mVolt / (1800 - mVolt);

    //AUXADCFlushFifo();
    R_R0 = (double)Rtemp / Rthermistor /*100000.0*/;
    T = (T0*B / (T0*log_x(R_R0) + B)) - 273.15;
    return (short) T;
}

static double log_x (double x)
{
    int m = 30; /* (m-длина ряда, целое число) присвойте m значение , чем больше длина ряда,
    тем точнее значение логарифма, но тем дольше идет его расчет */

    double b, c, d, e, f;
    b = (x-1)/(x+1);
    c = b*b;
    d = 1/b;
    f = 0;
    e = -1;
    while (m>0){
        d*=c;
        e+=2;
        f+=(d/e);
        m--;
    }
    f*=2;
    return f;
}
