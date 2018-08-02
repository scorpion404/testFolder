/*
 * Rkps.h
 *
 *  Created on: 26 апр. 2018 г.
 *      Author: Vladimir Bortnikov
 */

#ifndef APPLICATION_RKPS_RKPS_H_
#define APPLICATION_RKPS_RKPS_H_

#include <xdc/std.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Mailbox.h>


#include "Libs/RKPS_Lib/RKPS_PacketType.h"
#include "Board.h"

#define FREQ_PROG_CHAN  (19)

#define RFS_QUEUE_SIZE  (10)
#define PACK_LENGTH     (16)

#define RML_ADDRES   /*30*/ /*20*/ /*10*/   /*63*/  6

#define CCFG_SIZE   (4096)/*(256)*/                 /// user config section in the last flash memory sector

#define CONTROL_FRAME_OWN_QUANT (320)

#define NUMMSGS         2                       // число сообщений в буфере mailbox-типа (TI-RTOS), должно хватать и 1, но, на всякий, случай 2
#define RMLSTATE_SIGNAT (0x12345678)            // сигнатура для проверки целостности данных флагов с Flash



#define DEVICE_NAME (0x8000 + 5231)     // Jupiter-5231, MSB=1 - then Jupiter, else Saturn
#define DEVICE_TYPE DEVICETYPE_INFRARED_PASSIVE


#define OWN_TIME_US     (22461)                 // time of one OWN in uS
enum TxPower_Index                              // radio emission power index values
{
    TXPOW_MINUS_10,
    TXPOW_00,
    TXPOW_01,
    TXPOW_02,
    TXPOW_03,
    TXPOW_04,
    TXPOW_05,
    TXPOW_06,
    TXPOW_07,
    TXPOW_08,
    TXPOW_09,
    TXPOW_10,
    TXPOW_QUANT,
};

enum    _RML_inner_systems_modes
{
    MODE_NO_MODE        = 0,
    MODE_PROGRAMM       = (1 << 0),
    MODE_TEST_INDON     = (1 << 1),
    MODE_LINKED         = (1 << 2),
    MODE_PIR_TEST       = (1 << 3),
};

enum    FlagNumbers
{
    RML_FL_ALARM = 0,
    RML_FL_ARMED,
    RML_FL_BLOCK,
    RML_FL_INDON,
    RML_FL_PROGED,
    RML_FL_LOOPPRES,
    RML_FL_LOOP_WAS_RESTORE,
    RML_FL_QUANT
};

enum RfLinkQuality_enum                         // received radio signal power quality
{
    RFLINKQ_BAD             = 15,
    RFLINKQ_ACCEPTABLE      = 20,
    RFLINKQ_GOOD            = 25,
    RFLINKQ_EXCELLENT       = 30,
};

enum rfsRadioTransmitters
{
    RFS_TRANS_1,
    RFS_TRANS_2,
    RFS_TRANS_QUANT,
};

enum rfsRadioFrequencies
{
    RFS_FREQ_1,
    RFS_FREQ_2,
    RFS_FREQ_QUANT,
};

enum freqParts_enum             // for set frequency by command to radio core
{
    FREQ_MAIN_PART,
    FREQ_DECIMAL_PART,
};

enum sensorsThreshold_Quantites
{
    ALARM_SENSOR,
    SABOT_SENSOR,
    LOOP_SENSOR,
    SENSORS_QUANT,
};

enum RKPS_Handlers_Enum
{
    RKPS_LINK_HANDLER,
    RKPS_LISTEN_HANDLER,
    RKPS_MESSAGE_HANDLER,
    RKPS_SYNC_HANDLER,
    RKPS_PROG_HANDLER,
    RKPS_HANDLER_QUANT,
    RKPS_NO_HANDLER,
};


enum    RkpsQueueMessageStates
{
    Q_FREE,                 ///< message field is empty
    Q_NOT_SENDED,           ///< message haven`t sent yet
    Q_SENDED,               ///< message was send
    Q_ACKED,                ///< rml have received the ACK for this message
};


/// all event that could be in RML
enum DeviceEvents
{
    NO_EVENT,

    HALL1_ALARM_EVENT,
    HALL1_RELEASED_EVENT,
    HALL2_ALARM_EVENT,
    HALL2_RELEASED_EVENT,

    TAMPER_ALARM_EVENT,
    TAMPER_RELEASED_EVENT,

    MODE_PRESSED_EVENT,
    MODE_RELEASED_EVENT,

    DEVICE_NORMAL_EVENT,
    DEVICE_ALARM_EVENT,

    LOOP_SHORTCIRCUIT_EVENT,
    LOOP_NORMAL_EVENT,
    LOOP_ALARM_EVENT,
    LOOP_FIRE_EVENT,
    LOOP_BREAK_EVENT,
    LOOP_CUTOFF_EVENT,

    RKPS_MESSSAGE_EVENT,

    DEV_INDON_EVENT,
    DEV_INDOFF_EVENT,

    DEV_SETPARAM_EVENT,

    DEV_LOPPSENSE_ENABLE_EVENT,
    DEV_LOPPSENSE_DISABLE_EVENT,

    DEV_SW_VER_RESP_EVENT,
    DEV_HW_VER_RESP_EVENT,

    BATTERY_LOW_EVENT,
    BATTERY_NORMAL_EVENT,

    DEV_SWITCH_ARMED_EVENT,
    DEV_SWITCH_DISARMED_EVENT,
    DEV_DONTSW_ARMED_EVENT,                 // событие того, что РМЛ отказался переключатсья в режим охраны

    DEV_BLOCKED_EVENT,
    DEV_UNBLOCKED_EVENT,

    QUANTITY_EVENT,
};

/*
 * This type is accessed by the application. When changing the data members of
 * this structure, considerations should be made for padding and data alignment.
 */
typedef struct MsgObj {
    Int     id;
    Char    val;
} MsgObj;

/*
 * Mailbox messages are stored in a queue that requires a header in front of
 * each message. Mailbox_MbxElem is defined such that the header and its size
 * are factored into the total data size requirement for a mailbox instance.
 * Because Mailbox_MbxElem contains Int data types, padding may be added to
 * this struct depending on the data members defined in MsgObj.
 */
typedef struct MailboxMsgObj {
    Mailbox_MbxElem  elem;      /* Mailbox header        */
    MsgObj           obj;       /* Application's mailbox */
} MailboxMsgObj;



#pragma pack(1)

typedef struct
{
    char isPresent : 1;
    char isControl : 1;
} ADD_Loop_Settings_Type;
//--------------------------------------------- RADIO Queue struct types ----------------------------------------

//RKPS message format for rfsQueue (type QUEUE_Type)
typedef struct
{
    //RKPS_Pack_Type RFSPack;
    char msgId;
    char param1;
    char param2;
}QUEUEMESS_Type;

typedef struct _Q_ELEMENT_Type
{
    QUEUEMESS_Type msg;
    char state;         // state isn`t sending, it`s queue element. It`s state of current message in queue
}Q_ELEMENT_Type;


// rfs queue type for RKPS message tx
typedef struct
{
    Q_ELEMENT_Type elem[RFS_QUEUE_SIZE];
    char freeSpace;
}QUEUE_Type;



//--------------------------------------- link quality struct   ------------------------------------
typedef struct                              // Link quality budgets in up-node (RML->RMM) and down-node (RML<-RMM)
{
    char Up;
    char Down;
}LINKQUALITY_Type;

// use it in code
//LINKQUALITY_Type Lq[RFS_TRANS_QUANT][RFS_FREQ_QUANT];           // Link signal budgets for every device in RKPS-System. 2 Frequencies, 2 transmitters in RMM.


//------------------------------------------    RKPS_Type struct    --------------------------------------
typedef struct
{
    RKPS_Pack_Type      txArray;                                    ///< tx array
    RKPS_Pack_Type      rxArray;                                    ///< rx array
    unsigned short      txPowers[TXPOW_QUANT];                      ///< txPowers begin from 10 dBm
    unsigned char       txPowerIndex;                               ///< current txPowerIndex, calculated based on link quality budgetes
    char                noiseCounter;                               ///< counter for noise-field in packet
    unsigned short      curOwnNum;              // current Own number
    unsigned short      msgOwnNum;              // ext message OWN number (free OWN number)
    unsigned long       curOwnTime;             // current Own life time in system tick
    unsigned long       cfTime;                 // time of end of ACK own for LINK own (start CF = this value - 2*OWN_Time)
    char                shiftTimeMinus;
    unsigned long       shiftTime;
    char                haveCmd;
    char                cfCount;                // increment every LINk own
    unsigned short      myOwnNum;               // number of my Own

}RKPS_Type;


typedef struct
{
    char armed  : 1;
    char proged : 1;
    char indon  : 1;
    char alarm  : 1;
    char loopPresent    : 1;
    char blocked        : 1;
    char inFlash        : 1;
    char partInFlash    : 1;
    char loopWasRestore : 1;                // if ==1 then loop was restore in armed state, so no need to send msg about loop normal
}RML_Flag_Type;


typedef struct
{
    char isLineOk   : 1;
    char isGenOn    : 1;                // 1 - out voltage is generating, 0 - no out voltage
    char isOutOn    : 1;                // 1 - out voltage switch to the out,  0 - out voltage don`t switch to the out
}EXEC_DEV_Flags_Type;


typedef struct
{
    char num    : 4;
    char state  : 4;
}SENSOR_Type;


//----------------------------------------  RML_Settings_Type struct    --------------------------------
typedef struct
{
    char                    addr;       /*1*/
    char                    type;       /*2*/
    short                   systemMac;  /*4*/
    char                    freqChanIndexAr[RFS_FREQ_QUANT]; ///< frequency channel numbers array /*4 + RFS_FREQ_QUANT*/
    char                    sensThreshold[2 /*THRESH_QUANT*/][SENSORS_QUANT];                       // 2 levels for each sensor  /*4 + RFS_FREQ_QUANT + 2*SENSORS_QUANT*/
    ADD_Loop_Settings_Type  loop;           /*4 + RFS_FREQ_QUANT + 2*SENSORS_QUANT + 1*/
    unsigned short          name;           // 0xA427 = J2427; 0x2427 = S2427 -> старший бит указывает на J (== 1) или S (== 0) /*4 + RFS_FREQ_QUANT + 2*SENSORS_QUANT + 1 + 2*/
    long                    scode;          // 0x07030094 = 7003094 вторая часть берется из имени /*4 + RFS_FREQ_QUANT + 2*SENSORS_QUANT + 1 + 2 + 4*/
    // < -- delay time for arming. It`s empty now.
    char                    swVer[2];       // 0x01'A' = 0.1A; 0x12'M' = 1.2M           /*4 + RFS_FREQ_QUANT + 2*SENSORS_QUANT + 1 + 2 + 4 + 2 */
    char                    hwVer[2];       // 0x01'A' = 0.1A; 0x12'M' = 1.2M           /*4 + RFS_FREQ_QUANT + 2*SENSORS_QUANT + 1 + 2 + 4 + 2 + 2*/
    char                    aes128Key[16];                              ///< 128 bit AES key    /*4 + RFS_FREQ_QUANT + 2*SENSORS_QUANT + 1 + 2 + 4 + 2 + 16*/
    char reserve0[90 -
                  sizeof(char)*2 -                                      // addr + type
                  sizeof(short) -                                       // MAC
                  sizeof(char)*RFS_FREQ_QUANT -                         // freqChanIndexAr
                  sizeof(char)* 2 /*THRESH_QUANT*/ * SENSORS_QUANT -    //sensThreshold
                  sizeof(ADD_Loop_Settings_Type) -                      // loop
                  sizeof(short) -                                       // name
                  sizeof(long) -                                        // scode
                  sizeof(char)*2*2  -                                   // swVer + hwVer
                  sizeof(char)*16 -                                     // aes128Key
                  sizeof(char)                                          // crc8
                  ];
    char CRC8;                          /*4 + RFS_FREQ_QUANT + 2*SENSORS_QUANT + 1 + 2 + 4 + 2 + 16 + 1 = 38 byte */
}RML_Setting_Type;


//------------------------------------------    RML_State_Type struct   --------------------------------------
typedef struct
{
    RML_Flag_Type       flags;
//    RML_Flag_Type       flashFlags;             // флаги параметров, которые надо сохранить в Flash
    short               curTemper1;
    short               curTemper2;
    char                curRfChanIndex;        // could be 0 or 1, select from RML_Setting_Type.freqChanIndexAr[2]
    LINKQUALITY_Type    Lq[RFS_TRANS_QUANT][RFS_FREQ_QUANT];
    short               batVoltage1;
    short               batVoltage2;
    //----- for exec device ---------------------------
    EXEC_DEV_Flags_Type exeFlags;
    short               outVoltage;
    //-------------------------------------------------
    SENSOR_Type         sens[SENSORS_QUANT];
//    SENSOR_Type         buttons[SENSORS_QUANT];
    char                mode;               // current device modes bitfield (or the last passed handler), bits from _RML_inner_systems_modes
    unsigned long       testIndicTimeStart;
    unsigned char       packErrCount;
    unsigned char       packCount;
    unsigned char       maxPackErrCount;
}RML_State_Type;



typedef struct RML_State_Flags      // for saving to flash
{
    unsigned long signat;           ///< signature for checking data from Flash
    unsigned long armState;         ///< current state of rml: armed / disarmed. Look at RFS_ArmStateRefresh()
    unsigned long progState;
    unsigned long alarmState;
    unsigned long loopPresState;
    unsigned long blockState;
    unsigned long loopRestoreState; ///< flag that loop was in common state when device entered in disarmed state
}RML_State_Flags_Type;



typedef struct
{
    char cmdId;
    void (*CmdHandler)(char Off0_On1);
}CMD_EVENT_Handlers_type;

#pragma pack(4)


//extern MailboxMsgObj mailboxBuffer[NUMMSGS];
//extern Mailbox_Struct mbxStruct;
//extern Mailbox_Handle mbxHandle;


void RKPS_Init(void);
void RKPS_Task(UArg arg0, UArg arg1);

void SC_AlertCallback(void);

void RKPS_Link_Handler      (void);
void RKPS_Listen_handler    (void);
void RKPS_Prog_handler      (void);
void RKPS_Sync_handler      (void);
void RKPS_Message_handler   (void);
void RKPS_Keybutton_handler       (void);

void RKPS_ReadRkpsFromFlash     (void);
void RKPS_ReadSettingsFromFlash (void);
void RKPS_ReadStatesFromFlash   (void);

void RKPS_SaveRkpsToFlash       (void);
void RKPS_SaveSettingsToFlash   (void);
void RKPS_SaveRmlFlagsToFlash       (void);

void RKPS_ButtonClkChangePeriodRestart(unsigned short periodMulti);

#endif /* APPLICATION_RKPS_RKPS_H_ */
