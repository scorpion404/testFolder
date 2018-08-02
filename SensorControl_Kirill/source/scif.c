/// \addtogroup module_scif_driver_setup
//@{
#include "scif.h"
#include "scif_framework.h"
#undef DEVICE_FAMILY_PATH
#if defined(DEVICE_FAMILY)
    #define DEVICE_FAMILY_PATH(x) <ti/devices/DEVICE_FAMILY/x>
#elif defined(DeviceFamily_CC26X0)
    #define DEVICE_FAMILY_PATH(x) <ti/devices/cc26x0/x>
#elif defined(DeviceFamily_CC26X0R2)
    #define DEVICE_FAMILY_PATH(x) <ti/devices/cc26x0r2/x>
#elif defined(DeviceFamily_CC13X0)
    #define DEVICE_FAMILY_PATH(x) <ti/devices/cc13x0/x>
#else
    #define DEVICE_FAMILY_PATH(x) <x>
#endif
#include DEVICE_FAMILY_PATH(inc/hw_types.h)
#include DEVICE_FAMILY_PATH(inc/hw_memmap.h)
#include DEVICE_FAMILY_PATH(inc/hw_aon_event.h)
#include DEVICE_FAMILY_PATH(inc/hw_aon_rtc.h)
#include DEVICE_FAMILY_PATH(inc/hw_aon_wuc.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_sce.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_smph.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_evctl.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_aiodio.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_timer.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_wuc.h)
#include DEVICE_FAMILY_PATH(inc/hw_event.h)
#include DEVICE_FAMILY_PATH(inc/hw_ints.h)
#include DEVICE_FAMILY_PATH(inc/hw_ioc.h)
#include <string.h>
#if defined(__IAR_SYSTEMS_ICC__)
    #include <intrinsics.h>
#endif


// OSAL function prototypes
uint32_t scifOsalEnterCriticalSection(void);
void scifOsalLeaveCriticalSection(uint32_t key);




/// Firmware image to be uploaded to the AUX RAM
static const uint16_t pAuxRamImage[] = {
    /*0x0000*/ 0x1408, 0x040C, 0x1408, 0x0434, 0x1408, 0x044F, 0x1408, 0x0455, 0x4436, 0x2437, 0xAEFE, 0xADB7, 0x6442, 0x7000, 0x7C75, 0x687C, 
    /*0x0020*/ 0x0070, 0x142D, 0x687D, 0x0071, 0x142D, 0x687E, 0x0072, 0x142D, 0x7875, 0xF801, 0xFA01, 0xBEF2, 0x787A, 0x687C, 0xFD0E, 0x687E, 
    /*0x0040*/ 0xED92, 0xFD06, 0x7C7A, 0x7879, 0xFA01, 0xBE05, 0x7002, 0x7C79, 0x7879, 0xFA00, 0xBEFD, 0x642D, 0x0458, 0x7875, 0x8F1F, 0xED8F, 
    /*0x0060*/ 0xEC01, 0xBE01, 0xADB7, 0x8DB7, 0x6630, 0x6542, 0x0000, 0x187A, 0x9D88, 0x9C01, 0xB60D, 0x106F, 0xAF19, 0xAA00, 0xB609, 0xA8FF, 
    /*0x0080*/ 0xAF39, 0xBE06, 0x0C75, 0x8871, 0x8F08, 0xFD47, 0x9DB7, 0x0875, 0x8801, 0x8A01, 0xBEEC, 0x262F, 0xAEFE, 0x4630, 0x0458, 0x5527, 
    /*0x00A0*/ 0x6642, 0x0000, 0x0C75, 0x140B, 0x0458, 0x6742, 0x03FF, 0x0C77, 0x7876, 0x6877, 0xED37, 0xB605, 0x0000, 0x0C76, 0x7C7B, 0x652D, 
    /*0x00C0*/ 0x0C77, 0x7877, 0x6878, 0xFD0E, 0xF801, 0xE92B, 0xFD0E, 0xBE01, 0x6436, 0xBDB7, 0x241A, 0xA6FE, 0xADB7, 0x641A, 0xADB7, 0x0000, 
    /*0x00E0*/ 0x00CC, 0x00ED, 0x03DD, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x0100*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x0120*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x0140*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x0160*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x0180*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x7002, 0x17DE, 0x640F, 
    /*0x01A0*/ 0x6000, 0x6C98, 0x8607, 0x63D0, 0x6C9D, 0x689D, 0x8600, 0xE8AC, 0x6CA2, 0x689D, 0x86FF, 0xE854, 0x6CA6, 0x689D, 0xE839, 0x6C82, 
    /*0x01C0*/ 0x689D, 0xE8C7, 0x6C83, 0x6032, 0x6CBA, 0x6000, 0x6C92, 0x6000, 0x6C93, 0x0CC9, 0x0001, 0x0C6F, 0xADB7, 0x0000, 0x1001, 0x2002, 
    /*0x01E0*/ 0x3003, 0x489D, 0x8600, 0xC8AC, 0x589D, 0x86FF, 0xD854, 0x6887, 0xEA00, 0xB60F, 0x68C9, 0xED28, 0xBE0B, 0x00FA, 0x0C92, 0x0001, 
    /*0x0200*/ 0x0C93, 0x1CC9, 0x0000, 0x0C9A, 0x0000, 0x0C9B, 0x0080, 0x0C96, 0x0522, 0x68C9, 0xED28, 0xB616, 0x640F, 0x0CC9, 0x0000, 0x0C98, 
    /*0x0220*/ 0x0000, 0x0C99, 0x0000, 0x0C97, 0x0000, 0x0C7F, 0x0000, 0x0CBC, 0x0000, 0x0CC1, 0x0000, 0x0CC3, 0x0000, 0x0CCB, 0x0894, 0x86FF, 
    /*0x0240*/ 0x80FD, 0x0C94, 0x0893, 0x8A00, 0xB608, 0x0892, 0x88FF, 0x0C92, 0x0892, 0x8A00, 0xBE02, 0x0000, 0x0C93, 0x7009, 0x146A, 0xFB4D, 
    /*0x0260*/ 0x8609, 0x7101, 0x6431, 0x2531, 0xA6FE, 0xFB00, 0x7078, 0xFB54, 0x7030, 0xFB4C, 0x7003, 0xFB4C, 0xFD47, 0xFB4C, 0x146D, 0x7002, 
    /*0x0280*/ 0x17DE, 0x6403, 0x001F, 0x8B2C, 0xFDB1, 0x8902, 0x0C8F, 0x7005, 0x17DE, 0x6403, 0x001F, 0x8B2C, 0xFDB1, 0x8902, 0x0C9E, 0x08BA, 
    /*0x02A0*/ 0x8A00, 0xBE0F, 0x0032, 0x0CBA, 0x640E, 0x705E, 0x6007, 0x17EB, 0x7001, 0x17DE, 0x6403, 0x001F, 0x8B2C, 0xFDB1, 0x8902, 0x0CA7, 
    /*0x02C0*/ 0x440E, 0x17F1, 0x08C9, 0x8D29, 0xBE48, 0x0893, 0x8A00, 0xBE07, 0x0885, 0x8C01, 0xBE03, 0x0876, 0x8201, 0x0C76, 0x05AC, 0x0896, 
    /*0x02E0*/ 0x8A00, 0xB613, 0x0896, 0x88FF, 0x0C96, 0x088F, 0x0C80, 0x189B, 0x0880, 0x8D21, 0x0C80, 0x0880, 0x189B, 0x8D29, 0xA603, 0x089A, 
    /*0x0300*/ 0x8801, 0x0C9A, 0x0880, 0x0C9B, 0x05AC, 0x089A, 0x8DA0, 0x8DA1, 0x0C80, 0x189B, 0x9DAF, 0x0880, 0x8D21, 0x0C80, 0x0880, 0x1882, 
    /*0x0320*/ 0x8D29, 0xA614, 0x0880, 0x1883, 0x8D29, 0x9E09, 0x0880, 0x0C9D, 0x440F, 0x2CC9, 0x00FA, 0x0C92, 0x0001, 0x0C93, 0x05A5, 0x0000, 
    /*0x0340*/ 0x0C9A, 0x0000, 0x0C9B, 0x0080, 0x0C96, 0x05AC, 0x0000, 0x0C9A, 0x0000, 0x0C9B, 0x0080, 0x0C96, 0x07BA, 0x08C9, 0x8D2A, 0xBE1B, 
    /*0x0360*/ 0x0893, 0x8A00, 0xBE07, 0x0885, 0x8C02, 0xBE03, 0x0876, 0x8201, 0x0C76, 0x05CA, 0x088F, 0x1882, 0x8D29, 0xA60C, 0x088F, 0x1883, 
    /*0x0380*/ 0x8D29, 0x9E08, 0x0885, 0x8202, 0x0C85, 0x3CC9, 0x0000, 0x0C92, 0x0000, 0x0C93, 0x07BA, 0x08C9, 0x8D2B, 0xB601, 0x07BA, 0x08BC, 
    /*0x03A0*/ 0x8A00, 0xB612, 0x08BB, 0x88FF, 0x0CBB, 0x08BB, 0x8A00, 0xBE0C, 0x0000, 0x0CBC, 0x0894, 0x8C02, 0xB607, 0x0898, 0x88FE, 0x0C98, 
    /*0x03C0*/ 0x0894, 0x86FF, 0x80FD, 0x0C94, 0x08C1, 0x8A00, 0xB610, 0x08C0, 0x88FF, 0x0CC0, 0x08C0, 0x8A00, 0xBE0A, 0x0000, 0x0CC1, 0x0898, 
    /*0x03E0*/ 0x8A00, 0xFE05, 0x0894, 0x8C02, 0xBE02, 0x0000, 0x0C98, 0x08C3, 0x8A00, 0xB60D, 0x08C2, 0x88FF, 0x0CC2, 0x08C2, 0x8A00, 0xBE07, 
    /*0x0400*/ 0x0000, 0x0CC3, 0x0898, 0x8AFF, 0xBE02, 0x0000, 0x0C98, 0x08CB, 0x8A00, 0xB610, 0x08CA, 0x88FF, 0x0CCA, 0x08CA, 0x8A00, 0xBE0A, 
    /*0x0420*/ 0x0000, 0x0CCB, 0x08A6, 0xDD28, 0xB601, 0x5CA6, 0x08A2, 0xCD28, 0xB601, 0x4CA2, 0x08C3, 0x8A00, 0xB60D, 0x08C2, 0x88FF, 0x0CC2, 
    /*0x0440*/ 0x08C2, 0x8A00, 0xBE07, 0x0000, 0x0CC3, 0x0898, 0x8AFF, 0xBE02, 0x0000, 0x0C98, 0x089F, 0x189E, 0x20A8, 0x9F3A, 0x8801, 0x8A10, 
    /*0x0460*/ 0xEE03, 0x0000, 0x1001, 0x1C7F, 0x0C9F, 0x08A8, 0x0CA0, 0x08B7, 0x0CA1, 0x000E, 0x8A00, 0xFE12, 0x18A0, 0x20A8, 0xAF1A, 0x9D2A, 
    /*0x0480*/ 0xA603, 0x10A8, 0x9F19, 0x1CA0, 0x18A1, 0x20A8, 0xAF1A, 0x9D2A, 0x9E03, 0x10A8, 0x9F19, 0x1CA1, 0x88FF, 0x063A, 0x08A0, 0x18A1, 
    /*0x04A0*/ 0x8D19, 0x0CBD, 0x0889, 0x8A00, 0xBE23, 0x087F, 0x8A01, 0xBE20, 0x08BC, 0x8A00, 0xBE1D, 0x08BD, 0x8600, 0x8AC8, 0x9E17, 0x0898, 
    /*0x04C0*/ 0x8A00, 0xF611, 0x0895, 0x8801, 0x0C95, 0x0895, 0x8A1E, 0x960A, 0x0898, 0x8A00, 0xBE07, 0x0190, 0x0CC2, 0x0001, 0x0CC3, 0x0898, 
    /*0x04E0*/ 0x88FF, 0x0C98, 0x0675, 0x0000, 0x0C95, 0x0678, 0x0000, 0x0C95, 0x08A4, 0x0CA5, 0x08A3, 0x0CA4, 0x088F, 0x0CA3, 0x08A3, 0x189D, 
    /*0x0500*/ 0x8D19, 0x0CBE, 0x08A4, 0x189D, 0x8D19, 0x0CBF, 0x08A3, 0x18A4, 0x8D19, 0x0CC7, 0x08C7, 0x8D90, 0x0CC7, 0x08A4, 0x18A5, 0x8D19, 
    /*0x0520*/ 0x0CC8, 0x08C8, 0x8D90, 0x0CC8, 0x08C7, 0x0CC6, 0x08C6, 0x18C8, 0x8D29, 0xE602, 0x08C8, 0x0CC6, 0x08BE, 0x8D90, 0x0CB8, 0x08BF, 
    /*0x0540*/ 0x8D90, 0x0CB9, 0x0897, 0x8A00, 0xB660, 0x08BE, 0x8A00, 0xEE1A, 0x08BF, 0x8A00, 0xE616, 0x08C5, 0x8801, 0x8DA9, 0x8600, 0x8AAC, 
    /*0x0560*/ 0x9E04, 0x189D, 0x9D20, 0x1CA2, 0x06B6, 0x4CA2, 0x0190, 0x0CCA, 0x0001, 0x0CCB, 0x5CA6, 0x0000, 0x0C97, 0x0000, 0x0CC4, 0x0000, 
    /*0x0580*/ 0x0CC5, 0x06DB, 0x08BF, 0x8A00, 0xEE16, 0x08C4, 0x8801, 0x8DA9, 0x8600, 0x8AAC, 0x9E04, 0x189D, 0x9D18, 0x1CA6, 0x06D0, 0x5CA6, 
    /*0x05A0*/ 0x0190, 0x0CCA, 0x0001, 0x0CCB, 0x4CA2, 0x0000, 0x0C97, 0x0000, 0x0CC4, 0x0000, 0x0CC5, 0x08B8, 0x8A39, 0xA627, 0x08C5, 0x18C4, 
    /*0x05C0*/ 0x8D29, 0xAE0D, 0x08C5, 0x8801, 0x8DA9, 0x8600, 0x8AAC, 0x9E04, 0x189D, 0x9D20, 0x1CA2, 0x06ED, 0x4CA2, 0x5CA6, 0x06FB, 0x08C4, 
    /*0x05E0*/ 0x8801, 0x8DA9, 0x8600, 0x8AAC, 0x9E04, 0x189D, 0x9D18, 0x1CA6, 0x06FA, 0x5CA6, 0x4CA2, 0x0190, 0x0CCA, 0x0001, 0x0CCB, 0x0000, 
    /*0x0600*/ 0x0C97, 0x0000, 0x0CC4, 0x0000, 0x0CC5, 0x08B8, 0x8600, 0x8AAC, 0x9E14, 0x08B9, 0x8600, 0x8AAC, 0x9602, 0x0001, 0x0C97, 0x08BE, 
    /*0x0620*/ 0x8A00, 0xEE06, 0x08BF, 0x8A00, 0xE602, 0x0001, 0x0C97, 0x071D, 0x08BF, 0x8A00, 0xEE02, 0x0001, 0x0C97, 0x0000, 0x18BE, 0x9A00, 
    /*0x0640*/ 0xEE07, 0x18BF, 0x9A00, 0xE603, 0x0000, 0x0C99, 0x0001, 0x072E, 0x18BF, 0x9A00, 0xEE03, 0x0000, 0x0C99, 0x0001, 0x8A00, 0xBE74, 
    /*0x0660*/ 0x08B8, 0x8A39, 0xA603, 0x0000, 0x0C99, 0x07A4, 0x08BE, 0x8A00, 0xEE36, 0x0000, 0x18A3, 0x28A2, 0x9D2A, 0x9E05, 0x18A4, 0x28A2, 
    /*0x0680*/ 0x9D2A, 0x9601, 0x0001, 0x8A01, 0xBE1E, 0x08C6, 0x8603, 0x8AC0, 0xE615, 0x0899, 0x8A00, 0xBE12, 0x0894, 0x8C02, 0xBE09, 0x0898, 
    /*0x06A0*/ 0x8801, 0x0C98, 0x0889, 0x8A01, 0xBE03, 0x0876, 0x8201, 0x0C76, 0x0190, 0x0CC0, 0x0001, 0x0CC1, 0x0001, 0x0C99, 0x08B8, 0x0CC4, 
    /*0x06C0*/ 0x0000, 0x0CC5, 0x076E, 0x08B8, 0x18C4, 0x8D29, 0x9E07, 0x0899, 0x8A01, 0xBE04, 0x08B8, 0x0CC4, 0x0000, 0x0CC5, 0x07A4, 0x0000, 
    /*0x06E0*/ 0x18A3, 0x28A6, 0x9D2A, 0xA605, 0x18A4, 0x28A6, 0x9D2A, 0xAE01, 0x0001, 0x8A01, 0xBE1E, 0x08C6, 0x8603, 0x8AC0, 0xE615, 0x0899, 
    /*0x0700*/ 0x8A00, 0xBE12, 0x0894, 0x8C02, 0xBE09, 0x0898, 0x8801, 0x0C98, 0x0889, 0x8A01, 0xBE03, 0x0876, 0x8201, 0x0C76, 0x0190, 0x0CC0, 
    /*0x0720*/ 0x0001, 0x0CC1, 0x0001, 0x0C99, 0x08B8, 0x0CC5, 0x0000, 0x0CC4, 0x07A4, 0x08B8, 0x18C5, 0x8D29, 0x9E07, 0x0899, 0x8A01, 0xBE04, 
    /*0x0740*/ 0x08B8, 0x0CC5, 0x0000, 0x0CC4, 0x0898, 0x8A02, 0xEE13, 0x0894, 0x8C02, 0xBE10, 0x0889, 0x8A00, 0xB603, 0x0001, 0x0CBB, 0x07B5, 
    /*0x0760*/ 0x0876, 0x8201, 0x0C76, 0x007D, 0x0CBB, 0x0001, 0x0CBC, 0x0894, 0x8202, 0x0C94, 0x089D, 0x0C8E, 0x08A2, 0x0C8D, 0x08A6, 0x0C90, 
    /*0x0780*/ 0x089E, 0x0C8C, 0x08BD, 0x0C91, 0x0895, 0x8DA3, 0x0C8A, 0x1895, 0x9DA1, 0x088A, 0x8D21, 0x0C8A, 0x0898, 0x0C8B, 0x0888, 0x8A01, 
    /*0x07A0*/ 0xBE05, 0x0876, 0x8201, 0x0C76, 0x0000, 0x0C86, 0x0001, 0x0C6F, 0x0004, 0x1874, 0x8D01, 0x0C73, 0xADB7, 0xADB7, 0xF007, 0x146A, 
    /*0x07C0*/ 0x86FF, 0x63F8, 0xEB51, 0x8680, 0x6000, 0xED8F, 0xEB49, 0xFD47, 0xEB49, 0x146D, 0xADB7, 0xFB0C, 0xEDA4, 0xEB09, 0x640B, 0xCDB1, 
    /*0x07E0*/ 0xADB7, 0x146A, 0x7079, 0xFB55, 0x71FB, 0xFB54, 0xFD47, 0xFB54, 0x146D, 0x4431, 0x4400, 0xADB7
};


/// Look-up table that converts from AUX I/O index to MCU IOCFG offset
static const uint8_t pAuxIoIndexToMcuIocfgOffsetLut[] = {
    56, 52, 48, 44, 36, 40, 32, 28, 16, 12, 8, 4, 0, 0, 0, 0
};


/** \brief Look-up table of data structure information for each task
  *
  * There is one entry per data structure (\c cfg, \c input, \c output and \c state) per task:
  * - [31:20] Data structure size (number of 16-bit words)
  * - [19:12] Buffer count (when 2+, first data structure is preceded by buffering control variables)
  * - [11:0] Address of the first data structure
  */
static const uint32_t pScifTaskDataStructInfoLut[] = {
//  cfg         input       output      state       
    0x007010FE, 0x0040110C, 0x00801114, 0x03A01124  // AnalyseIK
};




/// Run-time logging signatures (CRC-16) for each data structure for each task
static const uint16_t pRtlTaskStructSignatures[] = {
    0xFDCF, 0x591C, 0x0AAF, 0x01EB
};




// No task-specific initialization functions




// No task-specific uninitialization functions




/** \brief Initilializes task resource hardware dependencies
  *
  * This function is called by the internal driver initialization function, \ref scifInit().
  */
static void scifTaskResourceInit(void) {
    scifInitIo(2, AUXIOMODE_ANALOG, -1, 0);
    scifInitIo(7, AUXIOMODE_ANALOG, -1, 0);
    scifInitIo(1, AUXIOMODE_ANALOG, -1, 0);
    scifInitIo(5, AUXIOMODE_ANALOG, -1, 0);
    scifInitIo(8, AUXIOMODE_OUTPUT, -1, 1);
    scifInitIo(0, AUXIOMODE_OUTPUT, -1, 0);
    scifInitIo(12, AUXIOMODE_OUTPUT, -1, 0);
} // scifTaskResourceInit




/** \brief Uninitilializes task resource hardware dependencies
  *
  * This function is called by the internal driver uninitialization function, \ref scifUninit().
  */
static void scifTaskResourceUninit(void) {
    scifUninitIo(2, -1);
    scifUninitIo(7, -1);
    scifUninitIo(1, -1);
    scifUninitIo(5, -1);
    scifUninitIo(8, -1);
    scifUninitIo(0, -1);
    scifUninitIo(12, -1);
} // scifTaskResourceUninit




/** \brief Re-initializes I/O pins used by the specified tasks
  *
  * It is possible to stop a Sensor Controller task and let the System CPU borrow and operate its I/O
  * pins. For example, the Sensor Controller can operate an SPI interface in one application state while
  * the System CPU with SSI operates the SPI interface in another application state.
  *
  * This function must be called before \ref scifExecuteTasksOnceNbl() or \ref scifStartTasksNbl() if
  * I/O pins belonging to Sensor Controller tasks have been borrowed System CPU peripherals.
  *
  * \param[in]      bvTaskIds
  *     Bit-vector of task IDs for the task I/Os to be re-initialized
  */
void scifReinitTaskIo(uint32_t bvTaskIds) {
    if (bvTaskIds & (1 << SCIF_ANALYSE_IK_TASK_ID)) {
        scifReinitIo(2, -1);
        scifReinitIo(7, -1);
        scifReinitIo(1, -1);
        scifReinitIo(5, -1);
        scifReinitIo(8, -1);
        scifReinitIo(0, -1);
        scifReinitIo(12, -1);
    }
} // scifReinitTaskIo




/// Driver setup data, to be used in the call to \ref scifInit()
const SCIF_DATA_T scifDriverSetup = {
    (volatile SCIF_INT_DATA_T*) 0x400E00EA,
    (volatile SCIF_TASK_CTRL_T*) 0x400E00F4,
    (volatile uint16_t*) 0x400E00DE,
    0x0000,
    sizeof(pAuxRamImage),
    pAuxRamImage,
    pScifTaskDataStructInfoLut,
    pAuxIoIndexToMcuIocfgOffsetLut,
    scifTaskResourceInit,
    scifTaskResourceUninit,
    (volatile uint16_t*) 0x400E00E6,
    (volatile uint16_t*) 0x400E00E8,
    pRtlTaskStructSignatures
};




/** \brief Starts or modifies RTC-based task scheduling tick generation
  *
  * RTC-based tick generation will wake up the Sensor Controller first at the specified value of the RTC
  * and then periodically at the specified interval. The application must call this function after
  * calling \ref scifInit().
  *
  * The application must ensure that:
  * - \a tickStart is not in the past (prefer using \ref scifStartRtcTicksNow() to avoid this)
  * - \a tickPeriod is not too short. RTC ticks will be skipped silently if the Sensor Controller does
  *   not complete its tasks within a single tick interval.
  *
  * \param[in]      tickStart
  *     RTC value when the first tick is generated:
  *     - Bits 31:16 = seconds
  *     - Bits 15:0 = 1/65536 of a second
  * \param[in]      tickPeriod
  *     Interval at which subsequent ticks are generated:
  *     - Bits 31:16 = seconds
  *     - Bits 15:0 = 1/65536 of a second
  */
void scifStartRtcTicks(uint32_t tickStart, uint32_t tickPeriod) {
    HWREG(AON_RTC_BASE + AON_RTC_O_CH2CMP) = tickStart;
    HWREG(AON_RTC_BASE + AON_RTC_O_CH2CMPINC) = tickPeriod;
    HWREG(AON_RTC_BASE + AON_RTC_O_CHCTL) |= AON_RTC_CHCTL_CH2_EN_M | AON_RTC_CHCTL_CH2_CONT_EN_M;
    HWREG(AON_EVENT_BASE + AON_EVENT_O_AUXWUSEL) =
        (HWREG(AON_EVENT_BASE + AON_EVENT_O_AUXWUSEL) & ~AON_EVENT_AUXWUSEL_WU0_EV_M) |
        (AON_EVENT_AUXWUSEL_WU0_EV_RTC_CH2);
} // scifStartRtcTicks




/** \brief Starts or modifies RTC-based task scheduling tick generation
  *
  * RTC-based tick generation will wake up the Sensor Controller first after approximately 128 us and
  * then periodically at the specified interval. The application must call this function after calling
  * \ref scifInit().
  *
  * The application must ensure that \a tickPeriod is not too short. RTC ticks will be skipped silently
  * if the Sensor Controller does not complete its tasks within a single tick interval.
  *
  * \param[in]      tickPeriod
  *     Interval at which subsequent ticks are generated:
  *     - Bits 31:16 = seconds
  *     - Bits 15:0 = 1/65536 of a second
  */
void scifStartRtcTicksNow(uint32_t tickPeriod) {
    uint32_t key, sec, subsec;
    key = scifOsalEnterCriticalSection();
    sec = HWREG(AON_RTC_BASE + AON_RTC_O_SEC);
    subsec = HWREG(AON_RTC_BASE + AON_RTC_O_SUBSEC);
    scifStartRtcTicks(((sec << 16) | (subsec >> 16)) + 8, tickPeriod);
    scifOsalLeaveCriticalSection(key);
} // scifStartRtcTicksNow




/** \brief Stops RTC-based task scheduling tick generation
  *
  * The application must call this function before calling \ref scifUninit().
  */
void scifStopRtcTicks(void) {
    HWREG(AON_RTC_BASE + AON_RTC_O_CHCTL) &= ~(AON_RTC_CHCTL_CH2_EN_M | AON_RTC_CHCTL_CH2_CONT_EN_M);
    HWREG(AON_EVENT_BASE + AON_EVENT_O_AUXWUSEL) =
        (HWREG(AON_EVENT_BASE + AON_EVENT_O_AUXWUSEL) & ~AON_EVENT_AUXWUSEL_WU0_EV_M) |
        (AON_EVENT_AUXWUSEL_WU0_EV_NONE);
    HWREG(AON_RTC_BASE + AON_RTC_O_SYNC);
} // scifStopRtcTicks


//@}


// Generated by ELESTA-25 at 2018-08-01 18:25:13.038