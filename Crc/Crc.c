#include "Crc.h"
/**
*some changes 002
*
*/

//**************************************************************
//	@ bref: calculate CRC16 for Array. 
//	@ param: uint8_t Array[] - input array for calculation
//			uint16_t arraysize - size Array in bytes
//	@ retval: CRC16
//	@ comment:
//***************************************************************
uint16_t Crc_16(uint8_t Array[], uint16_t arraysize)
{
    unsigned short  byteN = 0,
                    bitN = 0,
                    crc =   /*0x1D0F,*/ /*0x0,*/  0xffff,
                    xorOut = 0, //0xffff,
                    poly = /*0x3D65;*/  0x1021;//0xC867;  //0x0589
    unsigned char   byteV = 0,
                    msBit = 0,
                    addBit = 0;

    for(byteN = 0; byteN < arraysize + sizeof(crc); byteN++)    // байты с младшего
//    for(byteStep = arraysize+1; byteStep > 0; byteStep--)           // байты со старшего + 2 нулевых в хвосте
    {
        if(byteN < arraysize)
            byteV = Array[byteN];
          //if(byteStep > 1)
            //byteV = Array[byteStep-2];
        else
            byteV = 0;
        for(bitN = 0; bitN < 8; bitN++)
        {
            if(crc & 0x8000)
                msBit = 1;
            crc = crc << 1;
            //if(byteV & (1 << bitN))         // биты с младшего внутри байта
            if(byteV & (0x80 >> bitN))             // биты со старшего внутри байта
                addBit = 1;
            crc += addBit;
            if(msBit)
                crc ^= poly;
            msBit = addBit = 0;
        }
    }
    return (crc ^ xorOut);
//    unsigned short byteNum = 0;
//    unsigned char b = 0;
//    const unsigned short generator = 0x1021;     /* divisor is 16bit */
//    unsigned short crc = 0xffff;                 /* CRC value is 16bit */
//    int i = 0;
//    for (byteNum = 0; byteNum < arraysize; byteNum++)
//    {
//       b = Array[byteNum];
//       crc ^= (unsigned short)(b << 8); /* move byte into MSB of 16bit CRC */
//
//       for (i = 0; i < 8; i++)
//       {
//           if ((crc & 0x8000) != 0) /* test for MSB = bit 15 */
//           {
//               crc = (unsigned short)((crc << 1) ^ generator);
//           }
//           else
//           {
//               crc <<= 1;
//           }
//       }
//    }
//    return crc;
}


//**************************************************************
//	@ bref: calculate CRC32 for Array. 
//	@ param: uint8_t Array[] - input array for calculation
//			uint16_t arraysize - size Array in bytes
//	@ retval: CRC32
//	@ comment:
//***************************************************************
uint32_t Crc_32(uint8_t Array[], uint16_t arraysize)
{
    unsigned short byteNum = 0;
   unsigned char b = 0;
   const unsigned long generator = 0x04C11DB7;     /* divisor is 32bit */
   unsigned long crc = 0xffffffff;                 /* CRC value is 32bit */
   unsigned long i = 0;

   for (byteNum = 0; byteNum < arraysize; byteNum++)
   {
      b = Array[byteNum];
      crc ^= (unsigned long)(b << 24/*8*/); /* move byte into MSB of 32bit CRC */

      for (i = 0; i < 8; i++)
      {
          if ((crc & 0x80000000) != 0) /* test for MSB = bit 31 */
          {
              crc = (unsigned long)((crc << 1) ^ generator);
          }
          else
          {
              crc <<= 1;
          }
      }
   }
   return crc;
}

uint8_t Crc_8(uint8_t Array[], uint16_t arraysize)
{
    unsigned short byteNum = 0;
    unsigned char b = 0;
    const unsigned char generator = 0x07;     /* divisor is 16bit */
    unsigned char crc = 0x00;                 /* CRC value is 16bit */
    int i = 0;
    for (byteNum = 0; byteNum < arraysize; byteNum++)
    {
       b = Array[byteNum];
       crc ^= (unsigned char)(b /*<< 8*/); /* move byte into MSB of 16bit CRC */

       for (i = 0; i < 8; i++)
       {
           if ((crc & 0x80) != 0) /* test for MSB = bit 7 */
           {
               crc = (unsigned char)((crc << 1) ^ generator);
           }
           else
           {
               crc <<= 1;
           }
       }
    }
    return crc;
}
