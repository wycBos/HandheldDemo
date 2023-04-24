/**************************************************************************
 * The AD-DA converter routines.It comes from ADS1x15.c file, which is created
 * for ads131m04 ADC, mcp4822 DAC, and ADS1x15 chips.
 * The pigpio ISP & I2C routines support the read/write with such chips.
 *************************
 *
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <pigpio.h>
#include "ADS1x15.h"

float _GAIN[] =
    {6.144, 4.096, 2.048, 1.024, 0.512, 0.256, 0.256, 0.256};

const char *_CHAN[] =
    {"A0-A1", "A0-A3", "A1-A3", "A2-A3", "A0", "A1", "A2", "A3"};

int _SPS_1115[] = {8, 16, 32, 64, 128, 250, 475, 860};

ads1x15_p adc;
/* basic routines */
// int _read_config(ads1x15_p s);
// int _write_config(ads1x15_p s);
// int _write_comparator_thresholds(ads1x15_p s, int high, int low);
// int _update_comparators(ads1x15_p s);
// int _update_config(ads1x15_p s);

/* routines for I2C communications */
int _read_config(ads1x15_p s)
{
   unsigned char buf[8];

   buf[0] = CONFIG_REG;

   // lgI2cWriteDevice(s->i2ch, buf, 1);  // set config register
   i2cWriteDevice(s->i2ch, buf, 1);

   // lgI2cReadDevice(s->i2ch, buf, 2);
   i2cReadDevice(s->i2ch, buf, 2);

   s->configH = buf[0];
   s->configL = buf[1];

   buf[0] = COMPARE_LOW_REG;

   // lgI2cWriteDevice(s->i2ch, buf, 1); // set low compare register
   i2cWriteDevice(s->i2ch, buf, 1);

   // lgI2cReadDevice(s->i2ch, buf, 2);
   i2cReadDevice(s->i2ch, buf, 2);

   s->compare_low = (buf[0] << 8) | buf[1];

   buf[0] = COMPARE_HIGH_REG;

   // lgI2cWriteDevice(s->i2ch, buf, 1); // set high compare register
   i2cWriteDevice(s->i2ch, buf, 1);

   // lgI2cReadDevice(s->i2ch, buf, 2);
   i2cReadDevice(s->i2ch, buf, 2);

   s->compare_high = (buf[0] << 8) | buf[1];

   buf[0] = CONVERSION_REG;

   // lgI2cWriteDevice(s->i2ch, buf, 1); // set conversion register
   i2cWriteDevice(s->i2ch, buf, 1);

   s->channel = (s->configH >> 4) & 7;
   s->gain = (s->configH >> 1) & 7;
   s->voltage_range = _GAIN[s->gain];
   s->single_shot = s->configH & 1;
   s->sps = (s->configL >> 5) & 7;
   s->comparator_mode = (s->configL >> 4) & 1;
   s->comparator_polarity = (s->configL >> 3) & 1;
   s->comparator_latch = (s->configL >> 2) & 1;
   s->comparator_queue = s->configL & 3;

   if (s->comparator_queue != 3)
      s->set_queue = s->comparator_queue;
   else
      s->set_queue = 0;

   return 0;
}

int _write_config(ads1x15_p s)
{
   unsigned char buf[8];

   buf[0] = CONFIG_REG;
   buf[1] = s->configH;
   buf[2] = s->configL;

   // lgI2cWriteDevice(s->i2ch, buf, 3);
   i2cWriteDevice(s->i2ch, buf, 3);

   buf[0] = CONVERSION_REG;

   // lgI2cWriteDevice(s->i2ch, buf, 1);
   i2cWriteDevice(s->i2ch, buf, 1);

   return 0;
}

int _write_comparator_thresholds(ads1x15_p s, int high, int low)
{
   unsigned char buf[8];

   if (high > 32767)
      high = 32767;
   else if (high < -32768)
      high = -32768;

   if (low > 32767)
      low = 32767;
   else if (low < -32768)
      low = -32768;

   s->compare_high = high;
   s->compare_low = low;

   buf[0] = COMPARE_LOW_REG;
   buf[1] = (low >> 8) & 0xff;
   buf[2] = low & 0xff;

   // lgI2cWriteDevice(s->i2ch, buf, 3);
   i2cWriteDevice(s->i2ch, buf, 3);

   buf[0] = COMPARE_HIGH_REG;
   buf[1] = (high >> 8) & 0xff;
   buf[2] = high & 0xff;

   // lgI2cWriteDevice(s->i2ch, buf, 3);
   i2cWriteDevice(s->i2ch, buf, 3);

   buf[0] = CONVERSION_REG;

   // lgI2cWriteDevice(s->i2ch, buf, 1);
   i2cWriteDevice(s->i2ch, buf, 1);

   return 0;
}

int _update_comparators(ads1x15_p s)
{
   int h, l;

   if (s->alert_rdy >= ADS1X15_ALERT_TRADITIONAL)
   {
      h = s->vhigh * 32768.0 / s->voltage_range;
      l = s->vlow * 32768.0 / s->voltage_range;

      return _write_comparator_thresholds(s, h, l);
   }

   return 0;
}

int _update_config(ads1x15_p s)
{
   int H, L;

   H = s->configH;
   L = s->configL;

   s->configH = ((1 << 7) | (s->channel << 4) |
                 (s->gain << 1) | s->single_shot);

   s->configL = ((s->sps << 5) | (s->comparator_mode << 4) |
                 (s->comparator_polarity << 3) | (s->comparator_latch << 2) |
                 s->comparator_queue);

   if ((H != s->configH) || (L != s->configL))
      _write_config(s);

   return 0;
}

/* routines for ADS1x15 opertions */
int ADS1X15_set_channel(ads1x15_p s, int channel)
{
   if (channel < 0)
      channel = 0;
   else if (channel > 7)
      channel = 7;

   s->channel = channel;

   _update_config(s);

   return channel;
}

float ADS1X15_set_voltage_range(ads1x15_p s, float vrange)
{
   int val, i;

   val = 7;

   for (i = 0; i < 8; i++)
   {
      if (vrange > _GAIN[i])
      {
         val = i;
         break;
      }
   }

   if (val > 0)
      val = val - 1;

   s->gain = val;

   s->voltage_range = _GAIN[val];

   _update_comparators(s);

   _update_config(s);

   return s->voltage_range;
}

int ADS1X15_set_sample_rate(ads1x15_p s, int rate)
{
   int val, i;

   val = 7;

   for (i = 0; i < 8; i++)
   {
      if (rate <= s->SPS[i])
      {
         val = i;
         break;
      }
   }

   s->sps = val;
   _update_config(s);

   return s->SPS[val];
}

ads1x15_p ADS1X15_open(int sbc, int bus, int device, int flags)
{
   ads1x15_p s;

   s = calloc(1, sizeof(ads1x15_t));

   if (s == NULL)
      return NULL;

   s->sbc = sbc;       // sbc connection
   s->bus = bus;       // I2C bus
   s->device = device; // I2C device
   s->flags = flags;   // I2C flags

   s->SPS = _SPS_1115; // default

   // s->i2ch = lgI2cOpen(bus, device, flags);
   s->i2ch = i2cOpen(bus, device, flags);

   if (s->i2ch < 0)
   {
      free(s);
      return NULL;
   }

   _read_config(s);

   // ADS1X15_alert_never(s); // switch off ALERT/RDY pin.

   return s;
}

int ADS1X15_read(ads1x15_p s)
{
   unsigned char buf[8];

   if (s->single_shot)
      _write_config(s);

   // lgI2cReadDevice(s->i2ch, buf, 2);
   i2cReadDevice(s->i2ch, buf, 2);

   return (buf[0] << 8) + buf[1];
}

float ADS1X15_read_voltage(ads1x15_p s)
{
   return ADS1X15_read(s) * s->voltage_range / 32768.0;
}

ads1x15_p ADS1115_open(int sbc, int bus, int device, int flags)
{
   ads1x15_p s;

   s = ADS1X15_open(sbc, bus, device, flags);

   if (s)
      s->SPS = _SPS_1115;

   return s;
}

ads1x15_p ADS1X15_close(ads1x15_p s)
{
   if (s != NULL)
   {
      // lgI2cClose(s->i2ch);
      i2cClose(s->i2ch);
      free(s);
      s = NULL;
   }
   return s;
}

/* int main(int argc, char *argv[]) */

float ADS1115_main(void)
{
   int h;
   int err;
   int cb_id;
   ads1x15_p adc = NULL;
   // double end_time;
   int end_time, seconds;
   int micros;

   int g, fd, wid = -1;

   float ADvolt = 0;
#if 1
   if (1 /*|!gpio_initlized*/) // pigpio re-init
   {
      // printf("init pre-. \n");
      if (gpioInitialise() < 0)
      {
         // printf("init error. \n");
         return 1;
      }
   }
   // printf("pigpio initialized. \n");

   adc = ADS1115_open(0, 1, 0x48, 0);
   if (adc == NULL)
   {
      printf("ADS closed. \n");
      return -2;
   }
   // printf("ADS1115 start. \n");
   ADS1X15_set_channel(adc, ADS1X15_A0);
   ADS1X15_set_voltage_range(adc, 3.3);
   ADS1X15_set_sample_rate(adc, 0); // set minimum sampling rate

   if (0 /*(ALERT >= 0*/) /* ALERT pin connected */
   {
      // h = lgGpiochipOpen(0);

      // if (h <0) return -3;

      /* got a handle, now open the GPIO for alerts */
      // err = lgGpioClaimAlert(h, 0, LG_BOTH_EDGES, ALERT, -1);
      // if (err < 0) return -4;
      // lgGpioSetAlertsFunc(h, ALERT, cbf, adc);
   }
#endif
   // printf("ADS1115 read. \n");
   ADvolt = ADS1X15_read_voltage(adc);

   // printf("re-%.2f\n", ADvolt /*ADS1X15_read_voltage(adc)*/);

   ADS1X15_close(adc);
   return ADvolt;
}

/****************************************************************
 * The code above supports the ADS1115 ADC operations via I2C bus.
 *
 * ************************************************************
 *
 * */

/***********************************************************
 * The routines are following supprt the MCP4822 and ADS131m04.
 * The used bus is SPI supported by pigpio.
 * 
 * *********************************************************
 * 
 */
 
/* TODO - add routines for MCP4822 and ADS131 */

/* The routines for MCP4822 */
/*
***************************************************************************
*  The SPI periphearl for MCP4822 DAC                                     *
*  \fn void tspi_mcp4822(int channel, int command, double value)          *
*                                                                         *
*  \param channel is the MCP4822 DAC channel ID 0-A, 1-B                  *
*         command indicates the operation to do:                          *
*                       0-disable output,                                 *
*                       1-enable the output,                              *
*                       2-load the new value                              *
*         value is the output value in volt.                              *
*                                                                         *
*  \return None                                                           *
*                                                                         *
***************************************************************************
*/

void tspi_mcp4822(int channel, int command, double valu) // SPI channel test MCP4822
{
   /**********************************
    *  channel: 0-DAC_A; 1-DAC_B
    *  command: 0-DIS_ACT, 1-EN_ACT, 2-LDAC
    *  valu: 0 - 2
    *************************************/

   int h, b, e;
   char txBuf[8];

   printf("    MCP4822 Settings. %d %d %f\n", channel, command, valu);

   /* set the DAC_LDAC command 2 DAC_LDAC is high, otherwise it is low*/
   gpioSetMode(DAC_LDAC, PI_OUTPUT); //TODO - move to main()
   
   //if (command == 2)
   //   gpioWrite(DAC_LDAC, 1);
   //else
   //   gpioWrite(DAC_LDAC, 0);

   /* this test requires a MCP4822 on SPI channel 1 */
   /*
   *******************************************************************
   * the spiopen() has three parameters: spiChan, baud, and spiFlags *
   *   spiChan - two channels for main SPI, 0(gpio8) & 1(gpio7)      *
   *   baud - SPI speed, 1,250,000 Hz                                *
   *   spiFlags - the SPI module settings.                           *
   *   -------------------------------------------------------       *
   *   |21 |20 |19 |18 |17 |16 |15 |14 |13 |12 |11 |10 |9 |8 |       *
   *   |---|---|---|---|---|---|---|---|---|---|---|---|--|--|       *
   *   |b  |b  |b  |b  |b  |b  |R  |T  |n  |n  |n  |n  |W |A |       *
   *   |------------------------------------------------------       *
   *   |7  |6  |5  |4  |3  |2  |1  |0  |                             *
   *   |---|---|---|---|---|---|---|---|                             *
   *   |u2 |u1 |u0 |p2 |p1 |p0 |m  |m  |                             *
   *   ---------------------------------                             *
   *    A - 0 for main SPI, 1 for auciliart SPI                      *
   *    W - 0 the device is not 3-wire, 1 the device is 3-wire.      *
   *     e.g set to 0.                                               *
   *******************************************************************
   *
   */

   h = spiOpen(1, 1250000, 0); // open SPI decice "/dev/spidev0.1" with mode 0 for MCP4822
   //CHECK(12, 1, h, 4, 100, "spiOpenDAC");
   printf("    DAC - %d\n", h);

   /* set SPI device command e.g MCP4822 */
   /*
    *
    *******************************************************************
    * The commands send to SPI:                                       *
    *    channel: CHAN0 - 0, CHAN1 - 1                                *
    *    gain: GAIN2 - 0(2x), GAIN1 - 1(1x)
    *    outOn: OUT_ON - 1, OUT_OFF - 0
    *                                                                 *
    *    valuSet=voltSet/2.048*4096 ; referrence 2.048V & 12-bit      *
    *                                                                 *
    *******************************************************************
    *
    */
   unsigned int numSteps, remainVal, valuSet, ctrlData, setValue;
   char byte0, byte1;

   /* set MCP4822 value */
   if (valu < 0 && valu > 2.048)
      valu = 1.0;

   valuSet = (unsigned int)(valu / 2.048 * 4096);
   /* setting DAC_A value from 0 to set value with increacing 10 digital number per 20ms */
   numSteps = valuSet/10; remainVal = valuSet%10;

   /* set MCP4822 control bits */
   ctrlData = MCP4822_GA1;
   if (channel == 1)
      ctrlData |= MCP4822_DAB; // set DA-B
   else if(channel == 0)
      ctrlData &= (~MCP4822_DAB); // set DA-A

   if (command == 0)
      ctrlData &= (~MCP4822_ACT); // no DA activite
   else if (command == 1 || command == 2)
      ctrlData |= MCP4822_ACT; // set DA activite

   /* set DAC value */
   if(channel == 0) //DAC_A
   {
      printf("    MCP4822 CH_A steps. %d\n", numSteps);
      for(int n = 0; n < numSteps; n++)
      {
         setValue = ctrlData + n*10;
         //printf("    MCP4822 Data. %d\n", setValue);

         byte0 = setValue & 0xFF;
         byte1 = (setValue >> 8) & 0xFF;

         txBuf[1] = byte0;
         txBuf[0] = byte1;
         // sprintf(txBuf, "\x01\x80");
         //printf("MCP4822 Data. %x %x %x\n", setValue, txBuf[1], txBuf[0]);

         /* write data to SPI */
         b = spiWrite(h, txBuf, 2);
         //CHECK(12, 2, b, 2, 0, "spiWrie");

         /* latch data to DAC */
         gpioWrite(DAC_LDAC, 0);
         gpioDelay(400);
         gpioWrite(DAC_LDAC, 1);
         gpioDelay(20000); // delay 20ms
      }
   }
//   else{ //DAC_B
//
//   }
   setValue = ctrlData + valuSet;
   printf("    MCP4822 Data. 0x%x\n", setValue);

   byte0 = setValue & 0xFF;
   byte1 = (setValue >> 8) & 0xFF;

   txBuf[1] = byte0;
   txBuf[0] = byte1;
   // sprintf(txBuf, "\x01\x80");
   //printf("MCP4822 Data. %x %x %x\n", setValue, txBuf[1], txBuf[0]);

   /* write data to SPI */
   b = spiWrite(h, txBuf, 2);
   //CHECK(12, 2, b, 2, 0, "spiWrie");

   /* latch data to DAC */
   gpioWrite(DAC_LDAC, 0);
   gpioDelay(400);
   gpioWrite(DAC_LDAC, 1);

   /*
      for (x=0; x<5; x++)
      {
         b = spiXfer(h, txBuf, rxBuf, 3);
         CHECK(12, 2, b, 3, 0, "spiXfer");
         if (b == 3)
         {
            time_sleep(1.0);
            printf("%d ", ((rxBuf[1]&0x0F)*256)|rxBuf[2]);
         }
      }
   */

   e = spiClose(h);
   //CHECK(12, 3, e, 0, 0, "spiClose");
}

/* The routines for ADS131M04 */
/*
***************************************************************************
*  The routines for ADS131M04 ADC operations                   *
*  \fn void tspi_ads131m04_init(int channel, int command, double value)   *
*                                                                         *
*  \param channel is the ADS131m94 channel ID 0, 1, 2, 3                  *
*                                                                         *
*  \return None                                                           *
*                                                                         *
***************************************************************************
*/

adcRslts adcRltData[ADCLNTH];
adc_channel_data adcData; // TODO double check it
userData adcCapFuncData;

/********************** prototype ****************/
void set_DAC() //TOTO - set MCP4822 output value
{
   return;
}

void init_ADC() //TOTO - init/config ADS131
{
   return;
}

void ADC_capture() //TOTO - run ADS131 value input, event handler
{
   return;
}
/***************** end of prototype *************/

/* helper functions */
char upperByte(uint16_t uint16_Word)
{
   char msByte;
   msByte = (char)((uint16_Word >> 8) & 0x00FF);

   return msByte;
}

char lowerByte(uint16_t uint16_Word)
{
   char lsByte;
   lsByte = (char)(uint16_Word & 0x00FF);

   return lsByte;
}

/*
*********************************************************************************************************
*                                                                                                       *
* Builds SPI TX data arrays to be tranferred.                                                           *
*                                                                                                       *
* \fn uint8_t setSPIarray(const uint16_t opcodeArray[], uint8_t numberOpcodes, uint8_t byteArray[])   *
*                                                                                                       *
* \param opcodeArray[] pointer to an array of 16-bit opcodes to use in the SPI command.                 *
* \param numberOpcodes the number of opcodes provided in opcodeArray[].                                 *
* \param byteArray[] pointer to an array of 8-bit SPI bytes to send to the device.                      *
*                                                                                                       *
* NOTE: The calling function must ensure it reserves sufficient memory for byteArray[]!                 *
*                                                                                                       *
* \return number of bytes added to byteArray[].                                                         *
*                                                                                                       *
*********************************************************************************************************
*
*/
uint8_t setSPIarray(const uint16_t opcodeArray[], uint8_t numberOpcodes, char byteArray[])
{
   /*
    * Frame size = opcode word(s) + optional CRC word
    * Number of bytes per word = 2, 3, or 4
    * Total bytes = bytes per word * number of words
    */
   uint8_t numberWords = numberOpcodes; // as SPI CRC disabled
   uint8_t bytesPerWord = 3;            // as 24-bit per word, getWordByteLength();
   uint8_t numberOfBytes = numberWords * bytesPerWord;

   int i;
   for (i = 0; i < numberWords; i++)
   {
      // NOTE: Be careful not to accidentally overflow the array here.
      // The array and opcodes are defined in the calling function, so
      // we are trusting that no mistakes were made in the calling function!
      byteArray[(i * bytesPerWord) + 0] = upperByte(opcodeArray[i]);
      byteArray[(i * bytesPerWord) + 1] = lowerByte(opcodeArray[i]);
      byteArray[(i * bytesPerWord) + 2] = 0; // lowerByte(opcodeArray[i]);
   }

   // set rest of the byteArray to 0
   for (i = numberOfBytes; i < 18; i++)
   {
      byteArray[i] = 0;
   }
   // #ifdef ENABLE_CRC_IN
   //  Calculate CRC and put it into TX array
   //    uint16_t crcWord = calculateCRC(&byteArray[0], numberOfBytes, 0xFFFF);
   //    byteArray[(i*bytesPerWord) + 0] = upperByte(crcWord);
   //    byteArray[(i*bytesPerWord) + 1] = lowerByte(crcWord);
   // #endif

   return numberOfBytes;
}

uint32_t combineBytes(const char dataBytes[], int numOfbyte)
{
   uint32_t combinedValue;

   if (numOfbyte == 2)
      combinedValue = ((uint32_t)dataBytes[0] << 8) | ((uint32_t)dataBytes[1]);
   if (numOfbyte == 3)
   {
      combinedValue = ((uint32_t)dataBytes[0] << 16) | ((uint32_t)dataBytes[1] << 8) | ((uint32_t)dataBytes[2]);
   }
   return combinedValue;
}

/*
****************************************************************************
*                                                                          *
* Sends the specified SPI command to the ADC (NULL, STANDBY, or WAKEUP).   *
*                                                                          *
*                                                                          *
* \param opcode SPI command byte.                                          *
*                                                                          *
* NOTE: Other commands have their own dedicated functions to support       *
* additional functionality.                                                *
*                                                                          *
* \return ADC response byte (typically the STATUS byte).                   *
*                                                                          *
****************************************************************************
*/
//TODO move into gpio_proc.c
char txBuf[32] = {0};
char rxBuf[32] = {0};

uint16_t sendCommand(int SPIHandle, uint16_t opcode, regInfor *regData, adc_channel_data *DataStruct)
{
   int ret; // h, x, b, e;
   /* Assert if this function is used to send any of the following opcodes */
   // assert(OPCODE_RREG != opcode);      /* Use "readSingleRegister()"   */
   // assert(OPCODE_WREG != opcode);      /* Use "writeSingleRegister()"  */
   // assert(OPCODE_LOCK != opcode);      /* Use "lockRegisters()"        */
   // assert(OPCODE_UNLOCK != opcode);    /* Use "unlockRegisters()"      */
   // assert(OPCODE_RESET != opcode);     /* Use "resetDevice()"          */

   // Build TX and RX byte array

   char *ptxBuf = &txBuf[0]; //[32] = {0};
   char *prxBuf = &rxBuf[0]; //[32] = {0};

   uint8_t numberOfBytes;
   uint16_t lopcode[2] = {0};
   uint16_t lnumRegs = regData->numRegs;

   lopcode[0] = opcode;

   /* prepare SPI command package */
   if (OPCODE_RREG == lopcode[0] || OPCODE_NULL == lopcode[0]) // if it's read Reg or NULL.
   {
      lopcode[0] |= regData->regAddr << 7;
      lopcode[0] += lnumRegs;
      numberOfBytes = setSPIarray(&lopcode[0], 1, ptxBuf);
   }
   else if (OPCODE_WREG == lopcode[0]) // if it's write Reg.
   {
      lopcode[0] |= regData->regAddr << 7;
      lopcode[0] += lnumRegs;
      lopcode[1] = regData->setData;
      numberOfBytes = setSPIarray(&lopcode[0], 2, ptxBuf);
   }
   // printf("command: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x.\n", lopcode[0], lopcode[1], *(ptxBuf+0), *(ptxBuf+1), *(ptxBuf+2));

   // check the txBuf

   //printf("\ntxBuf0 numberOfBytes %d: 0x%x, 0x%x, 0x%x, 0x%x; \n", numberOfBytes, txBuf[0], txBuf[1], txBuf[2], txBuf[3]);

   //printf("txBuf1: 0x%x, 0x%x, 0x%x, 0x%x; \n", txBuf[4], txBuf[5], txBuf[6], txBuf[7]);

   // Send the opcode (and crc word, if enabled)
   numberOfBytes = 18; // 3bytes*6words.
   ret = spiXfer(SPIHandle, ptxBuf, prxBuf, numberOfBytes);

   // check the rxBuf
   // printf("rxBuf0: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x; \n", rxBuf[0], rxBuf[1], rxBuf[3], rxBuf[4], rxBuf[5]);

   // printf("rxBuf1: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x; \n", rxBuf[6], rxBuf[7], rxBuf[8], rxBuf[9], rxBuf[10], rxBuf[11]);

   // retrive data from the rxbuf[]
   DataStruct->response = (uint16_t)combineBytes(&rxBuf[0], 2);
   DataStruct->channel0 = combineBytes(&rxBuf[3], 3);
   DataStruct->channel1 = combineBytes(&rxBuf[6], 3);
   DataStruct->channel2 = combineBytes(&rxBuf[9], 3);
   DataStruct->channel3 = combineBytes(&rxBuf[12], 3);
   DataStruct->crc = (uint16_t)combineBytes(&rxBuf[15], 2);

   return ret; //DataStruct->response;
}

uint16_t tspi_ads131m04_rd(int SPIhandler, regInfor *getInf)
{
   uint16_t ret = 0, rsp;
   regInfor *lpgetInf = getInf;
   adc_channel_data *lpadcData = &adcData;  //TODO - move into arguments of the function

   /* reead ads131m04 register */
   // regSetInf.regAddr = getInf->regAddr;
   // regSetInf.setData = 0;
   ret = sendCommand(SPIhandler, OPCODE_RREG, lpgetInf, lpadcData);
   rsp = lpadcData->response;
   
   //printf("read Reg. addr - 0x%x, data - 0x%x. \n", getInf->regAddr, rsp);
   return rsp;
}

uint16_t tspi_ads131m04_wt(int SPIhandler, regInfor *setInf)
{
   uint16_t ret = 0, rsp;
   /* write ads131m04 register */
   regInfor *lpgetInf = setInf;
   adc_channel_data *lpadcData = &adcData;  //TODO - move into arguments of function

   /* reead ads131m04 register */
   // regSetInf.regAddr = setInf->regAddr;
   // regSetInf.setData = setInf->setData;
   ret = sendCommand(SPIhandler, OPCODE_WREG, lpgetInf, lpadcData);
   rsp = lpadcData->response;
   
   printf("write Reg. addr - 0x%x, data - 0x%x, rsp - 0x%x\n", setInf->regAddr, setInf->setData, rsp);
   return rsp;
}

/*************************************************
 * ads131m04 event function.
 * 
 * ***********************************************
 * 
 * */
regInfor regSetInf; //TODO - move to the definition part of this file.

void adcCaptureFun(int gpio, int level, uint32_t tick, userData* padcCapFuncData) //a callback function for capture adc data.
{
   int reps, h, isRn, idx;
   uint32_t lpreTick;
   regInfor *pregInf = &regSetInf;  //TODO - move into uaserData
   
   lpreTick = padcCapFuncData->preTick;
   h = padcCapFuncData->handle;
   isRn = padcCapFuncData->isRun;
   idx = (padcCapFuncData->datIdx);

   if((level == 0) && (isRn == 1) && (tick > (lpreTick + SAMPRAT)))
   {
         pregInf->regAddr = 1;
         pregInf->numRegs = 0; //numRegs;
         reps = tspi_ads131m04_rd(h, pregInf);
         /*
         double step = 1200000.0 / 8388607.0;
         double v1, v2, v3, v4;

         if(adcData.channel0 > 0x7fffff)
         {
            v1 = (double)(~(adcData.channel0 | 0xff000000)+1);
            v1 = -v1;
         }
         else
         {
            v1 = (double)adcData.channel0;
         }

         if(adcData.channel1 > 0x7fffff)
         {
            v2 = (double)(~(adcData.channel1 | 0xff000000)+1);
            v2 = -v2;
         }
         else
         {
            v2 = (double)adcData.channel1;
         }
         
         if(adcData.channel2 > 0x7fffff)
         {
            v3 = (double)(~(adcData.channel2 | 0xff000000)+1);
            v3 = -v3;
         }
         else
         {
            v3 = (double)adcData.channel2;
         }
         
         if(adcData.channel3 > 0x7fffff)
         {
            v4 = (double)(~(adcData.channel3 | 0xff000000)+1);
            v4 = -v4;
         }
         else
         {
            v4 = (double)adcData.channel3;
         }
         */
         /* updata the adcCapFuncData */
         idx = (idx + 1)%ADCLNTH;
         padcCapFuncData->datIdx = idx;
         padcCapFuncData->preTick = tick;
         (padcCapFuncData->pRslts + idx)->tick = tick;


         //v1 *= step;
         //v2 *= step;
         //v3 *= step;
         //v4 *= step;
         
         //(padcCapFuncData->pRslts + idx)->results0 = v1;
         //(padcCapFuncData->pRslts + idx)->results1 = v2;
         //(padcCapFuncData->pRslts + idx)->results2 = v3;
         //(padcCapFuncData->pRslts + idx)->results3 = v4;
         
         //TODO - convert to int type
         int dataIn = adcData.channel0;
         if(dataIn > 0x7fffff)
         {
            dataIn = (adcData.channel0 | 0xff000000);
         }
         (padcCapFuncData->pRslts + idx)->results0 = (float)dataIn;
         
         dataIn = adcData.channel1;
         if(dataIn > 0x7fffff)
         {
            dataIn = (adcData.channel1 | 0xff000000);
         }
         (padcCapFuncData->pRslts + idx)->results1 = (float)dataIn;
         
         dataIn = adcData.channel2;
         if(dataIn > 0x7fffff)
         {
            dataIn = (adcData.channel2 | 0xff000000);
         }
         (padcCapFuncData->pRslts + idx)->results2 = (float)dataIn;
         
         dataIn = adcData.channel3;
         if(dataIn > 0x7fffff)
         {
            dataIn = (adcData.channel3 | 0xff000000);
         }
         (padcCapFuncData->pRslts + idx)->results3 = (float)dataIn;

         
         //printf("Data(%d, %u): 0x%x, %.02f, %.02f, %.2f, %.2f\n", gpio, tick-lpreTick, adcData.response, v1, v2, v3, v4);
               
   }
   
   return;
}

/*************************************************
 * ads131m04 support routines.
 * 
 * **********************************************
 * 
 */
 int tspi_ads131m04_start(regInfor *pregInf, adc_channel_data *padcData) // start ads131m04, return SPI handle
{
   int h, vclk, vrst0, vrst1, vdrdy;
   uint16_t rep0, rep;
   // char txBuf[8] = {0};
   // char rxBuf[8] = {0};
   // adc_channel_data Data;

   printf("ads131 start up.\n");

   /* set the ADC_CLKIN_EN */
   gpioSetMode(ADC_CLKIN_EN, PI_OUTPUT);
   gpioWrite(ADC_CLKIN_EN, 1); // enable external clock 8.024MHz???
   vclk = gpioRead(ADC_CLKIN_EN);

   /* set the ADC_SYNC_RST */
   gpioSetMode(ADC_SYNC_RST, PI_OUTPUT);
   gpioWrite(ADC_SYNC_RST, 0); // set low to reset chip
   vrst0 = gpioRead(ADC_SYNC_RST);

   /* set the ADC_DRDY */
   gpioSetMode(ADC_DRDY, PI_INPUT); // set DRNY input
   vdrdy = gpioRead(ADC_DRDY);
   gpioDelay(1000);

   /* this test requires a ADS131M04 on SPI channel 1 */
   /*
   *******************************************************************
   * the spiopen() has three parameters: spiChan, baud, and spiFlags *
   *   spiChan - two channels for main SPI, 0(gpio8) & 1(gpio7)      *
   *   baud - SPI speed, 1,250,000 Hz                                *
   *   spiFlags - the SPI module settings.                           *
   *   -------------------------------------------------------       *
   *   |21 |20 |19 |18 |17 |16 |15 |14 |13 |12 |11 |10 |9 |8 |       *
   *   |---|---|---|---|---|---|---|---|---|---|---|---|--|--|       *
   *   |b  |b  |b  |b  |b  |b  |R  |T  |n  |n  |n  |n  |W |A |       *
   *   |------------------------------------------------------       *
   *   |7  |6  |5  |4  |3  |2  |1  |0  |                             *
   *   |---|---|---|---|---|---|---|---|                             *
   *   |u2 |u1 |u0 |p2 |p1 |p0 |m  |m  |                             *
   *   ---------------------------------                             *
   *    A - 0 for main SPI, 1 for auciliart SPI                      *
   *    W - 0 the device is not 3-wire, 1 the device is 3-wire.      *
   *     e.g set to 0.                                               *
   *******************************************************************
   *
   */

   // h = spiOpen(0, 1250000, 1); // open SPI decice "/dev/spidev0.0" with mode 1 for ads131m04
   h = spiOpen(0, 2500000, 1); // open SPI decice "/dev/spidev0.0" with mode 1 for ads131m04
   //CHECK(12, 1, h, 4, 100, "spiOpenADC");
   printf("ADC - %d", h);

   /* Initialize SPI device ADS131M04 */
   /*
    *
    *******************************************************************
    * The commands send to initialize ADS131M04:                      *
    *  # reset the chip with sending a low puls on RST pin            *
    *  # restore registers with defaults settings. internal records   *
    *  # validate first response word when beginning SPI              *
    *           (0xFF20 | CHANCNT)                                    *
    *  # configure MODE registers with defaults settings.             *
    *                                                                 *
    *******************************************************************
    *
    */

   // set reset pin high to finish the resetting chip
   gpioDelay(2000);
   gpioWrite(ADC_SYNC_RST, 1); // set high to end the reset chip
   vrst1 = gpioRead(ADC_SYNC_RST);

   //printf("\nads131 ctrl signals: enclk-%d, rst0-%d, rst1-%d, vdrdy-%d.\n", vclk, vrst0, vrst1, vdrdy);

   // write to Mode register(0x2) to enforce mode settings
   pregInf->regAddr = 2;
   pregInf->setData = 0x510;
   rep = sendCommand(h, OPCODE_WREG, pregInf, padcData);

   // valiate first response word with (0xFF20 | CHANCNT)
   // rep0 = (uint16_t)retrData(h, OPCODE_NULL, 2); //it's done in th esendCommand()???
   rep0 = padcData->response; //adcData.response;
   printf("start ads131m04 0x%x. \n", rep0);
   gpioDelay(500);
   
   /* set alert callback function */
   //userData* pfuncData = &adcCapFuncData;

   //pfuncData->handle = h; pfuncData->isRun = 0;
   //gpioSetAlertFuncEx(ADC_DRDY, adcCaptureFun, pfuncData);

   return h; // pigpio set, spi opened and return api handle, h.
}

int tspi_ads131m04_close(int SPIhandler) // close ads131m04, return SPI handle
{
   int ext;
   ext = spiClose(SPIhandler);
   //CHECK(12, 99, ext, 0, 0, "spiClose");

   gpioWrite(ADC_CLKIN_EN, 0); // disable external clock 8.024MHz

   return ext;
}


/* the code from piPeriphTest.c */
#if 0
int main(int argc, char *argv[]) // it will be divided to two ports: initialization & funciton(s).
{
   int h, i, t, c, status;
   double value = 0;
   int channel; // command;

   //uint16_t reps;
   char command[16]; //*piPeriID=argv[1];
   int reps, addr, setData, numRegs;
   adc_channel_data *padcData = &adcData;
   regInfor *pregInf = &regSetInf;

   status = gpioInitialise();

   if (status < 0)
   {
      printf("pigpio initialisation failed.\n");
      return 1;
   }

   h = tspi_ads131m04_start(pregInf, padcData); // open the pigpio spi
   //printf("set GPIO = %d.\n", GPIO);
   
   /* use alert function */
   userData* pfuncData = &adcCapFuncData;

   pfuncData->pRslts = &adcRltData[0]; pfuncData->datIdx = 0;
   pfuncData->handle = h; pfuncData->isRun = 0;
   gpioSetAlertFuncEx(ADC_DRDY, NULL, pfuncData);

   printf("pigpio started.\n    Input Command: ");
   scanf("%s", command);

   while (1) // the initialization port is above. the funcitons are in the while loop.
   {
      // printf("execute %s - %d.\n", command, addr);

      if (!strcmp("Reset", command))
      {
         // execute code;
         reps = tspi_ads131m04_rst(h);
         printf("    Input Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("Read", command))
      {
         //printf("    Input Address and No. Regs: ");
         //scanf("%d %d", &addr, &numRegs);
         printf("    Input Address: ");
         scanf("%x", &addr);
         // execute code;
         pregInf->regAddr = addr;
         pregInf->numRegs = 0; //numRegs;
         reps = tspi_ads131m04_rd(h, pregInf);
         printf("     reps: 0x%x \n    Input Command: ", reps);
         scanf("%s", command);
      }
      else if (!strcmp("Write", command))
      {
         printf("    Input Address and Setting: ");
         scanf("%x %x", &addr, &setData);
         // execute code;
         pregInf->regAddr = addr;
         pregInf->setData = setData;
         pregInf->numRegs = 0; // only set one register
         reps = tspi_ads131m04_wt(h, pregInf);
         printf("    reps: 0x%0x \n    Input Command: ", reps);
         scanf("%s", command);
      }
      else if (!strcmp("SetChxIntTest", command))
      {
         printf("    Input Setting: ");
         scanf("%x", &setData);
         // execute code;
         pregInf->regAddr = 0x09; //set channel0
         if((setData < 0) || (setData > 3))
            setData = 0;
         pregInf->setData = setData;
         pregInf->numRegs = 0; // only set one register
         reps = tspi_ads131m04_wt(h, pregInf);
         gpioDelay(20);

         pregInf->regAddr = 0x0e; //set channel1
         pregInf->setData = setData;
         pregInf->numRegs = 0; // only set one register
         reps = tspi_ads131m04_wt(h, pregInf);
         gpioDelay(20);

         pregInf->regAddr = 0x13; //set channel2
         pregInf->setData = setData;
         pregInf->numRegs = 0; // only set one register
         reps = tspi_ads131m04_wt(h, pregInf);
         gpioDelay(20);

         pregInf->regAddr = 0x18; //set channel3
         pregInf->setData = setData;
         pregInf->numRegs = 0; // only set one register
         reps = tspi_ads131m04_wt(h, pregInf);
         gpioDelay(20);

         printf("    reps: 0x%0x \n    Input Command: ", reps);
         scanf("%s", command);
      }
      /*       
      else if (!strcmp("Data", command)) // no this command
      {
         // execute code;
         reps = tspi_ads131m04_rdData(h, padcData);
         printf("    Input Command: ");
         scanf("%s", command);
      }
      */
      else if (!strcmp("Close", command))
      {
         // execute code;
         reps = tspi_ads131m04_close(h);
         printf("Close Execution.\n");
         break;
      }
      else if (!strcmp("GenWave", command))
      {
         // execute code;
         reps = twave_gen(h); // generate waveforms
         printf("    Input Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("ClsWave", command))
      {
         // execute code;
         reps = twave_cls(h); // close waveforms
         printf("    Input Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("SetDAC", command))
      {
         float value = 0;
         printf("    Input ChID and Value: ");
         scanf("%d %f", &addr, &value);
         // execute code;
         tspi_mcp4822(addr, 2, value);
         printf("    Input Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("LdisStart", command))
      {
         /* start Laser Distance main */
         UART_distMain(LASERDST);

         printf("    Input Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("TctrlStartGPS", command))
      {
         /* start Laser Distance main */
         //UART_tempCMain(TEMPCTRL);

         /* for python GPS peaser test */
         char *pargu1 = "nmeaParser";
         char argu2[32] = "parsMsg", argu3[32] = "The GPS parser calling test!";
         printf("    Input call name and argument: ");
         scanf("%s %s", &argu2, &argu3);


         tempCtrll_py(3, pargu1, &argu2, &argu3);
         printf("    Input Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("DbgData", command))
      {
         printf("Tx0:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", txBuf[0], txBuf[1], txBuf[2], txBuf[3], txBuf[4], txBuf[5], txBuf[6], txBuf[7], txBuf[8]);
         printf("Tx1:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", txBuf[9], txBuf[10], txBuf[11], txBuf[12], txBuf[13], txBuf[14], txBuf[15], txBuf[16], txBuf[17]);

         printf("Rx0:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", rxBuf[0], rxBuf[1], rxBuf[2], rxBuf[3], rxBuf[4], rxBuf[5], rxBuf[6], rxBuf[7], rxBuf[8]);
         printf("Rx1:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", rxBuf[9], rxBuf[10], rxBuf[11], rxBuf[12], rxBuf[13], rxBuf[14], rxBuf[15], rxBuf[16], rxBuf[17]);

         // printf("Data: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", adcData.response, adcData.channel0, adcData.channel1, adcData.channel2, adcData.channel3);
         double step = 1200000.0 / 8388607.0;
         double v1, v2, v3, v4; 
         
         if(adcData.channel0 > 0x7fffff)
         {
            v1 = (double)(~(adcData.channel0 | 0xff000000)+1);
            v1 = -v1;
         }
         else
         {
            v1 = (double)adcData.channel0;
         }

         if(adcData.channel1 > 0x7fffff)
         {
            v2 = (double)(~(adcData.channel1 | 0xff000000)+1);
            v2 = -v2;
         }
         else
         {
            v2 = (double)adcData.channel1;
         }
         
         if(adcData.channel2 > 0x7fffff)
         {
            v3 = (double)(~(adcData.channel2 | 0xff000000)+1);
            v3 = -v3;
         }
         else
         {
            v3 = (double)adcData.channel2;
         }
         
         if(adcData.channel3 > 0x7fffff)
         {
            v4 = (double)(~(adcData.channel3 | 0xff000000)+1);
            v4 = -v4;
         }
         else
         {
            v4 = (double)adcData.channel3;
         }

         v1 *= step;
         v2 *= step;
         v3 *= step;
         v4 *= step;
         printf("Data: 0x%x, %.02f, %.02f, %.02f, %.02f\n", adcData.response, v1, v2, v3, v4);

         printf("    Input Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("CaptureADCDbg", command))
      {
         #if 0
         /* use alert function */
         pfuncData->isRun = 1;
         
         //gpioSetAlertFuncEx(ADC_DRDY, adcCaptureFun, pfuncData);
         printf(" ");
         scanf("%s", command);
             
         pfuncData->isRun = 0;
            
         /* check the capture data */
         printf("\n ");
         for(int n = 0; n < ADCLNTH; n++)
         {
            //(padcCapFuncData->pRslts + idx)->results0 = v1;
            //(padcCapFuncData->pRslts + idx)->results1 = v2;
            //(padcCapFuncData->pRslts + idx)->results2 = v3;
            //(padcCapFuncData->pRslts + idx)->results3 = v4;
         
            printf("Data(%d, %u): %.2f, %.2f, %.2f, %.2f\n", n, 
               (pfuncData->pRslts + n)->tick,
               (pfuncData->pRslts + n)->results0,
               (pfuncData->pRslts + n)->results1,
               (pfuncData->pRslts + n)->results2,
               (pfuncData->pRslts + n)->results3);
         }

         /* end of using alert function */
         #endif
      
         #if 1 //read data number of data
         printf("    Input No. to capture: ");
         scanf("%d", &setData);

         pregInf->regAddr = 1;
         pregInf->numRegs = 0; //numRegs;
         reps = tspi_ads131m04_rd(h, pregInf);
         
         int count = 0, countf = 0;
         double step = 1200000.0 / 8388607.0;
         double v1, v2, v3, v4;
         uint32_t ticksSum = 0;

         while((count < setData) && (countf < (200 + setData)))
         {
            if(0x50f == adcData.response)
            {

               if(adcData.channel0 > 0x7fffff)
               {
                  v1 = (double)(~(adcData.channel0 | 0xff000000)+1);
                  v1 = -v1;
               }
               else
               {
                  v1 = (double)adcData.channel0;
               }

               if(adcData.channel1 > 0x7fffff)
               {
                  v2 = (double)(~(adcData.channel1 | 0xff000000)+1);
                  v2 = -v2;
               }
               else
               {
                  v2 = (double)adcData.channel1;
               }
               
               if(adcData.channel2 > 0x7fffff)
               {
                  v3 = (double)(~(adcData.channel2 | 0xff000000)+1);
                  v3 = -v3;
               }
               else
               {
                  v3 = (double)adcData.channel2;
               }
               
               if(adcData.channel3 > 0x7fffff)
               {
                  v4 = (double)(~(adcData.channel3 | 0xff000000)+1);
                  v4 = -v4;
               }
               else
               {
                  v4 = (double)adcData.channel3;
               }
               
               v1 *= step;
               v2 *= step;
               v3 *= step;
               v4 *= step;
               
               printf("Data(%d): %u, 0x%x, %.02f, %.02f, %.2f, %.2f\n", count, ticksSum, adcData.response, v1, v2, v3, v4);
               count++;
               ticksSum = 0;
            }

            ticksSum += gpioDelay(5000); //1000000/200Hz
            reps = tspi_ads131m04_rd(h, pregInf);
            countf++;
         }
         //printf("Tx0:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", txBuf[0], txBuf[1], txBuf[2], txBuf[3], txBuf[4], txBuf[5], txBuf[6], txBuf[7], txBuf[8]);
         //printf("Tx1:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", txBuf[9], txBuf[10], txBuf[11], txBuf[12], txBuf[13], txBuf[14], txBuf[15], txBuf[16], txBuf[17]);

         //printf("Rx0:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", rxBuf[0], rxBuf[1], rxBuf[2], rxBuf[3], rxBuf[4], rxBuf[5], rxBuf[6], rxBuf[7], rxBuf[8]);
         //printf("Rx1:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", rxBuf[9], rxBuf[10], rxBuf[11], rxBuf[12], rxBuf[13], rxBuf[14], rxBuf[15], rxBuf[16], rxBuf[17]);

         // printf("Data: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", adcData.response, adcData.channel0, adcData.channel1, adcData.channel2, adcData.channel3);
         //double step = 1200000.0 / 8388607.0;
         //double v1, v2, v3, v4;

            //v1 = (double)(~(0xe00000));
            //printf("v1 %f.\n", v1);
            //v1 = 0.0 - v1;
            //printf("0-v1 %f.\n", v1);

         //printf("Data: 0x%x, %f, %f, %f, %f\n", adcData.response, v1, v2, v3, v4);
         #endif

         printf("    Input Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("CaptureADC", command))
      {
         #if 0
         /* use alert function */
         pfuncData->isRun = 1;
         
         //gpioSetAlertFuncEx(ADC_DRDY, adcCaptureFun, pfuncData);
         printf(" ");
         scanf("%s", command);
             
         pfuncData->isRun = 0;
            
         /* check the capture data */
         printf("\n ");
         for(int n = 0; n < ADCLNTH; n++)
         {
            //(padcCapFuncData->pRslts + idx)->results0 = v1;
            //(padcCapFuncData->pRslts + idx)->results1 = v2;
            //(padcCapFuncData->pRslts + idx)->results2 = v3;
            //(padcCapFuncData->pRslts + idx)->results3 = v4;
         
            printf("Data(%d, %u): %.2f, %.2f, %.2f, %.2f\n", n, 
               (pfuncData->pRslts + n)->tick,
               (pfuncData->pRslts + n)->results0,
               (pfuncData->pRslts + n)->results1,
               (pfuncData->pRslts + n)->results2,
               (pfuncData->pRslts + n)->results3);
         }

         /* end of using alert function */
         #endif
      
         #if 1 //read data number of data
         printf("    Input No. to capture: ");
         scanf("%d", &setData);

         pregInf->regAddr = 1;
         pregInf->numRegs = 0; //numRegs;
         reps = tspi_ads131m04_rd(h, pregInf);
         
         int count = 0, countf = 0;
         double step = 1200000.0 / 8388607.0;
         double v1, v2, v3, v4;
         uint32_t ticksSum = 0;

         while((count < setData) && (countf < 200*setData))
         {
            if(0x50f == adcData.response)
            {

               if(adcData.channel0 > 0x7fffff)
               {
                  v1 = (double)(~(adcData.channel0 | 0xff000000)+1);
                  v1 = -v1;
               }
               else
               {
                  v1 = (double)adcData.channel0;
               }

               if(adcData.channel1 > 0x7fffff)
               {
                  v2 = (double)(~(adcData.channel1 | 0xff000000)+1);
                  v2 = -v2;
               }
               else
               {
                  v2 = (double)adcData.channel1;
               }
               
               if(adcData.channel2 > 0x7fffff)
               {
                  v3 = (double)(~(adcData.channel2 | 0xff000000)+1);
                  v3 = -v3;
               }
               else
               {
                  v3 = (double)adcData.channel2;
               }
               
               if(adcData.channel3 > 0x7fffff)
               {
                  v4 = (double)(~(adcData.channel3 | 0xff000000)+1);
                  v4 = -v4;
               }
               else
               {
                  v4 = (double)adcData.channel3;
               }
               
               v1 *= step;
               v2 *= step;
               v3 *= step;
               v4 *= step;
               
               printf("Data(%d): %.02f, %.02f, %.2f, %.2f\n", count, v1, v2, v3, v4);
               count++;
               ticksSum = 0;
            }

            ticksSum += gpioDelay(5000); //1000000/200Hz
            reps = tspi_ads131m04_rd(h, pregInf);
            countf++;
         }
         //printf("Tx0:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", txBuf[0], txBuf[1], txBuf[2], txBuf[3], txBuf[4], txBuf[5], txBuf[6], txBuf[7], txBuf[8]);
         //printf("Tx1:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", txBuf[9], txBuf[10], txBuf[11], txBuf[12], txBuf[13], txBuf[14], txBuf[15], txBuf[16], txBuf[17]);

         //printf("Rx0:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", rxBuf[0], rxBuf[1], rxBuf[2], rxBuf[3], rxBuf[4], rxBuf[5], rxBuf[6], rxBuf[7], rxBuf[8]);
         //printf("Rx1:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", rxBuf[9], rxBuf[10], rxBuf[11], rxBuf[12], rxBuf[13], rxBuf[14], rxBuf[15], rxBuf[16], rxBuf[17]);

         // printf("Data: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", adcData.response, adcData.channel0, adcData.channel1, adcData.channel2, adcData.channel3);
         //double step = 1200000.0 / 8388607.0;
         //double v1, v2, v3, v4;

            //v1 = (double)(~(0xe00000));
            //printf("v1 %f.\n", v1);
            //v1 = 0.0 - v1;
            //printf("0-v1 %f.\n", v1);

         //printf("Data: 0x%x, %f, %f, %f, %f\n", adcData.response, v1, v2, v3, v4);
         #endif

         printf("    Input Command: ");
         scanf("%s", command);
      }
      else
      {
         printf("\nIncorrect command!\n\n");
         printf("The commands should be:\n");
         printf("    \"GenWave\" to generate waveforms. \"ClsWave\" to stop waveforms.\n\n");

         printf("    \"SetDAC\" to set DAC output value.\n");
         printf("    \"SetChxIntTest\" to set ADC internal capture test. 0-nomal; 1-0;\n");
         printf("                      2-160mv; 3--160mv.\n");
         printf("    \"CaptureADC\" to capture ADC input.\n");
         printf("    \"Close\" to stop the application.\n\n");

         printf("    \"LdisStart\" to start the Laser Distence Sencor.\n\n");
         printf("    \"TctrlStart -unavailable\" to start the Temperature Contoller.\n\n");

         printf("    Input Command: ");
         scanf("%s", command);
         //break;
      }
   }

   printf("close the pigpio. \n");

   gpioTerminate();
   return 0;
}
#endif

