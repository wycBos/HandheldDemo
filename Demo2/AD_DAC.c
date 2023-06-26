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
#include <pthread.h>
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

void tspi_mcp4822(int channel, int command, double value, double prevalue) // SPI channel test MCP4822
{
   /**********************************
    *  channel: 0-DAC_A; 1-DAC_B
    *  command: 0-DIS_ACT, 1-EN_ACT, 2-LDAC
    *  value: 0.0 - 2.0
    *************************************/

   //printf("    MCP4822 Settings. %d %d %f\n", channel, command, value);
   //return; // for debug

   int h, b, e;
   char txBuf[8];

   printf("    MCP4822 Settings. %d %d %f\n", channel, command, value);

   /* set the DAC_LDAC command 2 DAC_LDAC is high, otherwise it is low*/
   gpioSetMode(DAC_LDAC, PI_OUTPUT); // TODO - move to main()

   // if (command == 2)
   //    gpioWrite(DAC_LDAC, 1);
   // else
   //    gpioWrite(DAC_LDAC, 0);

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

   // h = spiOpen(1, 1250000, 0); // open SPI decice "/dev/spidev0.1" with mode 0 for MCP4822
   h = spiOpen(1, 4000000, 0); // open SPI decice "/dev/spidev0.1" with mode 0 for MCP4822
   // CHECK(12, 1, h, 4, 100, "spiOpenDAC");
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
   unsigned int numSteps, remainVal, valuSet, StepsPre, prevlSet, ctrlData, setValue;
   char byte0, byte1;

   /* set MCP4822 value */
   if (value < 0 && value > 2.048)
      value = 1.0;

   /* prepare numbers for setting */
   if(prevalue < 0 || prevalue > 2.048)
   {
      StepsPre = 0;
   }
   else{
      prevlSet = (unsigned int)(prevalue / 2.048 * 4096);
      StepsPre = prevlSet/10;
   }

   valuSet = (unsigned int)(value / 2.048 * 4096);
   /* setting DAC_A value from 0 to set value with increacing 10 digital number per 20ms */
   numSteps = valuSet / 10;
   remainVal = valuSet % 10;

   /* set MCP4822 control bits */
   ctrlData = MCP4822_GA1;
   if (channel == 1)
      ctrlData |= MCP4822_DAB; // set DA-B
   else if (channel == 0)
      ctrlData &= (~MCP4822_DAB); // set DA-A

   if (command == 0)
      ctrlData &= (~MCP4822_ACT); // no DA activite
   else if (command == 1 || command == 2)
      ctrlData |= MCP4822_ACT; // set DA activite

   /* set DAC value */
   if (channel == 0) // DAC_A
   {
      printf("    MCP4822 CH_A steps. %d\n", numSteps);

      if(numSteps > StepsPre)
      {
         for(int n = StepsPre; n < numSteps + 1; n++)
         {
            setValue = ctrlData + n*10;
            //printf("    MCP4822 Data. %d\n", setValue);

            byte0 = setValue & 0xFF;
            byte1 = (setValue >> 8) & 0xFF;

            txBuf[1] = byte0;
            txBuf[0] = byte1;
            //printf("MCP4822 Data. %x %x %x\n", setValue, txBuf[1], txBuf[0]);

            /* write data to SPI */
            b = spiWrite(h, txBuf, 2);

            /* latch data to DAC */
            gpioWrite(DAC_LDAC, 0);
            gpioDelay(400);
            gpioWrite(DAC_LDAC, 1);
            gpioDelay(20000); // delay 20ms
         }
      }else if(numSteps < StepsPre)
      {
         for(int n = 0; n < (StepsPre - numSteps + 1); n++)
         {
            setValue = ctrlData + (StepsPre - n)*10;
            //printf("    MCP4822 Data. %d\n", setValue);

            byte0 = setValue & 0xFF;
            byte1 = (setValue >> 8) & 0xFF;

            txBuf[1] = byte0;
            txBuf[0] = byte1;
            //printf("MCP4822 Data. %x %x %x\n", setValue, txBuf[1], txBuf[0]);

            /* write data to SPI */
            b = spiWrite(h, txBuf, 2);
            
            /* latch data to DAC */
            gpioWrite(DAC_LDAC, 0);
            gpioDelay(400);
            gpioWrite(DAC_LDAC, 1);
            gpioDelay(20000); // delay 20ms
         }
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
   //printf("MCP4822 Data. %x %x %x\n", setValue, txBuf[1], txBuf[0]);

   /* write data to SPI */
   b = spiWrite(h, txBuf, 2);

   /* latch data to DAC */
   gpioWrite(DAC_LDAC, 0);
   gpioDelay(4000);
   gpioWrite(DAC_LDAC, 1);
   printf(" --- latch MCP4822 Data.\n");
   e = spiClose(h);
   // CHECK(12, 3, e, 0, 0, "spiClose");
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
userData adcCapFuncData;
adc_channel_data adcData; // TODO double check it
avgData avgFlt;
pthread_mutex_t pmtx_funcData;

/********************** prototype ****************/
void set_DAC() // TOTO - set MCP4822 output value
{
   return;
}

void init_ADC() // TOTO - init/config ADS131
{
   return;
}

void ADC_capture() // TOTO - run ADS131 value input, event handler
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
// TODO move into gpio_proc.c
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

   // printf("\ntxBuf0 numberOfBytes %d: 0x%x, 0x%x, 0x%x, 0x%x; \n", numberOfBytes, txBuf[0], txBuf[1], txBuf[2], txBuf[3]);

   // printf("txBuf1: 0x%x, 0x%x, 0x%x, 0x%x; \n", txBuf[4], txBuf[5], txBuf[6], txBuf[7]);

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

   return ret; // DataStruct->response;
}

uint16_t tspi_ads131m04_rd(int SPIhandler, regInfor *getInf)
{
   uint16_t ret = 0, rsp;
   regInfor *lpgetInf = getInf;
   adc_channel_data *lpadcData = &adcData; // TODO - move into arguments of the function

   /* reead ads131m04 register */
   // regSetInf.regAddr = getInf->regAddr;
   // regSetInf.setData = 0;
   ret = sendCommand(SPIhandler, OPCODE_RREG, lpgetInf, lpadcData);
   rsp = lpadcData->response;

   // printf("read Reg. addr - 0x%x, data - 0x%x. \n", getInf->regAddr, rsp);
   return rsp;
}

uint16_t tspi_ads131m04_wt(int SPIhandler, regInfor *setInf)
{
   uint16_t ret = 0, rsp;
   /* write ads131m04 register */
   regInfor *lpgetInf = setInf;
   adc_channel_data *lpadcData = &adcData; // TODO - move into arguments of function

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
regInfor regSetInf; // TODO - move to the definition part of this file.

void adcCaptureFun(int gpio, int level, uint32_t tick, userData *padcCapFuncData) // a callback function for capture adc data.
{
   int reps, h, isRn, idx, dataPtr, dataIn, dataOut, numData;
   uint32_t lpreTick;
   regInfor *pregInf = &regSetInf; // TODO - move into uaserData

   pthread_mutex_lock(&pmtx_funcData); //TODO - do not need?!
   isRn = padcCapFuncData->isRun; // TODO - check it need
#if 0 //move in if() statement
   lpreTick = padcCapFuncData->preTick;
   h = padcCapFuncData->handle;
   idx = (padcCapFuncData->datIdx);

   if(idx < FILTER_LEN)
   {
      if(idx <= 0) //less than 0 is not reasonable but in case to correct it...
      {
         numData = 1;
         avgFlt.updatePtr = 0;
         for(int n = 0; n < FILTER_LEN; n++)
         {
            avgFlt.sum0 = avgFlt.chan0[n] = 0;
            avgFlt.sum1 = avgFlt.chan1[n] = 0;
            avgFlt.sum2 = avgFlt.chan2[n] = 0;
            avgFlt.sum3 = avgFlt.chan3[n] = 0;
         }
      }else
         numData = idx + 1;
   }else
      numData = FILTER_LEN;
#endif
   if ((level == 0) && (isRn == 1) /* && (tick > (lpreTick + SAMPRAT))*/)
   {
      lpreTick = padcCapFuncData->preTick;
      h = padcCapFuncData->handle;
      idx = (padcCapFuncData->datIdx);

      if(idx < FILTER_LEN)
      {
         if(idx <= 0) //less than 0 is not reasonable but in case to correct it...
         {
            numData = 1;
            avgFlt.updatePtr = 0;
            for(int n = 0; n < FILTER_LEN; n++)
            {
               avgFlt.sum0 = avgFlt.chan0[n] = 0;
               avgFlt.sum1 = avgFlt.chan1[n] = 0;
               avgFlt.sum2 = avgFlt.chan2[n] = 0;
               avgFlt.sum3 = avgFlt.chan3[n] = 0;
            }
         }else
            numData = idx + 1;
      }else
         numData = FILTER_LEN;

      pregInf->regAddr = 1;
      pregInf->numRegs = 0; // numRegs;
      reps = tspi_ads131m04_rd(h, pregInf);

      /* updata the adcCapFuncData */
      padcCapFuncData->preTick = tick;
      (padcCapFuncData->pRslts + idx)->tick = tick;

      dataPtr = avgFlt.updatePtr;

      // TODO - convert to int type
      dataIn = adcData.channel0;
      if (dataIn > 0x7fffff)
      {
         dataIn = (adcData.channel0 | 0xff000000);
      }
      (padcCapFuncData->pRslts + idx)->results0 = (float)dataIn;
      dataOut = avgFlt.chan0[dataPtr];
      avgFlt.chan0[dataPtr] = dataIn;
      dataIn += avgFlt.sum0;
      dataIn -= dataOut;
      avgFlt.sum0 = dataIn;
      padcCapFuncData->avgData[0] = dataIn/numData;

      //padcCapFuncData->avgData[0] = dataIn;//for debug

      dataIn = adcData.channel1;
      if (dataIn > 0x7fffff)
      {
         dataIn = (adcData.channel1 | 0xff000000);
      }
      (padcCapFuncData->pRslts + idx)->results1 = (float)dataIn;
      dataOut = avgFlt.chan1[dataPtr];
      avgFlt.chan1[dataPtr] = dataIn;
      dataIn += avgFlt.sum1;
      dataIn -= dataOut;
      avgFlt.sum1 = dataIn;
      padcCapFuncData->avgData[1] = dataIn/numData;

      //padcCapFuncData->avgData[1] = dataIn;//for debug

      dataIn = adcData.channel2;
      if (dataIn > 0x7fffff)
      {
         dataIn = (adcData.channel2 | 0xff000000);
      }
      (padcCapFuncData->pRslts + idx)->results2 = (float)dataIn;
      dataOut = avgFlt.chan2[dataPtr];
      avgFlt.chan2[dataPtr] = dataIn;
      dataIn += avgFlt.sum2;
      dataIn -= dataOut;
      avgFlt.sum2 = dataIn;
      padcCapFuncData->avgData[2] = dataIn/numData;

      //padcCapFuncData->avgData[2] = dataIn;//for debug

      dataIn = adcData.channel3;
      if (dataIn > 0x7fffff)
      {
         dataIn = (adcData.channel3 | 0xff000000);
      }
      (padcCapFuncData->pRslts + idx)->results3 = (float)dataIn;
      dataOut = avgFlt.chan3[dataPtr];
      avgFlt.chan3[dataPtr] = dataIn;
      dataIn += avgFlt.sum3;
      dataIn -= dataOut;
      avgFlt.sum3 = dataIn;
      padcCapFuncData->avgData[3] = dataIn/numData;

      //padcCapFuncData->avgData[3] = dataIn;//for debug

      avgFlt.updatePtr = (dataPtr + 1)%FILTER_LEN;
      idx = (idx + 1) % ADCLNTH;
      padcCapFuncData->datIdx = idx;

      // printf("Data(%d, %u): 0x%x, %.02f, %.02f, %.2f, %.2f\n", gpio, tick-lpreTick, adcData.response, v1, v2, v3, v4);
      // printf("Data(%d, %u): %d\n", gpio, tick-lpreTick, adcData.response);
   }
   pthread_mutex_unlock(&pmtx_funcData);
   // printf("Data(%d, %u): %d\n", gpio, tick-lpreTick, level);
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

   h = spiOpen(0, 4000000, 1); // open SPI decice "/dev/spidev0.0" with mode 1 for ads131m04
   //h = spiOpen(0, 2500000, 1); // open SPI decice "/dev/spidev0.0" with mode 1 for ads131m04
   //CHECK(12, 1, h, 4, 100, "spiOpenADC");
   printf("ADC - %d\n", h);

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

   // printf("\nads131 ctrl signals: enclk-%d, rst0-%d, rst1-%d, vdrdy-%d.\n", vclk, vrst0, vrst1, vdrdy);

   // write to Mode register(0x2) to enforce mode settings
   pregInf->regAddr = MODE_ADDRESS; //2;
   pregInf->setData = MODE_DEFAULT; //0x510;
   rep = sendCommand(h, OPCODE_WREG, pregInf, padcData);

   //valiate first response word with (0xFF20 | CHANCNT)
   //rep0 = (uint16_t)retrData(h, OPCODE_NULL, 2); //it's done in th esendCommand()???
   rep0 = padcData->response; // adcData.response;
   printf("start ads131m04 0x%x. \n", rep0);
   gpioDelay(500);
   
   // write to clock register(0x3) to low power settings
   pregInf->regAddr = CLOCK_ADDRESS; //3;
   pregInf->setData = CLOCK_LOWPOWR; //0xF0D;
   rep = sendCommand(h, OPCODE_WREG, pregInf, padcData);

   // response word (mode settings)
   rep0 = padcData->response; //adcData.response;
   //printf("ads131m04 mode: 0x%x. \n", rep0);
   gpioDelay(500);

   // read to clock register(0x3) setting
   pregInf->regAddr = CLOCK_ADDRESS;
   pregInf->numRegs = 0;
   rep = sendCommand(h, OPCODE_RREG, pregInf, padcData);
   rep0 = padcData->response; //adcData.response;
   //printf("ads131m04 clk-Reg: 0x%x, 0x%x.\n", rep0, rep);
   gpioDelay(200);

   // read to clock register(0x3) setting
   pregInf->regAddr = CLOCK_ADDRESS;
   pregInf->numRegs = 0;
   rep = sendCommand(h, OPCODE_RREG, pregInf, padcData);

   rep0 = padcData->response; //adcData.response;
   printf("ads131m04 clk-Reg: 0x%x.\n", rep0);
   gpioDelay(200);

   /* set alert callback function TODO - remove it */
   //userData* pfuncData = &adcCapFuncData;

   // pfuncData->handle = h; pfuncData->isRun = 0;
   // gpioSetAlertFuncEx(ADC_DRDY, adcCaptureFun, pfuncData);

   return h; // pigpio set, spi opened and return api handle, h.
}

int tspi_ads131m04_close(int SPIhandler) // close ads131m04, return SPI handle
{
   int ext;
   ext = spiClose(SPIhandler);
   // CHECK(12, 99, ext, 0, 0, "spiClose");

   gpioWrite(ADC_CLKIN_EN, 0); // disable external clock 8.024MHz

   return ext;
}
