/* ADS1x15.c file is created for test I2C API for ADS1x15 chip in pigpio
it is retrived from lg_ads1x15 in lg ligrary.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
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

/* above for ADS1115 ADC */

/* TODO - add routines for MCP4822 and ADS131 */
/* Routines here are supported by SPI routines */
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

/* the code from piPeriphTest.c */
#if 0
int main(int argc, char *argv[])
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

   while (1)
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

