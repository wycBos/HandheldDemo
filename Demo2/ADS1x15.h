/* ADS1x15.h file is created for test I2C API in pigpio
it is retrived from lg_ads1x15 in lg ligrary.
*/
/*******************************************************************************
 * The file name will be changed to AD_DAC.h as the AD_DAC's header file.
 * It will support measuurer_Utility.c functions, as well as main file, HandMeasurer.c
 * 
 * *
 * 
 * *****/

/* orignal constants for ADS1x115 ADC operations */
#ifndef ADS1x115_H
#define ADS1x115_H

#include <stdio.h>
#include <wiringPi.h>
//#include <wiringSerial.h>
#include <stdbool.h>
#include <pigpio.h>

#define   ADS1X15_A0_1 0
#define   ADS1X15_A0_3 1
#define   ADS1X15_A1_3 2
#define   ADS1X15_A2_3 3
#define   ADS1X15_A0   4
#define   ADS1X15_A1   5
#define   ADS1X15_A2   6
#define   ADS1X15_A3   7

#define   ADS1X15_ALERT_NEVER       0
#define   ADS1X15_ALERT_READY       1
#define   ADS1X15_ALERT_TRADITIONAL 2
#define   ADS1X15_ALERT_WINDOW      3

#define CONVERSION_REG 0
#define CONFIG_REG 1
#define COMPARE_LOW_REG 2
#define COMPARE_HIGH_REG 3

/* new for MCP4822 */
#define DAC_LDAC 19
#define MCP4822_DAB (1 << 15)
#define MCP4822_GA1 (1 << 13)  //0-set to 2x gain; 1-set to 1x gain.
#define MCP4822_ACT (1 << 12)  //0-disable the DAC output; 1-enable the DAC output.

/* new for ADS131 */
#define ADC_CLKIN_EN 21
#define ADC_SYNC_RST 20
#define ADC_DRDY 16
#define ADCLNTH 32

// ADS131 SPI commands definitions
#define OPCODE_NULL ((uint16_t)0x0000)
#define OPCODE_RESET ((uint16_t)0x0011)
#define OPCODE_RREG ((uint16_t)0xA000)
#define OPCODE_WREG ((uint16_t)0x6000)
#define OPCODE_STANDBY ((uint16_t)0x0022)
#define OPCODE_WAKEUP ((uint16_t)0x0033)
#define OPCODE_LOCK ((uint16_t)0x0555)
#define OPCODE_UNLOCK ((uint16_t)0x0655)

#define ID_ADDRESS ((uint8_t)0x00)
#define ID_DEFAULT ((uint16_t)0x2000 | (CHANNEL_COUNT << 8)) // NOTE: May change with future device revisions!
#define STATUS_ADDRESS ((uint8_t)0x01)
#define STATUS_DEFAULT ((uint16_t)0x0500)
#define MODE_ADDRESS ((uint8_t)0x02)
#define MODE_DEFAULT ((uint16_t)0x0510)
#define CLOCK_ADDRESS ((uint8_t)0x03)
#define CLOCK_DEFAULT ((uint16_t)0x0F0E)
#define GAIN1_ADDRESS ((uint8_t)0x04)
#define GAIN1_DEFAULT ((uint16_t)0x0000)
#define GAIN2_ADDRESS ((uint8_t)0x05)
#define GAIN2_DEFAULT ((uint16_t)0x0000)
#define CFG_ADDRESS ((uint8_t)0x06)
#define CFG_DEFAULT ((uint16_t)0x0600)
#define THRSHLD_MSB_ADDRESS ((uint8_t)0x07)
#define THRSHLD_MSB_DEFAULT ((uint16_t)0x0000)
#define THRSHLD_LSB_ADDRESS ((uint8_t)0x08)
#define THRSHLD_LSB_DEFAULT ((uint16_t)0x0000)
#define CH0_CFG_ADDRESS ((uint8_t)0x09)
#define CH0_CFG_DEFAULT ((uint16_t)0x0000)
#define CH0_OCAL_MSB_ADDRESS ((uint8_t)0x0A)
#define CH0_OCAL_MSB_DEFAULT ((uint16_t)0x0000)
#define CH0_OCAL_LSB_ADDRESS ((uint8_t)0x0B)
#define CH0_OCAL_LSB_DEFAULT ((uint16_t)0x0000)
#define CH0_GCAL_MSB_ADDRESS ((uint8_t)0x0C)
#define CH0_GCAL_MSB_DEFAULT ((uint16_t)0x0000)
#define CH0_GCAL_LSB_ADDRESS ((uint8_t)0x0D)
#define CH0_GCAL_LSB_DEFAULT ((uint16_t)0x0000)
#define CH1_CFG_ADDRESS ((uint8_t)0x0E)
#define CH1_CFG_DEFAULT ((uint16_t)0x0000)
#define CH1_OCAL_MSB_ADDRESS ((uint8_t)0x0F)
#define CH1_OCAL_MSB_DEFAULT ((uint16_t)0x0000)
#define CH1_OCAL_LSB_ADDRESS ((uint8_t)0x10)
#define CH1_OCAL_LSB_DEFAULT ((uint16_t)0x0000)
#define CH1_GCAL_MSB_ADDRESS ((uint8_t)0x11)
#define CH1_GCAL_MSB_DEFAULT ((uint16_t)0x8000)
#define CH1_GCAL_LSB_ADDRESS ((uint8_t)0x12)
#define CH1_GCAL_LSB_DEFAULT ((uint16_t)0x0000)
#define CH2_CFG_ADDRESS ((uint8_t)0x13)
#define CH2_CFG_DEFAULT ((uint16_t)0x0000)
#define CH2_OCAL_MSB_ADDRESS ((uint8_t)0x14)
#define CH2_OCAL_MSB_DEFAULT ((uint16_t)0x0000)
#define CH2_OCAL_LSB_ADDRESS ((uint8_t)0x15)
#define CH2_OCAL_LSB_DEFAULT ((uint16_t)0x0000)
#define CH2_GCAL_MSB_ADDRESS ((uint8_t)0x16)
#define CH2_GCAL_MSB_DEFAULT ((uint16_t)0x8000)
#define CH2_GCAL_LSB_ADDRESS ((uint8_t)0x17)
#define CH2_GCAL_LSB_DEFAULT ((uint16_t)0x0000)
#define CH3_CFG_ADDRESS ((uint8_t)0x18)
#define CH3_CFG_DEFAULT ((uint16_t)0x0000)
#define CH3_OCAL_MSB_ADDRESS ((uint8_t)0x19)
#define CH3_OCAL_MSB_DEFAULT ((uint16_t)0x0000)
#define CH3_OCAL_LSB_ADDRESS ((uint8_t)0x1A)
#define CH3_OCAL_LSB_DEFAULT ((uint16_t)0x0000)
#define CH3_GCAL_MSB_ADDRESS ((uint8_t)0x1B)
#define CH3_GCAL_MSB_DEFAULT ((uint16_t)0x8000)
#define CH3_GCAL_LSB_ADDRESS ((uint8_t)0x1C)
#define CH3_GCAL_LSB_DEFAULT ((uint16_t)0x0000)
#define REGMAP_CRC_ADDRESS ((uint8_t)0x3E)
#define REGMAP_CRC_DEFAULT ((uint16_t)0x0000)

#define NUM_REGISTERS ((uint8_t)64)

#define SAMPRAT (1000000/210)

/* original data types used for ADS1x115 */
typedef struct ads1x15_s
{
   /* I2C device parameters */
   int sbc;       // sbc connection (unused)
   int bus;       // I2C bus
   int device;    // I2C device
   int flags;     // I2C flags
   int i2ch;      // I2C handle
   
   /* alert signal setting */
   int alert_rdy; // mode of ALERT_RDY pin
   /* configuration data format */
   int configH;   // config high byte
   int configL;   // config low byte
   /* AD converting parameters */
   int channel;   // channel setting
   int gain;      // gain setting
   /* sample rate constants */
   int *SPS;      // array of legal samples per seconds
   /* gain parameter */
   float voltage_range; // voltage range (set by gain setting)
   /* lo/hi threshholds */
   float vhigh;   // alert high voltage
   float vlow;    // alert low voltage
   
   int single_shot; // single shot setting
   /* sample rate setting */
   int sps;       // samples per second setting
   /* comparater configrations */
   int comparator_mode;
   int comparator_polarity;
   int comparator_latch;
   int comparator_queue;
   int compare_high; // set from vhigh
   int compare_low;  // set from vlow
   
   int set_queue;    // set from comparator queue
} ads1x15_t;

typedef ads1x15_t *ads1x15_p;

/* the data for ADS131 */
typedef struct ADC131_data_t
{
   uint16_t response;
   uint16_t crc;
   int32_t channel0;
   int32_t channel1;
   int32_t channel2;
   int32_t channel3;
} adc_channel_data;

typedef struct ADC131_RegInfo_t
{
   uint16_t regAddr;
   uint16_t setData;
   uint16_t numRegs;
} regInfor;

typedef struct CALRatio_t
{
   float ratio;
   float squF1;
   float squF2;
}caliRlt;

typedef struct CPARatio_t
{
   int dataCnt;
   caliRlt Rslt[30];
}manuCst;

extern adc_channel_data adcData; //TODO - move to .c file
extern regInfor regSetInf; //TODO - move to AD_DAC.c file

typedef struct ADCRsults_t{
   uint32_t tick;
   double results0;
   double results1;
   double results2;
   double results3;
}adcRslts;

typedef struct UserData_t{// it's use in the file.
   int handle;
   int isRun;
   int datIdx;
   uint32_t preTick;
   adcRslts *pRslts;
}userData;

//adcRslts adcRltData[ADCLNTH]; // TODO - move to .c file
extern userData adcCapFuncData; // TODO - move to .c file

/* functions supports ADS1115 AD Concerter */
//ads1x15_p adc;
//extern ads1x15_p adc;
float ADS1115_main(void);

int ADS1X15_set_comparator_polarity(ads1x15_p s, int level);
int ADS1X15_get_comparator_polarity(ads1x15_p s);
int ADS1X15_set_comparator_latch(ads1x15_p s, int value);
int ADS1X15_get_comparator_latch(ads1x15_p s);
int ADS1X15_set_comparator_queue(ads1x15_p s, int queue);
int ADS1X15_get_comparator_queue(ads1x15_p s);
int ADS1X15_set_continuous_mode(ads1x15_p s);
int ADS1X15_set_single_shot_mode(ads1x15_p s);
int ADS1X15_get_conversion_mode(ads1x15_p s);
int ADS1X15_set_sample_rate(ads1x15_p s, int rate);
int ADS1X15_get_sample_rate(ads1x15_p s);
float ADS1X15_set_voltage_range(ads1x15_p s, float vrange);
float ADS1X15_get_voltage_range(ads1x15_p s);
int ADS1X15_set_channel(ads1x15_p s, int channel);
int ADS1X15_get_channel(ads1x15_p s);
int ADS1X15_alert_when_high_clear_when_low(ads1x15_p s, float vhigh, float vlow);
int ADS1X15_alert_when_high_or_low(ads1x15_p s, float vhigh, float vlow);
int ADS1X15_alert_when_ready(ads1x15_p s);
int ADS1X15_alert_never(ads1x15_p s);
int ADS1X15_get_alert_data(ads1x15_p s, int *high, int *low);
int ADS1X15_read_config_data(ads1x15_p s, int *high, int *low);
int ADS1X15_read(ads1x15_p s);
float ADS1X15_read_voltage(ads1x15_p s);
ads1x15_p ADS1X15_open(int sbc, int bus, int device, int flags);
ads1x15_p ADS1115_open(int sbc, int bus, int device, int flags);
ads1x15_p ADS1X15_close(ads1x15_p s);

/* callback function */
//void cbf(int e, lgGpioAlert_p evt, void *userdata);

/* the functions of MCP4822 */
void tspi_mcp4822(int channel, int command, double valu);

/* the functions of ADS131 */
void adcCaptureFun(int gpio, int level, uint32_t tick, userData* padcCapFuncData);
uint16_t sendCommand(int SPIHandle, uint16_t opcode, regInfor *regData, adc_channel_data *DataStruct);
int tspi_ads131m04_start(regInfor *pregInf, adc_channel_data *padcData);
int tspi_ads131m04_close(int SPIhandler);

#endif // End define ADS1x115_H/AD_DAC_H