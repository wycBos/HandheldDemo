/**************************************
 * the pigpio application to generate two wave
 * code protype comes from pigpio example
 * *****************/

#include <stdio.h>
#include <wiringPi.h>
//#include <wiringSerial.h>
#include <pigpio.h>
#include "waveForm.h"
//#include "ADS1x15.h"

#if 1
static const unsigned int CFG_SAMRATE = 2;
int gpios[]={5,4,6,12,13}; //5-100kHz; 4-1KHz; 6-1KHz90degree; 12-2KHz; 13-2KHz90degree
gpioPulse_t pulses[]=
{
   /*
   {0x1020, 0x2040, 125}, 
   {0x3020, 0x0040, 125}, 
   {0x2060, 0x1000, 125}, 
   {0x0060, 0x3000, 125}, 
   {0x1040, 0x2020, 125}, 
   {0x3040, 0x0020, 125}, 
   {0x2000, 0x1060, 125}, 
   {0x0000, 0x3060, 125},
   */
   {0x1030, 0x0000, 5}, //1
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //2
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //3
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x2000, 0x0020, 5}, // -125us
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //4
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //5
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0060, 0x1000, 5}, //6 - 250us
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //7
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //8
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x2020, 5}, // -375us
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //9
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //10
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x1020, 0x0010, 5}, //11 - 500us
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //12
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //13
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x2000, 0x0020, 5}, // - 625us 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //14
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //15
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x1040, 5}, //16 - 750us
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //17
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //18
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x2020, 5}, //-875us 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //19
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //20
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

};
#else
int gpios[] = {5,6};
gpioPulse_t pulses[]=
{
   {0x60, 0x00, 20},
   {0x40, 0x20, 20},
   {0x60, 0x00, 24},
   {0x40, 0x20, 16},
   {0x60, 0x0, 29},
   {0x40, 0x20, 11},
   {0x60, 0x0, 33},
   {0x40, 0x20, 7},
   {0x60, 0x0, 36},
   {0x40, 0x20, 4},

   {0x20, 0x40, 39},
   {0x0, 0x60, 1},
   {0x20, 0x40, 39},
   {0x0, 0x60, 1},
   {0x20, 0x40, 39},
   {0x0, 0x60, 1},
   {0x20, 0x40, 38},
   {0x0, 0x60, 2},
   {0x20, 0x40, 35},
   {0x0, 0x60, 5},

   {0x20, 0x40, 31},
   {0x0, 0x60, 9},
   {0x20, 0x40, 27},
   {0x0, 0x60, 13},
   {0x20, 0x40, 22},
   {0x0, 0x60, 18},
   {0x20, 0x40, 17},
   {0x0, 0x60, 23},
   {0x20, 0x40, 12},
   {0x0, 0x60, 28},

   {0x20, 0x40, 8},
   {0x0, 0x60, 32},
   {0x20, 0x40, 4},
   {0x0, 0x60, 36},
   {0x20, 0x40, 1},
   {0x0, 0x60, 39},
   {0x20, 0x40, 0},
   {0x0, 0x60, 40},
   {0x20, 0x40, 0},
   {0x0, 0x60, 40},

   {0x20, 0x40, 0},
   {0x0, 0x60, 40},
   {0x20, 0x40, 3},
   {0x0, 0x60, 37},
   {0x20, 0x40, 6},
   {0x0, 0x60, 34},
   {0x20, 0x40, 10},
   {0x0, 0x60, 30},
   {0x20, 0x40, 15},
   {0x0, 0x60, 25},      
};
#endif
/* message for testing serial port */
char contimeas[4]   ={0x80,0x06,0x03,0x77};

//int main(int argc, char *argv[])
int wavePiset(void)
{
   int g, fd, wid=-1;
   ads1x15_p adc;
 
   //if (gpioInitialise() < 0) return 1;

   adc = ADS1115_open(0, 1, 0x48, 0);

   if (adc == NULL) return -2;

   printf("ADS1115 start. \n");

   ADS1X15_set_channel(adc, ADS1X15_A0);
   ADS1X15_set_voltage_range(adc, 3.3);
   ADS1X15_set_sample_rate(adc, 0); // set minimum sampling rate
#if 0 // test ADS
   float ADvolt = 0;
   int end_time, seconds, micros;

    
   gpioTime(1, &seconds, &micros);
   end_time = seconds + 1;

    while (seconds < end_time)
   {
      // lguSleep(0.2);
      gpioSleep(0, 1, 0);
      gpioTime(1, &seconds, &micros);
      printf("ADS1115 read. \n");

      ADvolt = ADS1X15_read_voltage(adc);

      printf("%.2f\n", ADvolt /*ADS1X15_read_voltage(adc)*/);
   }
#endif

#if 1
   for (g=0; g<sizeof(gpios)/sizeof(gpios[0]); g++)
      gpioSetMode(gpios[g], PI_OUTPUT);

   gpioWaveClear();
   gpioWaveAddGeneric(sizeof(pulses)/sizeof(pulses[0]), pulses);
   wid = gpioWaveCreate();
/*
   for(int n = 0; n < 4; n++)
   {
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      time_sleep(2);
   }
*/
   if (wid >= 0)
   {
      gpioWaveTxSend(wid, PI_WAVE_MODE_REPEAT_SYNC /*PI_WAVE_MODE_REPEAT*/);
      //time_sleep(100);
/*      
   for(int n = 0; n < 14; n++)
   {
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      time_sleep(2);
   }
*/
//      gpioWaveTxStop();
//      gpioWaveDelete(wid);
   }
/*
   for(int n = 0; n < 4; n++)
   {
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      time_sleep(2);
   }
*/
/*
   gpioTerminate();
   
   for(int n = 0; n < 4; n++)
   {
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      time_sleep(2);
   }
*/
#endif
   //ADvolt = ADS1X15_read_voltage(adc);
   //printf("-- %.2f\n", ADvolt /*ADS1X15_read_voltage(adc)*/);
   ADS1X15_close(adc);
   return wid;
}

int wavePistop(unsigned wid)
{
   int g, fd, ern=-1;
 
   ern = gpioWaveTxStop();
   ern = gpioWaveDelete(wid);

   gpioTerminate();
   
   return ern;
}
/* new routines start here, most of them wraps pigpio APIs. */

void gpio_start() //TOTO - start pigpio
{
   return;
}

void gpio_stop() //TOTO - close pigpio
{
   return;
}

void gpio_UART() //TOTO - run a UART port communication
{
   return;
}
/**********************************
 * The routines are used for UART port operations:
 *  data type: UARTport 
 *  serial_start()
 *  serial_close()
 *  serial_cmdMsg()
 *    serial_dataRdy()
 *    serial_write()
 *    serial_read()
 *  
 * ********************************/

void gpio_SPI() //TOTO - run a SPI communication
{
   return;
}

void gpio_I2C() //TOTO - run a I2C communicaiton
{
   return;
}

void gpio_genWave() //TOTO - replace the wavepiset()
{
   return;
}

void gpio_clsWave() //TOTO - replace the wavepistop()
{
   return;
}

/* code from piTest serial file */
/**************************************
 * the pigpio application to communicat via UART
 * code protype comes from pigpio example
 *
 * The following functions are included
 *  - open a serial port with parameters: port_id, baud rate, and flag (always 0).
 *    return a handle >= 0.
 *  - close a serial port indicated with a parameter: handle.
 *    return 0 if Ok.
 *  - send data stored in a tx buffer
 *    return 0 if Ok.
 *  - read data into rx buffer
 *  - check if there is data already received in the serial port
 *
 **************************************/

#include <stdio.h>
// #include <wiringPi.h>
// #include <wiringSerial.h>
#include <pigpio.h>
#include <stdbool.h>
#include <Python.h>
#include "piSerial.h"

// bool GoGo = TRUE;
#define SERNAME "/dev/serial0"
#define SERBAUD       9600
#define SERBAUD115    115200
#define SLE_LDIS      0
#define SLE_TMPC      1
#define SER_SEL       1

/* Laser Commands */ //TODO - it might be in a chip sepecific file
/* global commands */
char read_laserParameter[4] = {0xFA, 0x06, 0x01, 0xFF};				// Read Parameter following 0xFF.
char read_laserNumber[4] = {0xFA, 0x06, 0x04, 0xFC};				// Read machine number following 0xFC.
char set_laserAddr[5] = {0xFA, 0x04, 0x01, 0x80, 0xFF};				// Set address following a address & CS. default address 0x80
char set_laserDistRevise[6] = {0xFA, 0x04, 0x06, 0x2B, 0x01, 0xFF}; // Revise distance, following symbol (0x2D or 0x2B), revised value, CS.
char set_laserInterval[5] = {0xFA, 0x04, 0x05, 0x01, 0xFF};			// Set datacontinuous interver, following intervaer value & CS.
char set_laserStart[5] = {0xFA, 0x04, 0x08, 0x00, 0xFF};			// Set distance starting and end point, following position value & CS
char set_laserRange[5] = {0xFA, 0x04, 0x09, 0x50, 0xFF};			// Set measuring range, following range value (0x05, 0x0A, 0x1E, 0x32, 0x50) & CS.
char set_laserFreq[5] = {0xFA, 0x04, 0x0A, 0x0A, 0xFF};				// Set frequency, following frequency value (0x05, 0x0A, 0x14)) & CS.
char set_laserResol[5] = {0xFA, 0x04, 0x0C, 0x01, 0xFF};			// Set resolution, following resolution value (1, 2) & CS.
char enable_laserPowerOn[5] = {0xFA, 0x04, 0x0D, 0x00, 0xFF};		// Set measurement starts when powered on (0, 1), following start flag & CS.
char measure_laserSingleB[4] = {0xFA, 0x06, 0x06, 0xFA};			// Single measurement broadcast following 0xFA.
/* module commands */
char read_laserCache[4] = {0x80, 0x06, 0x07, 0xFF};		  // Read cache following a CS. the first byte is laser's address. defaule address 0x80.
char measure_laserSingle[4] = {0x80, 0x06, 0x02, 0xFF};	  // Single measurement following a CS. the first byte is laser's address. defaule address 0x80.
char measure_laserContinu[4] = {0x80, 0x06, 0x03, 0xFF};  // Continuous measurement following a CS. the first byte is laser's address. defaule address 0x80.
char control_laserOff[5] = {0x80, 0x06, 0x05, 00, 0xFF};  // Control laser off llowing a CS. the first byte is laser's address. defaule address 0x80.
char control_laserOn[5] = {0x80, 0x06, 0x05, 01, 0xFF};	  // Control laser on llowing a CS. the first byte is laser's address. defaule address 0x80.
char control_laserShutdown[4] = {0x80, 0x04, 0x02, 0x7A}; // Shut down llowing a CS. the first byte is laser's address. defaule address 0x80.

SERCmdMsg LD_RDPARA = {
	1,
	read_laserParameter,
	4,
	1, };
SERCmdMsg LD_MACHNUM = {
	2,
	read_laserNumber,
	4,
	1, };
SERCmdMsg LD_SETADDR = {
	3,
	set_laserAddr,
	5,
	4, };
SERCmdMsg LD_SETDISREV = {
	4,
	set_laserDistRevise,
	6,
	4, };
SERCmdMsg LD_SETINTVL = {
	5,
	set_laserInterval,
	5,
	4, };
SERCmdMsg LD_SETSTAEND = {
	6,
	set_laserStart,
	5,
	4, };
SERCmdMsg LD_SETMEASRG = {
	7,
	set_laserRange,
	5,
	4, };
SERCmdMsg LD_SETFREQ = {
	8,
	set_laserFreq,
	5,
	4, };
SERCmdMsg LD_SETRESOL = {
	9,
	set_laserResol,
	5,
	4, };
SERCmdMsg LD_ENPOWON = {
	10,
	enable_laserPowerOn,
	5,
	4, };
SERCmdMsg LD_MEASSING_B = {
	11,
	measure_laserSingleB,
	4,
	0, };
SERCmdMsg LD_READCACH = {
	12,
	read_laserCache,
	4,
	11, };
SERCmdMsg LD_MEASSINGL = {
	13,
	measure_laserSingle,
	4,
	11, };
SERCmdMsg LD_MEASCONTU = {
	14,
	measure_laserContinu,
	4,
	11, };
SERCmdMsg LD_LASERON = {
	15,
	control_laserOn,
	5,
	5, };
SERCmdMsg LD_LASEROFF = {
	16,
	control_laserOff,
	5,
	5, };
SERCmdMsg LD_LASERSHTDWN = {
	17,
	control_laserShutdown,
	4,
	4, };

SERCmdMsg *pLD_CMDMSG[20] = //TODO - it might be in a chip sepecific file
	{
		&LD_RDPARA,
		&LD_MACHNUM,
		&LD_SETADDR,
		&LD_SETDISREV,
		&LD_SETINTVL,
		&LD_SETSTAEND,
		&LD_SETMEASRG,
		&LD_SETFREQ,
		&LD_SETRESOL,
		&LD_ENPOWON,
		&LD_MEASSING_B,
		&LD_READCACH,
		&LD_MEASSINGL,
		&LD_MEASCONTU,
		&LD_LASERON,
		&LD_LASEROFF,
		&LD_LASERSHTDWN};

/* serial port data */
/* pre-command processing */ //TODO - the following data types are moved to chip related files later.
#if 0
typedef struct mtdCmd_t{
	bool cmdParFlay;
	int oper;
	int value;
}mtdCmdInf;

typedef struct serCmdMsg_t{
	int cmdID;
	char* pcmd;
	int txLgth;
	int rxLgth;
	mtdCmdInf comdInf;
}SERCmdMsg;
#endif
#if 1 // the following moved to other files later TODO
/* Laser Distance Commands */
#define LASER_REDPAR_CMD      1
#define LASER_REDNUM_CMD      2
#define LASER_SETADD_CMD      3
#define LASER_SETDSR_CMD      4
#define LASER_SETINL_CMD      5
#define LASER_SETSTA_CMD      6
#define LASER_SETRNG_CMD      7
#define LASER_SETFRQ_CMD      8
#define LASER_SETRES_CMD      9
#define LASER_ENAPOW_CMD      10
#define LASER_MESIGB_CMD      11
#define LASER_REDCAH_CMD      12
#define LASER_MEASIG_CMD      13
#define LASER_MEACON_CMD      14
#define LASER_CTLOFF_CMD      16
#define LASER_CTRLON_CMD      15
#define LASER_CTLSHT_CMD      17

/* Temp Control (MTD415T) Commands */
#define MTD_VER_CMD        1
#define MTD_UUID_CMD       2
#define MTD_GERR_CMD       3
#define MTD_CERR_CMD       4
#define MTD_TECUS_CMD      5
#define MTD_TECUG_CMD      6
#define MTD_TECUT_CMD      7
#define MTD_TEVOT_CMD      8
#define MTD_TEMPS_CMD      9
#define MTD_TEMPG_CMD      10
#define MTD_CTMPG_CMD      11
#define MTD_TMPWS_CMD      12
#define MTD_TMPWG_CMD      13
#define MTD_TMPDS_CMD      14
#define MTD_TMPDG_CMD      15
#define MTD_CLPGS_CMD      16
#define MTD_CLPGG_CMD      17
#define MTD_CLPOS_CMD      18
#define MTD_CLPOG_CMD      19
#define MTD_CLPCS_CMD      20
#define MTD_CLPCG_CMD      21
#define MTD_CLPPS_CMD      22
#define MTD_CLPPG_CMD      23
#define MTD_CLPIS_CMD      24
#define MTD_CLPIG_CMD      25
#define MTD_CLPDS_CMD      26
#define MTD_CLPDG_CMD      27
#define MTD_PMSAV_CMD      28
#endif
/* UART data type, TODO - modified later */
#if 0
typedef struct portUART_t{ // UART module
	int serHandle;
	int serID;
	char * pTxbuf; // tx buffer pointer.
	char * pRxbuf; // rx buffer pointer.
	uint16_t  Txbuf_lgth;
	uint16_t  Rxbuf_lgth;
}UARTport;
#endif
// char txData[64] = {0};
// char rxData[64] = {0};

/* temperature controller commands */ //TODO - it might be in a chip sepecific file
const uint8_t MTD415_Cmds[32][20] =
{
	{"m?\n\0"},                   //reads the version of hardware and software. [MTD415T FW0.6.8]
	{"u?\n\0"},                   //read the UUID (Universal Unique Identifier). [xxx...]
	{"E?\n\0"},                   //read the error reagester
	{"c\n\0"},                    //reset the error register
	/* TEC Cmds */
	{"Lx\n\0"},                   //set the TEC current limit to x. x: 200 to 1500 [mA].
	{"L?\n\0"},                   //read the TEC current limit. [x<LF>] [mA].
	{"A?\n\0"},                   //read the actual TEC current. [x<LF>] x < 0 heating; x > 0 cooling.
	{"U?\n\0"},                   //read the actural TEC voltage. [x<LF>] [mV].
	/* Temperature Cmds */
	{"Tx\n\0"},                   //set the setting temperture to x. x: 5000 to 45000 [10^-3 C-degree].
	{"T?\n\0"},                   //read the set temperature. [x<LF>].
	{"Te?\n\0"},                  //read the actual temperature. [x<LF>].
	{"Wx\n\0"},                   //set the setting temperature window to x. x: 1 to 32000 [mK].
	{"W?\n\0"},                   //read the temperature window. [x<LF>] [mK].
	{"dx\n\0"},                   //set the delay time between reaching the temperature window and activating the State output pin to x. x: 1 to 32000 [sec]. 
	{"d?\n\0"},                   //read the delay time. [x<LF>] [sec].
	/* Control Loop Cmds */
	{"Gx\n\0"},                   //set the dritical gain to x. x: 10 to 100000 [mA/K].
	{"G?\n\0"},                   //read the critical gain. [x<LF>] [mA/K].
	{"Ox\n\0"},                   //set the critical period to x. x: 100 to 100000 [msec].
	{"O?\n\0"},                   //read the critical period. [x<LF>] [msec].
	{"Cx\n\0"},	                  //set the cycling time to x. x: 1 to 1000 [msec].
	{"C?\n\0"},                   //read the cycling time. [x<LF>] [msec].
	{"Px\n\0"},                   //set the P Share to x. x: 0 to 100000 [mA/K].
	{"P?\n\0"},                   //read the P Share. [x<LF>] [mA/K].
	{"Ix\n\0"},                   //set the I Share to x. x: 0 to 100000 [mA/(K*sec)].
	{"I?\n\0"},                   //read the I Share. [x<LF>] [mA\(K*sec)].
	{"Dx\n\0"},	                  //set the D Share to x. x: 0 to 100000 [mA*sec/K].
	{"D?\n\0"},                   //read the D Share. [x<LF>] [mA*sec/K].
	/* save settings */
	{"M\n\0"}                    //save the setup. The actual parameters that have been set using he commands T, W, L, d, G, O, P, I, D, C and S, are saved to the nonvolatile memeory.
};


char testSum(char *pTxbuf, int numbytes)
{
	char chkSum = 0;
	int i, numChk = numbytes;

	for (i = 0; i < numChk; i++)
	{
		chkSum += *(pTxbuf + i);
	}

	chkSum = ~chkSum + 1;

	return chkSum;
}

/* get sending Cmd/Msg Length */
int serSendCnt(char *string)
{
	int len=0;
	
	while(string[len]!=0)
		len++;
	return len;
}

/* new serial port routines */
/*
****************************************************************************
* uart_start() configures a serial port supported by pigpio library.  a API
* it has two parameters:
* serChan, pUart
*   serChan - the serial port selection                              *
*   pUart - a uart port type, it discrebes the uart port             *
*                                                                    *                          
**************************/
int uart_start(int serChan, UARTport *pUart)
{
	int hd = -1;
	int vsel, serName, serBaud;
   /* get UART configuration data from pUart */
   // serName = pUart->serName;
   // serBaud = pUart->serBoud;

	/* set the serial mux output */
	gpioSetMode(SER_SEL, PI_OUTPUT);
	if (serChan == SLE_TMPC)
		gpioWrite(SER_SEL, SLE_TMPC); // select temperature controler
	else
		gpioWrite(SER_SEL, SLE_LDIS); // select laser distance measuring (default)

	vsel = gpioRead(SER_SEL);

	/* open a serial port */
	if(serChan == SLE_TMPC)
		hd = serOpen(SERNAME, SERBAUD115, 0); // open serial port for temp controller
      //hd = serOpen(serName, serBaud, 0);
	else
		hd = serOpen(SERNAME, SERBAUD, 0); // open serial port for laser distance measuring
      //hd = serOpen(serName, serBaud, 0);

	if (hd < 0)
		return hd; // open serial port failed.

	pUart->serHandle = hd;
	pUart->serID = serChan;
	pUart->Txbuf_lgth = 0;
	pUart->Rxbuf_lgth = 0;

	/* return the hd */
	return hd;
}

/*
*******************************************************************
* uart_Close() close a serial port that is opend previously. a API
* The routine is supported by pigpio library, it has two parameters:
* serID, pUart
*   serChan - the serial port selection                              *
*   pUart - a uart port type, it discrebes the uart port             *
*                                                                    *                          
**************************/
int uart_close(int SERHandle, UARTport *pUart)
{
	int hd, ok = -1;
	hd = SERHandle;

	/* close a serial port with SERHandle */
	ok = serClose(hd);

	/* clear uart */
	pUart->serHandle = -1;
	pUart->serID = -1;
	pUart->pTxbuf = NULL;
	pUart->pRxbuf = NULL;
	pUart->Txbuf_lgth = 0;
	pUart->Rxbuf_lgth = 0;

	/* return the close ok value */
	return ok;
}

/*
*******************************************************************
* uart_cmdMsg() send/receive command ro message. a API
* The routine is supported by pigpio library, it has three parameters:
* pUart, pcdmg, CmdID
*   pUart - a uart port type, it discrebes the uart port             *
*   pcdms - the contents of the command or message                   *
*   CmdID - the command or message ID                                *
*                                                                    *                          
**************************/
int uart_cmdmsg(UARTport *pUart, SERCmdMsg *pcdmg, int CmdID)
{
	int numofbytes = 0, sendNo;
	//SERCmdMsg *lpcdmg = pcdmg;

	if(pUart->serID == SLE_TMPC)/* if LDIS */
	{
		pcdmg->cmdID = CmdID;
		pcdmg->pcmd = (char *)&MTD415_Cmds[CmdID - 1][0];
		pcdmg->txLgth = serSendCnt(pcdmg->pcmd);
		pcdmg->rxLgth = 2;
		sendNo = pcdmg->txLgth;
	}
	else{
		pcdmg = pLD_CMDMSG[CmdID - 1];
		sendNo = pcdmg->txLgth;
		//printf("txlg: %d; numSend %d.\n", pcdmg->txLgth, sendNo);
	}

	// check lpcdmg contents. debug data
	//printf("uartID: %d; CmdID1: %d - %d; Count %d; SendLgt %d - %d.\n", pUart->serID, CmdID, pcdmg->cmdID, pcdmg->rxLgth, sendNo, pcdmg->txLgth);
	//printf("cmdID: %d;   %d; Count %d; SendLgt %d.\n", CmdID, pcdmg->cmdID, pcdmg->rxLgth, sendNo);

	//for (int i = 0; i < sendNo; i++)
	//{
	//	printf("Cmd: 0x%x ", *((pcdmg->pcmd) + i));
	//}
	//printf("0x%x\n", pcdmg->pcmd);

	/* prepare the data for sending */
	uart_preExch(pUart, pcdmg);

	numofbytes = uart_ExchData(pUart, pcdmg);

	return numofbytes;
}

/*
*******************************************************************
* uart_preExch() prepare the content of the command/message before data exchange.
* The routine is supported by pigpio library, it has two parameters:
* pUart, cmdMsg
*   pUart - a uart port type, it discrebes the uart port             *
*   cmdMsg - the command/message for exchange                              *
*                                                                    *                          
**************************/
void uart_preExch(UARTport *pUart, SERCmdMsg *cmdMsg)
{
	unsigned char sumChk;
	char *lpTxbuf = pUart->pTxbuf;
	uint16_t lnumbytes = cmdMsg->txLgth - 1;
	int i;

	/* prepare uart */
	printf("tx length %d \n", lnumbytes + 1);

	for (i = 0; i < lnumbytes; i++)
	{
		*(lpTxbuf + i) = *(cmdMsg->pcmd + i);
		//printf("0x%x - 0x%x; ", *(lpTxbuf + i), *((cmg->pcmd) + i));
	}

	/* calculate checksum */
	if(pUart->serID == SLE_LDIS)
	{
		sumChk = testSum(lpTxbuf, lnumbytes);
		*(lpTxbuf + lnumbytes) = sumChk;
	}else// if(uart->serID == SLE_LDIS)
	{
		*(lpTxbuf + lnumbytes) = *(cmdMsg->pcmd + lnumbytes);
	}
	//sumChk = testSum(lpTxbuf, lnumbytes);
	//*(lpTxbuf + lnumbytes) = sumChk;

	pUart->Txbuf_lgth = cmdMsg->txLgth;
	pUart->Rxbuf_lgth = cmdMsg->rxLgth;

	//printf("pre data: sum - 0x%x txlg - %d rxlg - %d. \n", sumChk, pUart->Txbuf_lgth, pUart->Rxbuf_lgth);//debug data
	return;
}

/*
*******************************************************************
* uart_dataExch() exchanges the content of the command/message via UART.
* The routine is supported by pigpio library, it has two parameters:
* pUart, cmdMsg
*   pUart - a uart port type, it discrebes the uart port             *
*   pcmdMsg - the command/message for exchange                       *
*                                                                    *                          
*********************************************************/
int uart_ExchData(UARTport *pUart, SERCmdMsg *pcmdMsg)
{
	const int maxTry = 300;
	int OK = false;
	int hd, counter = 0, numBytes = 0, rxNum = 0, n;
	char dumBuf[32] = {0};
	char *lprxbuf;
	uint16_t ltxlth, lrxlth;

	hd = pUart->serHandle;
	ltxlth = pUart->Txbuf_lgth;
	lrxlth = pUart->Rxbuf_lgth;

	/* Debug code */
	//printf("TX Data0: \n");
	//for (n = 0; n < pcmdMsg->txLgth; n++)
	//{
	//	printf("0x%x - 0x%x ", *(uart->pTxbuf + n), *(pcmdMsg->pcmd + n));
	//}

	//printf(" No.Rx %d pcmdMsg 0x%x.\n\n", lrxlth, pcmdMsg->pcmd);

	/* Debug code */
	//printf("\nTX Data: ");
	//for (n = 0; n < ltxlth; n++)
	//{
	//	printf("0x%x ", *(pUart->pTxbuf + n));
	//}

	//printf(".\n\n");

	//return n;
	/* end of Debug */

	/* flush the receiver buffer */
	numBytes = uart_dataRdy(hd);
	while (1)
	{
		lprxbuf = pUart->pRxbuf;
		// lrxlth = pUart->Rxbuf_lgth;
		pUart->pRxbuf = dumBuf;
		pUart->Rxbuf_lgth = numBytes;
		uart_read(pUart, numBytes);
		
		/* Debug code */
		//printf("dump Data: ");
		//for (n = 0; n < numBytes; n++)
		//{
		//	printf("0x%x ", *(pUart->pRxbuf + n));
		//}
		
		//printf(".\n\n");
		/* end of Debug code */

		if(numBytes = uart_dataRdy(hd) == 0)
		{
			pUart->pRxbuf = lprxbuf;
			break;
		}
		// uart->Rxbuf_lgth = lrxlth;
	}

	/* send data */
	uart_write(pUart);
	gpioDelay(2000);

	numBytes = uart_dataRdy(hd);
	do
	{
		if (numBytes >= lrxlth)
		{
			rxNum += uart_read(pUart, numBytes);
			
			//printf("rx-%d\n", rxNum); //debug data
			gpioDelay(10000);
			numBytes = uart_dataRdy(hd);
			if(numBytes > 0)
			{
				continue;
			}

			break;
		}
		else
		{
			gpioDelay(10000);
			counter++;
			numBytes = uart_dataRdy(hd);
			//printf("count-%d, No.-%d, %d\n", counter, numBytes, lrxlth);
		}
	} while (/*(numBytes <= lrxlth) &&*/ (counter < maxTry));

		pUart->Rxbuf_lgth = rxNum;
		
      /* Debug code */
		//printf("Received Data %d ,%d: ", counter, numBytes);
		//for (n = 0; n < rxNum; n++)
		//{
		//	printf("0x%x, 0x%x ", *(pUart->pRxbuf + n), *(pUart->pTxbuf + n));
		//}
		
		//printf(".\n\n");
		/* end of Debug code */
		
		/* process received data */
	//tspi_serPosExch(uart, pcdmg);

	return rxNum;
}

/*
*******************************************************************
* uart_write() writes command/message to UART.
* The routine is supported by pigpio library, it has a parameters:
* pUart
*   pUart - a uart port type, it discrebes the uart port             *
*                                                                    *                          
*********************************************************/
int uart_write(UARTport *pUart)
{
	int sendCnt = 0;
	int hd = pUart->serHandle;

	char *lptxbuf = pUart->pTxbuf;
	uint16_t lcnt = pUart->Txbuf_lgth;

	/* send data via UART */
	sendCnt = serWrite(hd, lptxbuf, lcnt);

	/* return number of byte sended */
	return sendCnt;
}

/*
*******************************************************************
* uart_read() reads command/message to UART.
* The routine is supported by pigpio library, it has two parameters:
* pUart, byteCnt
*   pUart - a uart port type, it discrebes the uart port             *
*   byteCnt - the number of the data to be read                      *
*                                                                    *                          
*********************************************************/
int uart_read(UARTport *pUart, int byteCnt)
{
	int hd, receiveCnt;
	int numbyte;

	char *lprxbuf = pUart->pRxbuf;

	hd = pUart->serHandle;
	// numbyte = byteCnt;

	/* check the data ready to read */
	numbyte = uart_dataRdy(hd); // for debug. then rm into calling routine.

	if (numbyte > byteCnt)
	{
		printf("get more data: %d %d\n", numbyte, byteCnt);
	}
	else
		numbyte = byteCnt;

	// if(numbyte >= lreqnm)
	{
		/* read data */
		receiveCnt = serRead(hd, pUart->pRxbuf, numbyte);
	}
	// else{
	//	receiveCnt = 0;
	// }

	/* return number of byte received */
	return receiveCnt;
}

/*
*******************************************************************
* uart_dataRdy() indicates how many byte data is ready to be read via UART.
* The routine is supported by pigpio library, it has parameters:
* SERHandle
*   SERHandle - a uart port handle that is created at uart being opened     *
*                                                                           *                          
*********************************************************/
int uart_dataRdy(int SERHandle)
{
	int numbyte = 0;

	/* check available data on uart */
	numbyte = serDataAvailable(SERHandle);

	/* return number of available data */
	return numbyte;
}


#if 0 //TODO - open the code later
int tspi_serPosExch(UARTport *uart, SERCmdMsg *cmg)
{
	bool okFlag = true;
	float dist = 0;
	char result = 0;
	int cmdId = cmg->cmdID;

	switch (cmdId)
	{
	case LASER_REDPAR_CMD:
		uart->pRxbuf[3]; // address.
		uart->pRxbuf[4]; // light
		uart->pRxbuf[5]; //???
		uart->pRxbuf[6]; // temperature
		break;
	case LASER_REDNUM_CMD:
		// cmdResult->laserAddr = RxBuffer[3];
		// cmdResult->light = RxBuffer[4];
		// cmdResult->returned = RxBuffer[5];
		// cmdResult->temperature = RxBuffer[6];
		break;
	case LASER_SETADD_CMD:
		result = uart->pRxbuf[1];
		if (result == 0x04)
			okFlag = false;
		break;
	case LASER_SETDSR_CMD:
		result = uart->pRxbuf[1];
		if (result == 0x04)
			okFlag = false;
		break;
	case LASER_SETINL_CMD:
		result = uart->pRxbuf[1];
		if (result == 0x04)
			okFlag = false;
		break;
	case LASER_SETSTA_CMD:
		result = uart->pRxbuf[1];
		if (result == 0x04)
			okFlag = false;
		break;
	case LASER_SETRNG_CMD:
		result = uart->pRxbuf[1];
		if (result == 0x04)
			okFlag = false;
		break;
	case LASER_SETFRQ_CMD:
		result = uart->pRxbuf[1];
		if (result == 0x04)
			okFlag = false;
		break;
	case LASER_SETRES_CMD:
		result = uart->pRxbuf[1];
		if (result == 0x04)
			okFlag = false;
		break;
	case LASER_ENAPOW_CMD:
		result = uart->pRxbuf[1];
		if (result == 0x04)
			okFlag = false;
		break;
	case LASER_MESIGB_CMD:
		break;
	case LASER_REDCAH_CMD:
	case LASER_MEASIG_CMD:
	case LASER_MEACON_CMD:
		result = uart->pRxbuf[3];
		if (result == 0x45)
		{
			result = uart->pRxbuf[4];
			if (result == 0x52)
				okFlag = false;
			dist = 0;
		}
		else
		{
			dist = (uart->pRxbuf[3] - 0x30) * 100 + (uart->pRxbuf[4] - 0x30) * 10 +
				   (uart->pRxbuf[5] - 0x30) * 1 + (uart->pRxbuf[7] - 0x30) * 0.1 +
				   (uart->pRxbuf[8] - 0x30) * 0.01 + (uart->pRxbuf[9] - 0x30) * 0.001;
		}
		// cmdResult->distance = dist;
		break;
	case LASER_CTLOFF_CMD:
	case LASER_CTRLON_CMD:
		result = uart->pRxbuf[3];
		if (result == 0x0)
			okFlag = false;
		break;
	case LASER_CTLSHT_CMD:
		break;
	}
	return okFlag;
}


//mtdCmdInf comdInf;

void preSerCmd(mtdCmdInf* comdInf)
{
	int oper, value;
	
	printf("    Input Operation: ");  // 0 - set; 1 - get
	scanf("%d", &oper);
		
	printf("    Input set value: ");  // 0 - set; 1 - get
	scanf("%d", &value);
	
	comdInf->oper = oper;
	comdInf->value = value;
	comdInf->cmdParFlay = true;
	
	return;
}

/*******************************************************
 * It calls a python rouitne to set the temperature controller.
 * - Parameters:
 *   argc: number of parameters
 *   argv1: module name
 *   argv2: function name
 *   argv3: parameter1
 *   argv4: parameter2
 *   ......
 * TODO - move to measure_Utility.c
 * *****************************************************/
// cresult = call_Python_Stitch(6, "Image_Stitching", "main", "Images", "output.jpeg","--images","--output");
				
void tempCtrll_py(int argc, char *argv1, char *argv2, char *argv3)
{
	PyObject *pName, *pModule, *pDict, *pFunc, *pValue, *pmyresult, *args, *kwargs;
	int i;

	// Set PYTHONPATH TO working directory used for GPS parser
	setenv("PYTHONPATH", "/home/pi/gpsPy:/home/pi/nmea_parser-master:/home/pi/nmea_parser-master/nmea:/home/pi/nmea_parser-master/nmea/core", 1);
	//printf("PATH: %s\n", getenv("PATH"));
	//printf("PYTHONPATH: %s\n", getenv("PYTHONPATH"));
	printf("in the ctrl_py(%d):\n   %s\n   %s\n   %s\n", argc, argv1, argv2, argv3);
	//return;

	wchar_t *program = Py_DecodeLocale(argv1, NULL);
	if (program == NULL)
	{
		fprintf(stderr, "Fatal error: cannot decode argv[0]\n");
		exit(1);
	}
	Py_SetProgramName(program); /* optional but recommended */
	// Initialize the Python Interpreter
	Py_Initialize();
	//PySys_SetPath("/home/pi/nmea_parser-master");
	//printf("PATH: %s\n", getenv("PATH"));
	//printf("PYTHONPATH: %s\n", getenv("PYTHONPATH"));
	
	// Build the name object
	pName = PyUnicode_DecodeFSDefault(argv1);
	//pName = PyUnicode_FromString("nmeaParser");

	// Load the module object
	pModule = PyImport_Import(pName);
	if(pModule == NULL)
	{
		fprintf(stderr, "Fatal error: cannot load the module\n");
		exit(1);
	}

	// pDict is a borrowed reference
	pDict = PyModule_GetDict(pModule);
	if(pDict == NULL)
	{
		fprintf(stderr, "Fatal error: cannot get a Dict\n");
		exit(1);
	}
	// pFunc is also a borrowed reference
	pFunc = PyDict_GetItemString(pDict, argv2);
	if(pFunc == NULL)
	{
		fprintf(stderr, "Fatal error: cannot get a function\n");
		exit(1);
	}
	args = PyTuple_Pack(1,PyUnicode_DecodeFSDefault(argv3));
	// kwargs = PyTuple_Pack(2,PyUnicode_DecodeFSDefault(argv5), PyUnicode_DecodeFSDefault(argv6));
	// args = Py_BuildValue("ssss", argv5, argv3, argv6, argv4);
	// kwargs = Py_BuildValue("ss", argv5, argv6);

	if (PyCallable_Check(pFunc))
	{
		pmyresult = PyObject_CallObject(pFunc, args/*NULL*/);
		i = 0;
	}
	else
	{
		PyErr_Print();
		i = 1;
		return;

	}
	PyUnicode_CheckExact(pmyresult);
	//printf("in the ctrl_py\n");

	if (PyUnicode_Check(pmyresult))
	{
		//clrscr();
		PyObject *temp_bytes = PyUnicode_AsEncodedString(pmyresult, "UTF-8", "strict"); // Owned reference
		if (temp_bytes != NULL)
		{
			char *resultStr = PyBytes_AS_STRING(temp_bytes); // Borrowed pointer
			resultStr = strdup(resultStr);
			Py_DECREF(temp_bytes);
			printf(resultStr);
			/* split string resultStr by "," */
			//char *p = strtok(resultStr, ", ");
    		//while(p)
    		//{
        	//	printf("%s \n", p); //print newline
        	//	p = strtok(NULL, ", ");
    		//}
		}
		else
		{
			printf("in the ctrl_py\n");
			return;
			// TODO: Handle encoding error.
		}
	}
	else{
		printf("no string return\n");
	}

	// Clean up
	Py_DECREF(pModule);
	Py_DECREF(pName);

	// Finish the Python Interpreter
	Py_Finalize();

	return;
}

/* the pigpio processing routines, some of them wrap the pigpio APIs */
// pigpio start
int start_pigpio()
{
   int status;

   status = gpioInitialise();

   return status;
}

// pigpio end
void end_pigpio()
{
   gpioTerminate();
}

/* routine for SPI */
/* TODO - rename to spi_pigpio_init() */

/*
*******************************************************************
* spiOpen_pigpio() wraps the pigpio API, spiopen(). it has three parameters:
* spiChan, baud, and spiFlags *
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
int spiOpen_pigpio(uint16_t chan, uint32_t baud, uint32_t spiFlags)
{
   int spiId;
   
   spiId = spiOpen(chan, baud, spiFlags);

   return spiId;
}

/*
*******************************************************************
* spiClose_pigpio() wraps the pigpio API, spiClose(). it has a parameters:
* spiId
*   spiId - the ID that was returned at calling the spi open routine  *
* 
**********************************************************************/
int spiClose_pigpio(uint16_t spiId)
{
   int status;
   
   status = spiClose(spiId);

   return status;
}


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
