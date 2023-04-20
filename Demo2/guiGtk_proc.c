/**************************************
 * the pigpio application to generate two wave
 * code protype comes from pigpio example
 * it comes from waveForm.h.
 * TODO -
 *   wavepies() is moved into measurer_utility.c and wrapped with wave form relate pigpio routines.
 * *****************/

#include <stdio.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <pigpio.h>
#include "waveForm.h"

#if 0
int gpios[]={5,6,13,12,};
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
   {0x1060, 0x0000, 5}, //1
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
   {0x0000, 0x0020, 5}, 
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

   {0x0020, 0x1000, 5}, //6
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
   {0x0000, 0x0020, 5}, 
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

   {0x1020, 0x0040, 5}, //11
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
   {0x0000, 0x0020, 5}, 
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

   {0x0020, 0x1000, 5}, //16
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
   {0x0000, 0x0020, 5}, 
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
//#else
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
//char contimeas[4]   ={0x80,0x06,0x03,0x77};

//remove the wavePiset()... they are moved into gpio_proce.c


/* add guiGtk routines start here TODO */
void btn_init()
{
   return;
}

void btn_event() //TODO - device state handler
{
   return;
}

void set_GUI() //TODO - GUI features operations
{
   return;
}

void run_video() //TOTO - run stream pipeline
{
   return;
}

void cap_image() //TOTO - run image capture
{
   return;
}