/* the UART header included in gpioTest file */
/*******************************************************************************
 * The file name will be changed to measurer_Utility.h as the measurer_Utility's header file.
 * It will support HandMeasurer.c functions, as well as main file, HandMeasurer.c
 * 
 * *
 * 
 * *****/
#include "ADS1x15.h"

//float UART_main();
float UART_distMain(int isConti);
float getGasConcentr();
int gasMeasStart();
float getLSRatio(userData* pfuncData);