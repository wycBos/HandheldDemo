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
int gasMeasClose();
float getLSRatio(userData* pfuncData);

/* mtd415 APIs TODO - if it's moved to HandMeasurer.h */
extern char *mtd415;
extern char *mtd415setTempPoint;
extern char *mtd415setCurrentlimt;
extern char *mtd415setPGain;
extern char *mtd415setDGain;
extern char *mtd415setIGain;

extern char *mtd415getTemperture;
extern char *mtd415getCurrent;
extern char *mtd41paraSave;

const char *tempCtrll_py(int argc, char *argv1, char *argv2, char *argv3);