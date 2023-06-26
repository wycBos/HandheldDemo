/* the UART header included in gpioTest file */
/*******************************************************************************
 * The file name will be changed to measurer_Utility.h as the measurer_Utility's header file.
 * It will support HandMeasurer.c functions, as well as main file, HandMeasurer.c
 * 
 * ***********************************
 * 
 * *****/
#ifndef MEASURUTLIT_H
#define MEASURUTLIT_H

#include "ADS1x15.h"

#define MAXRTIOD  (0.003)
#define ADJSTEP   (0.002)

typedef struct SensorCalib_t
{
	float dacCh0;
    float avgRatio;
	//uint32_t u32tickDbg[32];
	//int inumData[32];
	//float fRatio[32];
	//float f1[32];
	//float f2[32];
} senCali;

typedef struct CalibDataCap_t
{
	//float dacCh0;
    //float avgRatio;
	uint32_t u32tickDbg[32];
	int inumData[32];
	float fRatio[32];
	float f1[32];
	float f2[32];
} dataCap;

/* gas measurement data */
typedef struct tecParms_t
{
	int curState;
	float tempPoint;
	float currenLimit;
	float p_gain;
	float i_gain;
	float g_gain;
} tecSettings;

typedef struct DACParms_t
{
	float voltCh0;
	float voltCh1;
} dacSettings;

typedef struct ADCParms_t
{
	int curState;
	//float preRatio; //TODO - it may be added later for daily calib.
	float conMod;
	void *pSettings;
} adcSettings;

/* sharing data */
typedef struct measData_t
{
	bool startFlag;
	int dailyJust;
	int ydays;	   // count
	float gas_ppm; // TODO - present ratio right now. it'll be changed later.
	float cur_temp;
	float ADVoltag;
	float dist;
	int wid;       // the waveform generator handle ID.
	tecSettings tecSets;
	dacSettings dacSets;
	adcSettings adcSets;
} measData;

//float UART_main();
float UART_distMain(int isConti);
float getGasConcentr();
int gasMeasStart();
int gasMeasClose();
bool gasMeasJust(senCali *pdailyCal, senCali *ppredailyCal);
float getLSRatio(userData* pfuncData);

int getSets(const char *filename, measData *pDataSet);
int saveSets(const char *filename, measData *pDataSet);

/* mtd415 APIs TODO - if it's moved to HandMeasurer.h */
extern char *mtd415;
extern char *mtd415setTempPoint;
extern char *mtd415getErrors;
extern char *mtd415setCurrentlimt;
extern char *mtd415setPGain;
extern char *mtd415setDGain;
extern char *mtd415setIGain;

extern char *mtd415getTemperture;
extern char *mtd415getErrors;
extern char *mtd415getCurrent;
extern char *mtd41paraSave;

const char *tempCtrll_py(int argc, char *argv1, char *argv2, char *argv3);

#endif // MEASURUTLIT_H
