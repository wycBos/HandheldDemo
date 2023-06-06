/******************************************
 * the file is modified from UART_test.c and
 * piPeriphTest.c, it includes SPI, UART, etc.
 *
 * *****************************************/

#include <stdio.h>
#include <wiringPi.h>
//#include <wiringSerial.h>
#include <stdbool.h>
#include "UART_test.h"
#include "piSerial.h"
#include "ADS1x15.h"

#include <pigpio.h>

#include <Python.h>

/* remove the wiringSerial function call */
/*
void serialread(int fd, int numbytes)
{
    unsigned char data[11] = {0};
    for (int i = 0; i < numbytes; i++)
    {
        data[i] = serialGetchar(fd);
        printf("%x ", data[i]);
    }
    printf("\n");
    unsigned char Check = 0;
    for (int i = 0; i < numbytes - 1; i++)
    {
        Check = Check + data[i];
    }
    Check = ~Check + 1;
    printf("%x \n", Check);
    if (data[numbytes - 1] == Check)
    {
        printf("CheckSum ok!\n");
    }
    else
    {
        printf("Invalid Data!\n");
    }
}
*/

/* new code starts here */
/******************************
 * it includes:
 *  - laser distance measuring, uart-laserDistance (is gpio_proc.c)
 *  - ADS131 input capture, ADC-ads131m04
 *  - gas concentration output, gasMeasStart();getGasCentr();getLSRatio()
 *  - temperature controller set, uart-tempCtrl (in gpio_proc.c)
 *    ......
 *
 * ****************************/

/*******************************************************
 * UART_distMain communicates to a laser distance sensor for the distance measuring.
 * - Parameters:
 *   isConti: indicate the continue or single measuring.
 *   ......
 *
 * *****************************************************/
float UART_distMain(int isConti)
{
    int hd, reps, addr;
    int value;
    int numbytes;
    char command[16];
    UARTport muart;
    SERCmdMsg cmdmsg;

    char txbuf[32] = {0}, rxbuf[32] = {0};

    float distance = 0;

    /* start UART */
    // hd = uart_start(SLE_LDIS, &muart);
    hd = uart_start(0, &muart);
    if (hd < 0)
    {
        printf("start uart failed.\n");
        return 0;
    }

    /* set tx/rx buffers */
    muart.pTxbuf = &txbuf[0];
    muart.pRxbuf = &rxbuf[0];

    /* Execute Dis_Laser single measurement operations */

    // execute code single measuring
    // printf("Execute %s.\n", command);
    reps = uart_cmdmsg(&muart, &cmdmsg, LASER_MEASIG_CMD);

    distance = 0;
    distance = (rxbuf[3] - 0x30) * 100 + (rxbuf[4] - 0x30) * 10 + (rxbuf[5] - 0x30) * 1 + (rxbuf[7] - 0x30) * 0.1 + (rxbuf[8] - 0x30) * 0.01 + (rxbuf[9] - 0x30) * 0.001;

    //printf("Distance = ");
    //printf("%5.1f", distance);
    //printf(" m\n");

    // execute code shutdown;
    // printf("Execute %s.\n", command);
    reps = uart_cmdmsg(&muart, &cmdmsg, LASER_CTLSHT_CMD);

    uart_close(hd, &muart); // serialClose(hd);
    //printf("LDistance Sensor UART Closed. \n");
    return distance;
}

/*****************************************************************
 * gas concentration measurement routintes:
 *  - getGasConcentr() - it calculates the gas concentration based on the ratio & distance.
 *  - gasMeasStart() - configure related components for gas measuring.
 *  - getLSRatio() - get gas laser data to calculate two frequency data ratio.
 *    ......
 *
 * **************************
 * 
 **/
float getGasConcentr()
{

}

int gasMeasStart() //for ADS131
{
   int hd = -1;
   regInfor *pregInf = &regSetInf;
   adc_channel_data *padcData = &adcData; //DONE
   userData* pfuncData = &adcCapFuncData;
   //userData* pfuncDataB = &adcCapFuncDataB;
   
   /* start the ads131 chip */
   hd = tspi_ads131m04_start(pregInf, padcData);

   /* set capture function to ads131 */
   pfuncData->pRslts = &adcRltData[0];
   pfuncData->handle = hd;
   pfuncData->datIdx = 0;
   pfuncData->isRun = 0;
   //gpioSetAlertFuncEx(ADC_DRDY, NULL, pfuncData);
   gpioSetISRFuncEx(ADC_DRDY, 1, 10, NULL, pfuncData); //TODO - remove it later?!

   return hd;
}

int gasMeasClose() //for ADS131
{
   int ret, hd;
   regInfor *pregInf = &regSetInf;
   adc_channel_data *padcData = &adcData; //DONE
   userData* pfuncData = &adcCapFuncData;
   
   /* close the ads131 chip */
   //gpioSetAlertFuncEx(ADC_DRDY, NULL, pfuncData);
   gpioSetISRFuncEx(ADC_DRDY, 1, 10, NULL, pfuncData);
   pfuncData->datIdx = 0;
   pfuncData->isRun = 0;
   ret = tspi_ads131m04_close(hd);
   pfuncData->handle = -1;
   
   return ret;
}

/*******************************************************
 * It justs the current to supply gas laser every day and only one time a day.
 * Before it's called the followings should be done:
 *  - Tec is tured on;
 *  - DAC_CH0 is not set;
 * 
 * The arguments is 
 *  - DAC_CH0 setting value, dac0Volt;
 *  - signals' Ratio, sigRatio; -TODO it should be ppm value?!
 * 
 * Return -1 if no change, otherwise return 0 or > 0.
 *   ......
 * 
 * *****************************************************/
//int gasMeasJusted(float dac0Volt, float conMod)
bool gasMeasJust(senCali dailyCal, senCali predailyCal)
{
   bool retFlag = false;
   float preVol, curVol, preRatio, curRatio;

   preVol = curVol = dailyCal.dacCh0;
   curRatio = dailyCal.avgRatio; preRatio = predailyCal.avgRatio; 
   if(fabs(preRatio - curRatio) > MAXRTIOD) //TODO - set ratio throuheld constant, MAXRTIOD
   {
      if(curRatio > preRatio)
      {
         predailyCal.avgRatio = curRatio; //preVol = curVol;
         dailyCal.dacCh0 = curVol = preVol + ADJSTEP; // TODO - adjust step constant, ADJSTEP
         //tspi_mcp4822(0, 2, curVol, preVol); // set DAC CH0 value;
      }
      else{
         // keep pre-ratio
         dailyCal.dacCh0 = curVol = preVol - ADJSTEP;
         //tspi_mcp4822(0, 2, curVol, preVol);
      }
      tspi_mcp4822(0, 2, curVol, preVol);
   }else
   {
      retFlag = true;
   }

   predailyCal.dacCh0 = preVol; // TODO - it may not be used, but can be used for debugging
   dailyCal.dacCh0 = curVol;
   
   return retFlag;
}

/* old ratio function */
// data used in the getLSRatio, DONE - moved to UART_test.h
float getLSRatio(userData* pfuncData) //TODO - place userData by measThr.
{
   int count = 0;
   float tecRet = 0;
   char *presult;
   manuCst cnstRlts;
   //adcRslts *lpdata = pfuncData->presultOne->padcData;

   //(lpdata + 1)->tick;
   //((pfuncData->presultOne->padcData) + 1)->tick;
   
   uint32_t curTick, preTick;
   uint32_t preDatTick = 0;
   int ratioIdx = 0, capIdx = 0;
   float avgRatio;

   /* while loop for checking temp and adc each 100 ms */ //to calulate ratio
   preTick = curTick = gpioTick(); 
   {
         /* calculate data */
         int32_t fx1 = 0,fy1 = 0, fx2 = 0, fy2 = 0, maxDtick = 0, curTick;
         double df1, df2, v1, v2, v3, v4, step;
         fx1 = pfuncData->avgData[0]; fy1 = pfuncData->avgData[1];
         fx2 = pfuncData->avgData[2]; fy2 = pfuncData->avgData[3];
         
         /* convert to voltage */
         step = 1200000.0 / 8388607.0;
         v1 = (double)fx1; v1 *= step;
         v2 = (double)fy1; v2 *= step;
         v3 = (double)fx2; v3 *= step;
         v4 = (double)fy2; v4 *= step;
        
         df1 = v1*v1 + v2*v2;
         df2 = v3*v3 + v4*v4;

         df1 = sqrt(df1);
         df2 = sqrt(df2);

         if(df2 != 0.0)
         {
            avgRatio = df2/df1;;

            //printf("     lasting %d max-dalt %d; result %.4f, %.4f and ratio %.4f\n\r", (curTick - preTick), maxDtick, df1, df2, df1/df2);
            //printf("     result %.4f, %.4f and ratio %.4f\n\r", df1, df2, df1/df2);
         }
         else
         {
            avgRatio = 0;
            printf("    dF2 is zero. %.4f, %.4f", df1, df2);
         }
         //printf("    (%d): %.02f, %.02f, %.2f, %.2f\n\r", count, v1, v2, v3, v4);
      
         /* renew the data buffer */
         pfuncData->datIdx = 0;
         //pfuncData->isRun = 1;
         curTick = preTick = gpioTick();
         //count++;
      
      
         //}

         /* end of convert */
         
         /* output data */
         //printf("\r    Time lasting in Samples: %d - %d -- %d\n\r", (pfuncData->pRslts + 2)->tick,
         //(pfuncData->pRslts + 21)->tick,
         //(pfuncData->pRslts + 21)->tick - (pfuncData->pRslts + 2)->tick);

         //curTick = gpioTick();
   }

   /* save data in a file */
   // save date and time in file

   // save current constant in file TODO save result into a file
   //fprintf(fh, "Gas Concentration: %.4f, Constant: %.4f, AvgRatio: %.4f\n", samplePercent, cnstRlt, avgRatio);

   return avgRatio;
}

/**********************************************************
 * The file operations for get/set the calibaraion data.
 * - getSets(const char *filename, mData *pDataSets) no years data!
 *    return file open/close state
 * - saveSets(const char *filename, mData *pDataSets)
 *     return file open/close state
 * 
 * ***************************************************
 * 
 */
int getSets(const char *filename, measData *pDataSet)
{
	FILE *fh;
   int fret;
   char lineData[64], parm0[16], parm1[16], parm2[16], parm3[16], parm4[16];
   char *retC;
	int lydays, ret, retI, outN;
	//float ltempPoint, lDAch0, lConMod;
   measData *lpmData = pDataSet;
  
   fh = fopen(filename, "r"); //"./settingsData.txt"
	
	if(fh == NULL)
	{
		printf("open file failure!\n");
		//exit(EXIT_FAILURE);
      return -1;
	}

	printf("    reading a file\n");
				
	retC = fgets(lineData, 64, fh);
	printf("    %s, (%d), %s.\n", lineData, outN, retC);
	outN++;

	while(outN < 32){
		//retI = fscanf(fh, "%s %s %s", parm1, parm2, parm3);
		retI = fscanf(fh, "%s %s %s %s %s", parm0, parm1, parm2, parm3, parm4);
		outN++;

		//printf("%s, (%d), %d.\n", parm1, outN, retI); // debug info

		if(retI < 0)
			break;
	}

	fret = fclose(fh);

	/* get date and parameters */
	printf("last line(%d): %s, %s, %s, %s, %s, %i\n", outN, parm0, parm1, parm2, parm3, parm4, retI);

	//ltempPoint = atof(parm2);
	lpmData->tecSets.tempPoint = atof(parm2);

	//lDAch0 = atof(parm3);
	lpmData->dacSets.voltCh0 = atof(parm3);

	//lydays = atoi(parm1);// keep ydays record
	lpmData->dailyJust = atoi(parm1);
				
	//lConMod = atoi(parm4);
	lpmData->adcSets.conMod = atof(parm4);

	printf("(i)temp-point: %.2f, DACH0: %.2f, ConMod: %.3f\n", lpmData->tecSets.tempPoint, lpmData->dacSets.voltCh0, lpmData->adcSets.conMod);
   return fret;
}

int saveSets(const char *filename, measData *pDataSet)
{
   FILE *fh;
   int fret;
   char lineData[64];
   //char *retC;
	//int lydays, ret, retI, outN;
	//float ltempPoint, lDAch0, lConMod;
   measData *lpmData = pDataSet;
  
   fh = fopen(filename, "ab"); //"./settingsData.txt"
	
	if(fh == NULL)
	{
		printf("open file failure!\n");
		//exit(EXIT_FAILURE);
      return -1;
	}

	printf("    writing a file\n");

	sprintf(lineData,"%d, %d, %.2f, %.2f, %.2f",
      23/*lpmData->year*/, lpmData->ydays, lpmData->tecSets.tempPoint, 
      lpmData->dacSets.voltCh0, lpmData->adcSets.conMod);

   printf(lineData); printf("\n");
   fprintf(fh, "%s\n", lineData);
   fret = fclose(fh);

   return fret;
}

char *mtd415 = "TecOps";
char *mtd415setTempPoint = "setTempPoint";
char *mtd415setCurrentlimt = "setCurrentLimit";
char *mtd415setPGain = "setPGain";
char *mtd415setDGain = "setDGain";
char *mtd415setIGain = "setIGain";

char *mtd415getTemperture = "getTempture";
char *mtd415getCurrent = "getCurrent";
char *mtd41paraSave = "paraSave";

/*******************************************************
 * It calls a python rouitne to set the temperature controller.
 * - Parameters:
 *   argc: number of parameters
 *   argv1: module name
 *   argv2: function name
 *   argv3: parameter1
 *   argv4: parameter2
 *   ......
 * 
 * *****************************************************/
// cresult = call_Python_Stitch(6, "Image_Stitching", "main", "Images", "output.jpeg","--images","--output");
				
const char *tempCtrll_py(int argc, char *argv1, char *argv2, char *argv3)
{
   //char resultStr[32], *presultStr = &resultStr[0];
#if 0
	PyObject *pName, *pModule, *pDict, *pFunc, *pValue, *pmyresult, *args, *kwargs;
	int i;
	char resultStr[32], *presultStr = &resultStr[0];

    //gpioSetMode(SER_SEL, PI_OUTPUT);
	gpioWrite(SER_SEL, SLE_TMPC); // select temperature controler
    
	// Set PYTHONPATH TO working directory used for GPS parser
	//setenv("PYTHONPATH", "/home/pi/gpsPy:/home/pi/nmea_parser-master:/home/pi/nmea_parser-master/nmea:/home/pi/nmea_parser-master/nmea/core", 1);
	setenv("PYTHONPATH", "/home/pi/mtd415py:/home/pi/mtd415lib/thorlabs-mtd415t:/home/pi/mtd415lib/thorlabs-mtd415t/thorlabs_mtd415t", 1);
	//printf("PATH: %s\n", getenv("PATH"));
	//printf("PYTHONPATH: %s\n", getenv("PYTHONPATH"));
	//printf("in the ctrl_py(%d):\n\r   %s\n\r   %s\n\r   %s\n\r", argc, argv1, argv2, argv3);
	//return; //the code is passed here - DONE

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
	if(strcmp("null", argv3))
	{
		printf("the argv3 is not null. %s\n\r", argv3);
		//args = PyTuple_Pack(1,PyUnicode_DecodeFSDefault(argv3));
	}
	args = PyTuple_Pack(1,PyUnicode_DecodeFSDefault(argv3));

	if (PyCallable_Check(pFunc))
	{
		if(strcmp("null", argv3))
      {
         //printf("the argv3 is %s.\n\r", argv3);
			pmyresult = PyObject_CallObject(pFunc, args/*NULL*/);

      }
		else
      {
         //printf("the argv3 is %s.\n\r", argv3);
      	pmyresult = PyObject_CallObject(pFunc, NULL);
      }  
		i = 0;
	}
	else
	{
		PyErr_Print();
		i = 1;
		return "err0\n";

	}
	PyUnicode_CheckExact(pmyresult);
	//printf("in the ctrl_py\n");

	if (PyUnicode_Check(pmyresult))
	{
		//clrscr();
		PyObject *temp_bytes = PyUnicode_AsEncodedString(pmyresult, "UTF-8", "strict"); // Owned reference
		if (temp_bytes != NULL)
		{
			presultStr = PyBytes_AS_STRING(temp_bytes); // Borrowed pointer
			presultStr = strdup(presultStr);
			Py_DECREF(temp_bytes);
			//printf("  py return: %s", presultStr);
			/* split string presultStr by "," */
			//char *p = strtok(presultStr, ", ");
    		//while(p)
    		//{
        	//	printf("%s \n", p); //print newline
        	//	p = strtok(NULL, ", ");
    		//}
		}
		else
		{
			printf("in the ctrl_py\n");
			return "err2\n";
			// TODO: Handle encoding error.
		}
	}
	else{
		printf("no string return\n\r");
	}

	// Clean up
	Py_DECREF(pModule);
	Py_DECREF(pName);

	// Finish the Python Interpreter
	Py_Finalize();

	gpioWrite(SER_SEL, SLE_LDIS); // set low (borrow SLE_LDIS)

	return presultStr;
#endif
   //return "no mtd415.\n";//presultStr;
   delay(100);
   return "23.0";
}
#if 0
/* files operations */
void *getParas(FILE *fh)
{
   char *dataIn = NULL;
   size_t lenth = 0;
   int readin;

   while((readin = getline(&dataIn, &lenth, fh)) != -1)
   {
      printf("Reading the line with length = %d.\n", readin);
      printf("%s", dataIn);
   }
}
#endif

