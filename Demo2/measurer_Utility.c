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

bool GoGo = TRUE;
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

    printf("Distance = ");
    printf("%5.1f", distance);
    printf(" m\n");

    // execute code shutdown;
    // printf("Execute %s.\n", command);
    reps = uart_cmdmsg(&muart, &cmdmsg, LASER_CTLSHT_CMD);

    uart_close(hd, &muart); // serialClose(hd);
    printf("LDistance Sensor UART Closed. \n");
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
   //regInfor *pregInf;
   //adc_channel_data *padcData; // TODO check where it is definced(DONE).
   regInfor *pregInf = &regSetInf;
   adc_channel_data *padcData = &adcData;
   userData* pfuncData = &adcCapFuncData;
   
   /* start the ads131 chip */
   hd = tspi_ads131m04_start(pregInf, padcData);

   /* set capture function to ads131 */
   pfuncData->pRslts = &adcRltData[0];
   pfuncData->handle = hd;
   pfuncData->datIdx = 0;
   pfuncData->isRun = 0;
   gpioSetAlertFuncEx(ADC_DRDY, NULL, pfuncData);

   return hd;
}

/* old ratio function */
// data used in the getLSRatio
char *mtd415 = "mtd415Set";
char *mtd415setFunc = "tecCmdset";
char *mtd415getFunc = "tecCmdget";

float getLSRatio(userData* pfuncData) //TODO - place userData by measThr.
{
    int count = 0;
    float tecRet = 0;
    char *presult;
    manuCst cnstRlts;

   //return 0.5; //debug code
   /* use alert function */
   pfuncData->datIdx = 0;
   pfuncData->isRun = 1;

   #if 0 //for capture ads131 data   
      gpioSetAlertFuncEx(ADC_DRDY, adcCaptureFun, pfuncData);

      /* while loop for checking temp and adc each 100 ms */
      uint32_t curTick, preTick;
      uint32_t preDatTick = 0;
      int ratioIdx = 0, capIdx = 0;
      float ratioArray[20];
      preTick = curTick = gpioTick(); 
      while(/*!kbhit()*/pfuncData->isRun == 1 && count < 24)
      {
         //if((curTick - preTick) >= 100000)
         //cnstRlts.dataCnt;
         if(pfuncData->datIdx > 23)
         {
            pfuncData->isRun = 0;
            /* get temperature */              
            //char *pargu1 = "mtd415Set";
            //char argu2[32] = "tecCmd"; 
            char argu3[32] = "get temperature";
            //char *presult;
         
            presult = tempCtrll_py(3, mtd415, mtd415getFunc, &argu3[0]);
            printf("\r     temperature is %s.\r\n", presult);

            /* calculate data */
            int32_t fx1 = 0,fy1 = 0, fx2 = 0, fy2 = 0, maxDtick = 0, curTick;
            double df1, df2, v1, v2, v3, v4, step;
            
            for(int n = 2; n < 22; n++)
            {
               fx1 += (pfuncData->pRslts + n)->results0;
               fy1 += (pfuncData->pRslts + n)->results1;
               fx2 += (pfuncData->pRslts + n)->results2;
               fy2 += (pfuncData->pRslts + n)->results3;
               if(n > 3)
                  curTick = (pfuncData->pRslts + n)->tick - (pfuncData->pRslts + (n - 1))->tick;
               maxDtick = (maxDtick > curTick) ? maxDtick:curTick;
            }

            fx1 /= 20; fy1 /= 20; fx2 /= 20; fy2 /= 20;
            
            /* convert to voltage */
            //int count = 0, countf = 0;
               
            #if 1  //TODO - rm following code 
            uint32_t ticksSum = 0;

            if(fx1 > 0x7fffff) //adcData.channel0
            {
               v1 = (double)(~(fx1 | 0xff000000)+1);
               v1 = -v1;
            }
            else
            {
               v1 = (double)fx1;
            }

            if(fy1 > 0x7fffff) //adcData.channel1
            {
               v2 = (double)(~(fy1 | 0xff000000)+1);
               v2 = -v2;
            }
            else
            {
               v2 = (double)fy1;
            }
            
            if(fx2 > 0x7fffff) //adcData.channel2
            {
               v3 = (double)(~(fx2 | 0xff000000)+1);
               v3 = -v3;
            }
            else
            {
               v3 = (double)fx2;
            }
            
            if(fy2 > 0x7fffff) //adcData.channel3
            {
               v4 = (double)(~(fy2 | 0xff000000)+1);
               v4 = -v4;
            }
            else
            {
               v4 = (double)fy2;
            }
            #endif //TODO end

            step = 1200000.0 / 8388607.0;

            v1 *= step;
            v2 *= step;
            v3 *= step;
            v4 *= step;
            
            df1 = v1*v1 + v2*v2;
            df2 = v3*v3 + v4*v4;

            df1 = sqrt(df1);
            df2 = sqrt(df2);

            curTick = gpioTick();

            capIdx = cnstRlts.dataCnt = count;
            
            capIdx = capIdx%20;
            cnstRlts.dataCnt = capIdx;
            if(df2 != 0.0)
            {
               //n = (n+1)%20;
               //cnstRlts.dataCnt = n;

               cnstRlts.Rslt[capIdx].squF1 = df1;
               cnstRlts.Rslt[capIdx].squF2 = df2;
               ratioArray[capIdx] = cnstRlts.Rslt[capIdx].ratio = df1/df2;
               //printf("     lasting %d max-dalt %d; result %.4f, %.4f and ratio %.4f\n\r", (curTick - preTick), maxDtick, df1, df2, df1/df2);
               printf("     result %.4f, %.4f and ratio %.4f\n\r", df1, df2, df1/df2);
            }
            else
            {
               printf("    dF2 is zero. %.4f, %.4f", df1, df2);
            }
            //printf("    (%d): %.02f, %.02f, %.2f, %.2f\n\r", count, v1, v2, v3, v4);
         
            /* renew the data buffer */
            pfuncData->datIdx = 0;
            pfuncData->isRun = 1;
            curTick = preTick = gpioTick();
            count++;
            //}

            /* end of convert */
            
            /* output data */
            printf("\r    Time lasting in Samples: %d - %d -- %d\n\r", (pfuncData->pRslts + 2)->tick,
            (pfuncData->pRslts + 21)->tick,
            (pfuncData->pRslts + 21)->tick - (pfuncData->pRslts + 2)->tick);

            //curTick = gpioTick();
         }
         //if(kbhit())
         //{
         //   pfuncData->isRun = 0;
         //   break;
         //}
      }

      /* any keyboard press ends the loop */
      //printf(" ");
      //scanf("%s", command);

      pfuncData->isRun = 0;
      
      //endwin(); //end ncurses

      /* check the capture data */
      printf("\n\n    Calculating Constant.\n     Input the gas concentration: ");
      float cnstRlt = 0, samplePercent = 0, avgRatio = 0;
      samplePercent = 0; avgRatio = 0;
      scanf("%f", &samplePercent);

      for(ratioIdx = 0; ratioIdx < 20; ratioIdx++)
      {
         avgRatio += ratioArray[ratioIdx];
         //printf("     debug (%d)) %.4f, %.4f\n", ratioIdx, ratioArray[ratioIdx], avgRatio);
      }
      
      if(avgRatio == 0)
      {
         printf("the ratio is zero!\n");
      }
      else{
         printf("    totalRatio %.4f; numbers %d\n", avgRatio, ratioIdx);
         avgRatio /= ratioIdx;
         cnstRlt = samplePercent/avgRatio;
         printf("\n     Average Ration %.4f; concentration %.4f The contant is %.4f. \n", 
               avgRatio, samplePercent, cnstRlt);
      }


      /* save data in a file */
      // save date and time in file

      // save current constant in file TODO save result into a file
      //fprintf(fh, "Gas Concentration: %.4f, Constant: %.4f, AvgRatio: %.4f\n", samplePercent, cnstRlt, avgRatio);
   #endif
   return 0.5; //avgRatio;
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
 * 
 * *****************************************************/
// cresult = call_Python_Stitch(6, "Image_Stitching", "main", "Images", "output.jpeg","--images","--output");
				
const char *tempCtrll_py(int argc, char *argv1, char *argv2, char *argv3)
{
   char resultStr[32], *presultStr = &resultStr[0];
#if 0 //for debug
	PyObject *pName, *pModule, *pDict, *pFunc, *pValue, *pmyresult, *args, *kwargs;
	int i;
	char resultStr[32], *presultStr = &resultStr;

    //gpioSetMode(SER_SEL, PI_OUTPUT);
	gpioWrite(SER_SEL, SLE_TMPC); // select temperature controler
    
	// Set PYTHONPATH TO working directory used for GPS parser
	//setenv("PYTHONPATH", "/home/pi/gpsPy:/home/pi/nmea_parser-master:/home/pi/nmea_parser-master/nmea:/home/pi/nmea_parser-master/nmea/core", 1);
	setenv("PYTHONPATH", "/home/pi/mtd415py:/home/pi/mtd415lib/thorlabs-mtd415t:/home/pi/mtd415lib/thorlabs-mtd415t/thorlabs_mtd415t", 1);
	//printf("PATH: %s\n", getenv("PATH"));
	//printf("PYTHONPATH: %s\n", getenv("PYTHONPATH"));
	//printf("in the ctrl_py(%d):\n\r   %s\n\r   %s\n\r   %s\n\r", argc, argv1, argv2, argv3);
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
	if(strcmp("null", argv3))
	{
		//printf("the argv3 is not null. %s\n\r", argv3);
		args = PyTuple_Pack(1,PyUnicode_DecodeFSDefault(argv3));
	}
		args = PyTuple_Pack(1,PyUnicode_DecodeFSDefault(argv3));
	// kwargs = PyTuple_Pack(2,PyUnicode_DecodeFSDefault(argv5), PyUnicode_DecodeFSDefault(argv6));
	// args = Py_BuildValue("ssss", argv5, argv3, argv6, argv4);
	// kwargs = Py_BuildValue("ss", argv5, argv6);

	if (PyCallable_Check(pFunc))
	{
		if(strcmp("null", argv3))
			pmyresult = PyObject_CallObject(pFunc, args/*NULL*/);
		else
			pmyresult = PyObject_CallObject(pFunc, NULL);
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
			return;
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

	//gpioWrite(SER_SEL, SLE_LDIS); // set low (borrow SLE_LDIS)

	return presultStr;
#endif
   return presultStr;
}




