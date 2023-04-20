/**************************************
 * the pigpio application to communicat via UART
 * code protype comes from pigpio example
 * piSerial.h
 * *****************/
 /*****************//**
*  \brief Laser Distance Measuring Module APIs.
*  The UART Module Routines which are based on the pigpio library.
*  tspi_serial_start(): open a serial port with a baud rate, return a handle.
*  tspi_serial_close(): close a serial port.
*  tspi_serial_write(): write a group of bytes with count value.
*  tspi_serial_read(): read a group of bytes that received via uart port. Return a count value.
*  tspi_serial_dataRdy(): check any received data is available to be read.
*  //uartStat(): get uart current state.
*
*  laser turn on - laserOn()
*  laser turn off - laserOff()
*  continuous measurement - measureCon()
*  single measurement - measureSig()
*/

#ifndef PISERIAL_H
#define PISERIAL_H

#include <stdbool.h>

#define LASERDST   0
#define TEMPCTRL   1
#define BTSTRINGS  32

#define LASER_ADDRESS     0x80
 
/* pre-command processing */
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
 
typedef struct portUART_t{ // UART module
	int serHandle;
	int serID;
	char * pTxbuf; // tx buffer pointer.
	char * pRxbuf; // rx buffer pointer.
	unsigned int  Txbuf_lgth;
	unsigned int  Rxbuf_lgth;
}UARTport;

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

/* serial port data */
/* serial operations functions */
//void UART_distMain(int isConti);
//void UART_tempCMain(int isConti);
void tempCtrll_py(int argc, char *argv1, char *argv2, char *argv3);

int uart_start(int chan, UARTport* uart);
int uart_close(int SERHandle, UARTport* uart);
int uart_write(UARTport* uart);
int uart_read(UARTport* uart, int byteCnt);
int uart_dataRdy(int SERHandle);

/* serial data operations functions for polling */
void uart_preExch(UARTport* uart, SERCmdMsg* cmg);
//int uart_PosExch(UARTport* uart, SERCmdMsg* cmg);
int uart_ExchData(UARTport* uart, SERCmdMsg* cmg);
int uart_cmdmsg(UARTport *uart, SERCmdMsg *pcdmg, int CmdID);

/* help routines */
char testSum(char *pTxbuf, int numbytes);
int serSendCnt(char *string);

#endif // End PISERIAL_H
