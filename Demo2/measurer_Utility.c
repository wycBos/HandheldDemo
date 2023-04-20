/******************************************
 * the file is modified from UART_test.c and
 * piPeriphTest.c, it includes SPI, UART, etc.
 *
 * *****************************************/

#include <stdio.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdbool.h>
#include "UART_test.h"
#include "piSerial.h"

bool GoGo = TRUE;

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

// int main()
float UART_main()
{
    int fd;
    int c;
    int numbytes;
    char contimeas[4] = {0x80, 0x06, 0x03, 0x77};
    char singlemeas[4] = {0x80, 0x06, 0x02, 0x78};
    char shutdown[4] = {0x80, 0x04, 0x02, 0x7A};
    char laseron[5] = {0x80, 0x06, 0x05, 0x01, 0x74};
    char laseroff[5] = {0x80, 0x06, 0x05, 0x00, 0x75};
    char setrange5[5] = {0xFA, 0x04, 0x09, 0x05, 0xF4};
    unsigned char data[11] = {0};
    float distance = -1.0;

    if (wiringPiSetup() < 0)
        return 1;
    if ((fd = serialOpen("/dev/serial0", 9600)) < 0)
        return 1;
    //    if((fd = serialOpen("/dev/ttyAMA0",115200)) < 0)return 1;
    //    serialFlush(fd);
    printf("serial test start ...\n");

    serialPrintf(fd, contimeas);
    // serialPrintf(fd,singlemeas);

    int counter = 0, counterErr = 0;
    GoGo = TRUE;
    while (GoGo)
    {

        if (counter > 2 || counterErr > 50)
            GoGo = FALSE;
        // delay(50);
        if ((numbytes = serialDataAvail(fd)) > 0)
        {
            delay(50);
            counter++;
            printf("received %d \n", numbytes);
            for (int i = 0; i < 11; i++)
            {
                data[i] = serialGetchar(fd);
                // printf("%x ",data[i]);
            }
            // printf("\n");
            unsigned char Check = 0;
            for (int i = 0; i < 10; i++)
            {
                Check = Check + data[i];
            }
            Check = ~Check + 1;
            printf("%x \n", Check);
            if (data[10] == Check)
            {
                if (data[3] == 'E' && data[4] == 'R' && data[5] == 'R')
                {
                    printf("Out of range");
                }
                else
                {
                    distance = 0;
                    distance = (data[3] - 0x30) * 100 + (data[4] - 0x30) * 10 + (data[5] - 0x30) * 1 + (data[7] - 0x30) * 0.1 + (data[8] - 0x30) * 0.01 + (data[9] - 0x30) * 0.001;
                    printf("Distance = ");
                    printf("%5.1f", distance);
                    printf(" m\n");
                }
            }
            else
            {
                printf("Invalid Data! %d\n", numbytes);
            }
        }
        else
        {
            counterErr++;
        }
        delay(20);
    }
    // serialPrintf(fd,laseroff);
    // delay(500);
    // serialPrintf(fd,shutdown);
    // delay(500);
    serialClose(fd);
    // printf("Received q for Quit \n");
    return distance; // 0;
}

/* new code starts here */
/******************************
 * it includes:
 *  - laser distance measuring, uart-laserDistance (is gpio_proc.c)
 *  - ADS131 input capture, ADC-ads131m04
 *  - gas concentration outpur, gas_meas()
 *  - temperature controller set, uart-tempCtrl (in gpio_proc.c)
 *    ......
 *
 * ****************************/

/*******************************************************
 * UART_distMain communicates to a laser distance sensor for the distance measuring.
 * - Parameters:
 *   isCounti: indicate the continue or single measuring.
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
