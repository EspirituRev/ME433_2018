/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include <stdio.h>
#include <math.h>
#include "ST7735.h"
#include "i2c_master.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    #define CS      LATAbits.LATA0
    #define addr    0b00100001
    #define addr2   0b01101011
    #define screen  0x001F
    
    TRISAbits.TRISA4 = 0;
    //TRISBbits.TRISB4 = 1;
    
    i2c_master_setup();
    init_i2c(addr); //initialize i2c
    LCD_init();
    init_I2C_IMU(addr2);
    LCD_clearScreen(screen);
    
    
    
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    int time=12000;
    unsigned char val=0;
    int aux = 0;
    char message1[20];
    char message2[20];
    int aux2 =0;
    int x=0;
    int y=0;
    unsigned char data[14];
    signed short temp;
    signed short gyrox;
    signed short gyroy;
    signed short gyroz;
    signed short accelx;
    signed short accely;
    signed short accelz;
    char test;
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
                
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
        _CP0_SET_COUNT(0);

        
        val = get(addr);
        aux = (val>>7); //look at pin 0 only
        if (aux){ // if button is pressed
            
            set(0,0,2,addr); // level (low), pin number (GP0)
            //set(0,1,addr); // level (low), pin number (GP0)
        }
        else {
            set(1,0,2,addr);
            //set(1,1,2,addr);
        }
        sprintf(message1,"Expander value: %d  ",!aux);
        
        LCD_drawString(15,5,message1,WHITE,screen);
        aux2 = I2C_read_WAI(addr2,WHO_AM_I);
        
        sprintf(message2,"Imu Value: %d  ",aux2);
        LCD_drawString(15,15,message2,WHITE,screen);
        
        I2C_readall_imu(addr2, 0x20, data);
        test=I2C_readtest(addr2, 0x22, test);
        temp=(data[1]<<8)|data[0];
        gyrox=(data[3]<<8)|data[2];
        gyroy=(data[5]<<8)|data[4];
        gyroz=(data[7]<<8)|data[6];
        accelx=(data[9]<<8)|data[8];
        accely=(data[11]<<8)|data[10];
        accelz=(data[13]<<8)|data[12];
        
        float xscale = accelx/163.0; 
        float yscale = accely/163.0;
        float zscale = accelz/163.0;
        
        sprintf(message2,"TEMP: %d  ",temp);
        LCD_drawString(15,25,message2,WHITE,screen);
        
        sprintf(message2,"X: %3.0f  ",xscale);
        LCD_drawString(15,35,message2,WHITE,screen);
        
        sprintf(message2,"Y: %3.0f  ",yscale);
        LCD_drawString(15,45,message2,WHITE,screen);
        
        sprintf(message2,"Z: %3.0f  ",zscale);
        LCD_drawString(15,55,message2,WHITE,screen);
        
        LCD_drawBarVer(63,40,4,120,zscale/2,RED,WHITE);
        LCD_drawBarHor(1,98,128,4,xscale/2,RED,WHITE);
        
        LATAbits.LATA4=1;
            
        while(_CP0_GET_COUNT()<24000000/10){ }
        
        _CP0_SET_COUNT(0);
        LATAbits.LATA4=0;
        while(_CP0_GET_COUNT()<24000000/10){ }
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
