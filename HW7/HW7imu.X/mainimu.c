#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
#include <stdio.h>
#include <math.h>
#include "i2c_master.h"
#include "ST7735.h"

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25// wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 00001111 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define CS      LATAbits.LATA0
#define addr    0b00100001
#define addr2   0b01101011

int main(void) 
{
      
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;
    
    //TRISBbits.TRISB4 = 1;
    short screen=BLUE;
    i2c_master_setup();
    init_i2c(addr); //initialize i2c
    LCD_init();
    init_I2C_IMU(addr2);
    LCD_clearScreen(screen);
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
    
    __builtin_enable_interrupts();
    
    while(1) 
    {
        _CP0_SET_COUNT(0);

        
        val = get(addr);
        aux = (val>>7); //look at pin 0 only
        if (aux){ // if button is pressed
            
            set(0,0,addr); // level (high), pin number (GP0)
        }
        else {
            set(1,0,addr);
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
        
        float xscale = accelx/325.0; 
        float yscale = accely/325.0;
        float zscale = accelz/325.0;
        
        sprintf(message2,"TEMP: %d  ",temp);
        LCD_drawString(15,25,message2,WHITE,screen);
        
        sprintf(message2,"X: %3.0f  ",xscale);
        LCD_drawString(15,35,message2,WHITE,screen);
        
        sprintf(message2,"Y: %3.0f  ",yscale);
        LCD_drawString(15,45,message2,WHITE,screen);
        
        sprintf(message2,"Z: %3.0f  ",zscale);
        LCD_drawString(15,55,message2,WHITE,screen);
        
        LCD_drawBarVer(63,40,4,120,yscale,RED,WHITE);
        LCD_drawBarHor(1,98,128,4,xscale,RED,WHITE);
        
        LATAbits.LATA4=1;
            
        while(_CP0_GET_COUNT()<24000000/10){ }
        
        _CP0_SET_COUNT(0);
        LATAbits.LATA4=0;
        while(_CP0_GET_COUNT()<24000000/10){ }
        
    }          
}