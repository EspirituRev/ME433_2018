#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
//#include <NU32.h>
#include <math.h>
#include "i2c_master_noint.h"

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

void init_spi(void){
    SPI1CON = 0;
    SPI1BUF;                 // clear rx buffer
    SPI1BRG = 4;
    //0x1000;        // baud rate compatible with nScope
    SPI1STATbits.SPIROV = 0; // clear overflow bit
    SPI1CONbits.CKE = 1;     // change voltage output when clk active -> idle
    SPI1CONbits.CKP = 0;     // clk active when high
    SPI1CONbits.MSTEN = 1;    
    SPI1CONbits.ON = 1;

    //set cs pin as output
    TRISAbits.TRISA0 = 0; //hook up CS wire to A0(Pin 2)
           
    SDI1Rbits.SDI1R = 0x0001;// RPA1 (Pin 3) as SDI1 Master input
    ANSELAbits.ANSA1 = 0; // Disable analog
    RPA1Rbits.RPA1R = 0b0011; // RPB13 as SDO1 Master output
    // RPB14 is fixed to SCK1 for SPI1    
}


char spi_io(unsigned char c) {
  SPI1BUF = c;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void setVoltage(char a, int V)
{
     short data = 0b0000000000000000;
	data |= a << 15;			// puts channel in the 16th bit by shifting it by 15
    data |= 0b111 << 12;			// makes next three bits 1 by bit-shifting them by 12
    data |= V << 2; 			// voltage stored in the next 8 bits
	
    CS = 0;
    //data=0b0110000011110110;
	spi_io(data >> 8);
	spi_io(data);
    //spi_io(data);
    CS = 1;
    
}



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
    
    //__builtin_enable_interrupts();
    
    //init_spi();
    i2c_master_setup();
    init_i2c(); //
    float count1=1;
    float count2=1;
    float v1=0;
    float v2=0;
    float pi=3.121;
    float steps = 100.0;
    unsigned char val=0;
    int aux = 0;
    
    __builtin_enable_interrupts();
    
    while(1) 
    {
        _CP0_SET_COUNT(0);
//        count2=count2+1.0;
//        v1=floor(512.0*(cos(2.0*pi*count2/steps)+1));
//            setVoltage(0,v1);
//        if (count2>2*steps)
//        {
//            count1=0;
//            //v1=0.0;
//            
//        }
//        //count2=count2+1;
//        if (count2==2*steps)
//        {
//            setVoltage(1,v2);
//            count2=0;
//            v2=0;
//            
//        }   
//        if (count2<steps)
//        {
//            v2=1024.0*count2/steps;
//            setVoltage(1,v2);
//        }
//        if (count2>steps)
//        {
//            v2=-1024.0*(count2-steps)/steps;
//            setVoltage(1,v2);
//        }
        
        val = get();
        aux = (val>>7); //look at pin 0 only
        if (aux){ // if button is pressed
            set(1,0); // level (high), pin number (GP0)
        }
        else {
            set(0,0);
        }
        
        LATAbits.LATA4=1;
            
        while(_CP0_GET_COUNT()<24000000/10){ }
        
        _CP0_SET_COUNT(0);
        LATAbits.LATA4=0;
        while(_CP0_GET_COUNT()<24000000/10){ }
    }          
}










//void initExp()
//{
//    ANSELBBITS.ANSB2=0;
//    ANSELBBITS.ANSB3=0;
//    i2c_master_setup();
//    
//    writei2c(req,val);//IODIR
//    
//    writei2c(req,val);//LAT
//    
//    
//}
//
//#define ADDR 0B010000
//void writei2c(unsigned char reg, unsigned char val)
//{
//    i2c_master_start();
//    i2c_master_send(ADDR<<1|0);
//    i2c_master_send(
//}