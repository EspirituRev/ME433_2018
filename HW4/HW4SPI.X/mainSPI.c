#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
//#include <NU32.h>
#include <math.h>

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
    SPI1BRG = 0x1000;
    //0x1000;        // baud rate compatible with nScope
    SPI1STATbits.SPIROV = 0; // clear overflow bit
    SPI1CONbits.CKE = 1;     // change voltage output when clk active -> idle
    SPI1CONbits.CKP = 0;     // clk active when high
    SPI1CONbits.MSTEN = 1;    
    SPI1CONbits.ON = 1;

    //set cs pin as output
    TRISAbits.TRISA0 = 0; //hook up CS wire to A0(Pin 2)
           
    SDI1Rbits.SDI1R = 0x0;// RPA1 (Pin 3) as SDI1 Master input
    ANSELBbits.ANSB13 = 0; // Disable analog
    RPB13Rbits.RPB13R = 0b0011; // RPB13 as SDO1 Master output
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
    //TRISAbits.TRISA4 = 0;
    
    //TRISBbits.TRISB4 = 1;
    
    //__builtin_enable_interrupts();
    
    init_spi();
    int count1=0;
    int count2=0;
    float v1=0;
    float v2=0;
    float pi=3.121;
    
    __builtin_enable_interrupts();
    
    while(1) 
    {
        _CP0_SET_COUNT(0);
//        count1=count1+1;
//        if (count1<100)
//        {
//            v1=512.0*sin(2.0*pi*count1/100.0);
//            setVoltage(0,v1);
//        }
//        else
//        {
//            count2=0;
//            v1=0.0;
//            setVoltage(0,v1);
//        }
//        count2=count2+1;
//        if (count1<100)
//        {
//            v2=512.0*count2/100.0;
//            setVoltage(0,v2);
//        }
//        if (count2>100)
//        {
//            v2=-512.0*(count2-100.0)/100.0;
//            setVoltage(0,v2);
//        }
//        if (count2==200)
//        {
//            count2=0;
//            v2=0;
//            setVoltage(0,v2);
//        }
        //setVoltage(0,512);
        setVoltage(0,3*1024/4);
        setVoltage(1,1024/4);
        while(_CP0_GET_COUNT()<24000000/100)
    {
        
    }
    
    }
	// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
	// remember the core timer runs at half the sysclk
            
}
