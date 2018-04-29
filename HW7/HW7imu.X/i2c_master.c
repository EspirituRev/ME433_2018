#include<xc.h>
#include "i2c_master.h"


// I2C Master utilities, 100 kHz, using polling rather than interrupts
// The functions must be callled in the correct order as per the I2C protocol
// Change I2C1 to the I2C channel you are using
// I2C pins need pull-up resistors, 2k-10k

void i2c_master_setup(void) {
  I2C2BRG = 235;//some number for 100kHz;            // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2
                                    // look up PGD for your PIC32
  I2C2CONbits.ON = 1;               // turn on the I2C1 module
}

// Start a transmission on the I2C bus
void i2c_master_start(void) {
    I2C2CONbits.SEN = 1;            // send the start bit
    while(I2C2CONbits.SEN) { ; }    // wait for the start bit to be sent
}

void i2c_master_restart(void) {
    I2C2CONbits.RSEN = 1;           // send a restart
    while(I2C2CONbits.RSEN) { ; }   // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
  I2C2TRN = byte;                   // if an address, bit 0 = 0 for write, 1 for read
  while(I2C2STATbits.TRSTAT) { ; }  // wait for the transmission to finish
  if(I2C2STATbits.ACKSTAT) {        // if this is high, slave has not acknowledged
    // ("I2C2 Master: failed to receive ACK\r\n");
  }
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
    I2C2CONbits.RCEN = 1;             // start receiving data
    while(!I2C2STATbits.RBF) { ; }    // wait to receive the data
    return I2C2RCV;                   // read and return the data
}

void i2c_master_ack(int val) {        // sends ACK = 0 (slave should send another byte)
                                      // or NACK = 1 (no more bytes requested from slave)
    I2C2CONbits.ACKDT = val;          // store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1;            // send ACKDT
    while(I2C2CONbits.ACKEN) { ; }    // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) {          // send a STOP:
  I2C2CONbits.PEN = 1;                // comm is complete and master relinquishes bus
  while(I2C2CONbits.PEN) { ; }        // wait for STOP to complete
}

void init_i2c(unsigned char addr){
    ANSELBbits.ANSB2 = 0;       //turn off analog
    ANSELBbits.ANSB3 = 0;
    //com i2c 0-3 in 4-7 out
    i2c_master_start();
    i2c_master_send(addr<<1);
    i2c_master_send(0x00);
    i2c_master_send(0b11110000);
    i2c_master_stop();
    
}
void set(int level, int pin , unsigned char addr ){
    unsigned char data = 0x00;
    data = (data|level)<<pin;
    
    i2c_master_start();
    i2c_master_send(addr<<1);
    i2c_master_send(0x0A);
    i2c_master_send(data);
    i2c_master_stop();
}

void I2C_send(unsigned char addr,unsigned char reg, unsigned char val){
    i2c_master_start();
    i2c_master_send(addr << 1); //device opcode write mode
    i2c_master_send(reg);         //specify register address 
    i2c_master_send(val);             //data to slave
    i2c_master_stop();
}
unsigned char get(unsigned char addr){
    unsigned char ans=0;
    i2c_master_start();
    i2c_master_send(addr<<1);
    i2c_master_send(0x09);
    i2c_master_restart();
    i2c_master_send(addr<<1|1);
    ans=i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return ans;    
}

void init_I2C_IMU(unsigned char addr){
    I2C_send(addr, CTRL1_XL, 0b10000000); //Sample rate 1.66kHz, 2g sensitivity, 100Hz filter
                                  //0b10000010
    I2C_send(addr, CTRL2_G, 0b10000000);  //Sample rate 1.66kHz, 1000dps sensitivity
                                  //0b10001000
    I2C_send(addr, CTRL3_C, 0b100); //IF_INC bit = 1, this is bit2 on register   
}
unsigned char I2C_read_WAI(unsigned char addr, unsigned char dir){
    unsigned char ans=0;
    i2c_master_start();
    i2c_master_send(addr<<1);
    i2c_master_send(dir);
    i2c_master_restart();
    i2c_master_send(addr<<1|1);
    ans=i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return ans;
}
void I2C_readall_imu(unsigned char addr, unsigned char dir,unsigned char *data){
    unsigned char ans=0;
    i2c_master_start();
    i2c_master_send(addr<<1);
    i2c_master_send(dir);
    i2c_master_restart();
    i2c_master_send((addr<<1)|1);
    unsigned int ii=0;
    for(ii=0;ii<14;ii++){
        data[ii]=i2c_master_recv();
        if (ii<13){
            i2c_master_ack(0);
        }
        else{
            i2c_master_ack(1);
        }
    }
    i2c_master_stop();
}
unsigned char I2C_readtest(unsigned char addr, unsigned char dir, unsigned char test){
       
    i2c_master_start();
    i2c_master_send(addr<<1);
    i2c_master_send(dir);
    i2c_master_restart();
    i2c_master_send(addr<<1|1);
    test=i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return test;
}