


void initExp()
{
    ANSELBBITS.ANSB2=0;
    ANSELBBITS.ANSB3=0;
    i2c_master_setup();
    
    writei2c(req,val);//IODIR
    
    writei2c(req,val);//LAT
    
    
}

#define ADDR 0B010000
void writei2c(unsigned char reg, unsigned char val)
{
    i2c_master_start();
    i2c_master_send(ADDR<<1|0);
    i2c_master_send(
}