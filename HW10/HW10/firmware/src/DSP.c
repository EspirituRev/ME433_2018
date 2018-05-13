#include<xc.h>
#include "DSP.h"


int ii=0;
float aux;

void InitBuffer(float* buffer){
    for (ii=0;ii<LENGTH;ii++){
        buffer[ii]=0;
    }
}
void MoveBuffer(float* buffer,float newv){
    for(ii=0;ii<LENGTH-1;ii++){
        buffer[LENGTH-ii-1]=buffer[LENGTH-ii-2];
    }
    buffer[0]=newv;
}
float MAF(float* buffer){
    aux=0;
    for (ii=0;ii<LENGTH;ii++){
        aux=buffer[ii]+aux;
    }
    return aux/LENGTH;
}

float FIR(float* buffer){
    aux=0;
    for(ii=0;ii<LENGTH;ii++){
        aux=w[ii]*buffer[ii]+aux;
    }
    return aux;
}