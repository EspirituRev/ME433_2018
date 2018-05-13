#ifndef DSP_H__
#define DSP_H__
// do all the differents process of DSP
#define LENGTH 5
float w[]={0.02,0.24,0.48,0.24,0.02};
void InitBuffer(float* buffer);
void MoveBuffer(float* buffer,float newv);
float MAF(float* buffer);
float FIR(float* buffer);

#endif
