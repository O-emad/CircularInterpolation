/*Misc.c
Runs on tm4c123
Omar Emad El-Deen
March 2, 2017*/


////Doc.////
/*
*/

////Includes////
#include <stdint.h>
#include "tm4c123gh6pm.h"
////Defines////

/////////////////////////////////////

void Sync(uint32_t timers){
	TIMER0_SYNC_R = timers;
}

void Async(uint32_t timers){
	TIMER0_SYNC_R &=~ timers;
}
void CLK_GPIO(uint32_t ports){
	SYSCTL_RCGCGPIO_R |= ports;
	while((SYSCTL_PRGPIO_R&ports) != ports ){}
}

void CLK_GPTM(uint32_t timers){
	SYSCTL_RCGCTIMER_R |= timers;
	while((SYSCTL_PRTIMER_R&timers) != timers ){}
}

void CLK_WGPTM(uint32_t timers){
	SYSCTL_RCGCWTIMER_R |= timers;
	while((SYSCTL_PRWTIMER_R&timers) != timers ){}
}

void CLK_UART(uint32_t uart){
	SYSCTL_RCGCUART_R |= uart;
	while((SYSCTL_PRUART_R&uart) != uart ){}
}

void CLK_ADC(uint32_t adc){
	SYSCTL_RCGCADC_R |= adc;
	while((SYSCTL_PRADC_R&adc) != adc){}
}
int adjust_num(double num) {
	return num*1e4; 
}


////////////////////////////////////////////////////////////////////
/*double** createArray(uint32_t row, uint32_t val){
	uint32_t i;
	double* values = calloc(row*val,sizeof(double));
	double** rows = malloc(row*sizeof(double*));
	for(i=0;i<row;++i){
		rows[i] = values +i*val;
	}
	return rows;
}

void destroyArray(double** arr){
	free(*arr);
	free(arr);
}


*/
///////////////////////////////////////////////////////////////////

