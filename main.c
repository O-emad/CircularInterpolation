


#include <stdint.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "global.h"
#include "PLL.h"
#include "SysTick.h"


double Gnext[T];
double Gcurrent[T];
double machine_cons[3];


int t,q;
double center[3];
int8_t direction[2];
void G02(void);
int main(){
	PLL_Init();
	SysTick_Init();
	Gcurrent[X] = 15;
	Gcurrent[Y] = 0;
	Gnext[X] = 0;
	Gnext[Y] = 15;
	Gnext[R] = 15;
	machine_cons[0] = 0.00125;
	machine_cons[1] = 0.00125;
	G02();
	while(1){
	
	
	}
}

