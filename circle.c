/*circle.c
Runs on tm4c123
Omar Emad El-Deen
Aug 16, 2017*/

//---Doc.---//
/*
this work is based on our previous project in June 19,2017 under the name of CInterpolation. 
what is new... the circle interpolation algorithm has been modified and provides the same accuracy as a bresenham's circle and capable
of running at 42khz that's faster than the previous version by 15khz.. but it will be forced to run at maximum 30khz just for safty 
that's around 2200 mm/min feedrate on a system that runs with 200 step/rev stepper on 1/8th microstepping and a screw pitch of 2mm.
also the method for calculating the arc center using only 2 points and radius can differentiate between the four possible arcs,
and calculate the arc length in each case.
the arc length is used for calculating the number of steps required to finish the arc
*/

//---problems---//
/*
till now every calculation and step is done in real time without using fifos which might have a bad effect on the motion planning,
and violates the planned curve.
the system uses delays for planning the time between steps, we think interrupts might be a better solution.
*/

#include <math.h>
#include <stdint.h>
#include <string.h>
#include "global.h"
#include "Misc.h"
#include "SysTick.h"
#include "UART.h"


static double centers[3];
static int32_t comp[10];
static double rsq;
static double xi,yi;
static uint32_t steps;


//--get_centers--//
//calculates the center of an arc using only two points and raduis, and calculates the arc length is radians
//can differentiate between the 2 major and 2 minor arc using the direction and the sign of the radius
//Inputs: direction of arc CW or CCW, array in which the values will be places
//Outputs: none
//Notes: accesses the global variables declared at global.h
void get_centers(uint8_t dir,double arc_cal[]){
	double dx,dy,ksq,q,xm,ym;
	xm = (Gnext[X] + Gcurrent[X])/2;
	ym = (Gnext[Y] + Gcurrent[Y])/2;
	dy = Gnext[Y] - Gcurrent[Y];
	dx = Gnext[X] - Gcurrent[X];
	ksq = (dx*dx)+(dy*dy);
	q = sqrt(ksq);
	rsq = Gnext[R]*Gnext[R];
	arc_cal[2] = acos(1-(ksq/(2*rsq)));
	if(dir == CW){
		if(Gnext[R]>0){
			arc_cal[0] = xm - sqrt(rsq-((q/2)*(q/2)))*(-dy/q);
			arc_cal[1] = ym - sqrt(rsq-((q/2)*(q/2)))*(dx/q);
		}
		else{
			arc_cal[0] = xm + sqrt(rsq-((q/2)*(q/2)))*(-dy/q);
			arc_cal[1] = ym + sqrt(rsq-((q/2)*(q/2)))*(dx/q);
			arc_cal[2] = 6.283185307-arc_cal[2];
		}
	}
	else if(dir == CCW){
		if(Gnext[R]>0){
			arc_cal[0] = xm + sqrt(rsq-((q/2)*(q/2)))*(-dy/q);
			arc_cal[1] = ym + sqrt(rsq-((q/2)*(q/2)))*(dx/q);
		}
		else{
			arc_cal[0] = xm - sqrt(rsq-((q/2)*(q/2)))*(-dy/q);
			arc_cal[1] = ym - sqrt(rsq-((q/2)*(q/2)))*(dx/q);
			arc_cal[2] = 6.283185307-arc_cal[2];
		}
	
	}
}

void Arc_Param_Init(void){
	memset(centers,0,sizeof(double));
	memset(comp,0,sizeof(int32_t));
	rsq = 0;
	steps = 1000;
	xi = Gcurrent[X];
	yi = Gcurrent[Y];
}

void Arc_Param_Cal(uint8_t dir){
	get_centers(dir,centers);
	
	//--integer values used for comparing--//
	comp[0] = adjust_num(centers[0]);  //xc
	comp[1] = adjust_num(centers[1]);  //yc
	comp[2] = adjust_num(Gnext[X]);	//xnext
	comp[3] = adjust_num(Gnext[Y]);	//ynext
	comp[4] = adjust_num(Gcurrent[X]);	//xcurrent
	comp[5] = adjust_num(Gcurrent[Y]);	//ycurrent
	comp[6] = adjust_num(machine_cons[0]); //trajconsx	
	comp[7] = adjust_num(machine_cons[1]);	//trajconsy
	comp[8] = comp[4];   //xi
	comp[9] = comp[5];   //yi
}

//--quadrant_binrep--//
//Inputs: xo,yo are true or false variables which which gains their states from a previous comparison between the intermediate point and arc center point
//Outputs: the quadrant at which the point is located 
//Notes: 0 --> 1st quadrant   1 --> 2nd    3 --> 3rd   2 --> 4th
uint8_t quadrant_binrep(uint8_t xo, uint8_t yo){
	return ((xo)? 0 : 1)+((yo)? 0 : 2); 
}
//--direction-binrep--//
//Inputs: direction of arc CW or CCW and the quadrant of the intermediate point
//Outputs: pointer to an array containing the direction of each axis
//Notes: -1 stands form CCW rotation of motor and 1 for CW
void direction_binrep(uint8_t dir, uint8_t i,int8_t direction[]){
	switch(i){
		case 0: direction[0] = (dir == CW)? 1:-1; direction[1] = - direction[0]; break;
		case 1: direction[0] = (dir == CW)? 1:-1; direction[1] =  direction[0]; break;
		case 2: direction[0] = (dir == CW)? -1:1; direction[1] =  direction[0]; break;
		case 3: direction[0] = (dir == CW)? -1:1; direction[1] = - direction[0]; break;
	}
}


void Circular_Interpolation(uint8_t dir){
	double fx,fy,Xo,Yo,Xi,Yi,xerror,yerror,xyerror;
	int8_t direction[2];
	uint8_t xo,yo,i;
	uint32_t time = 0;
	do{
/////////////get quadrant of next point///////////////
		start_micro();
		xo = ((comp[8]-comp[0]) >= 0);    //xi-xc
		yo = ((comp[9]-comp[1]) >= 0);		//yi-yc
		i = quadrant_binrep(xo,yo);
////////////direction of each axis//////////////	
		direction_binrep(dir,i,direction);
//////////////////interpolation/////////////////////
		Xo = copysign(machine_cons[0],direction[0]);
		Xi = xi-centers[0];
		Yo = copysign(machine_cons[1],direction[1]);
		Yi = yi-centers[1];		
		fx = Xi+Xo;  //futuristic step on x
		fy = Yi+Yo;   //futuristic step on y
		fx = fx*fx;		//X sqruare
		fy = fy*fy;		//Y square
		xerror = fabs(fx+(Yi*Yi)-rsq);  //error if step on x
		yerror = fabs(fy+(Xi*Xi)-rsq);	//error if step on y
		xyerror = fabs(fx+fy-rsq); 			//error if step on x and y
		if(xerror < yerror){
			if(xerror < xyerror){
				//step on x
				xi = xi + Xo;
				steps--;
			}
			else{
				//step on x and y
				xi = xi + Xo;
				yi = yi + Yo;
				steps-=2;
			}
		}
		else{
			if(yerror < xyerror){
				//step on y
				yi = yi + Yo;
				steps--;
			}
			else{
				//step on x and y
				xi = xi + Xo;
				yi = yi + Yo;
				steps-=2;
			}
		}
		comp[8] = adjust_num(xi);  //xi
		comp[9] = adjust_num(yi);  //yi
		time = (0x00FFFFFF - current())&0x00FFFFFF;
	}while(steps);
		UART_OutDec(0,time);
}


void G02(void){
	Arc_Param_Init();
	Arc_Param_Cal(CW);
	Circular_Interpolation(CW);
}
