#include "PID.h"
extern float deltaT_PID;

void parameter_calculation (PID_t *DataStruct){
	DataStruct->ampha = 2*deltaT_PID*(DataStruct->KP)+(DataStruct->KI*deltaT_PID*deltaT_PID)+2*(DataStruct->KD);
	DataStruct->betal = deltaT_PID*deltaT_PID*(DataStruct->KI) - 4*(DataStruct->KD) - 2*deltaT_PID*(DataStruct->KP);
	DataStruct->gamma = 2*(DataStruct->KD);
	DataStruct->delta = 2*deltaT_PID;	
}

void setK (double KP, double KI, double KD, PID_t *DataStruct){
	DataStruct->KP = KP;
	DataStruct->KI = KI;
	DataStruct->KD = KD;
	DataStruct->e  = 0;
	DataStruct->e1 = 0;
	DataStruct->e2 = 0;
	DataStruct->u  = 0; 
	DataStruct->u1 = 0;
}

void get_out (PID_t *DataStruct, double set_Angle, double current_Angle){
	DataStruct->e2 = DataStruct->e1;
	DataStruct->e1 = DataStruct->e;
	DataStruct->e  = set_Angle - current_Angle;
	DataStruct->u1 = DataStruct->u;
	DataStruct->u = ((DataStruct->ampha*DataStruct->e)+(DataStruct->betal*DataStruct->e1)
									+(DataStruct->gamma*DataStruct->e2)+ DataStruct->delta*DataStruct->u1)/(DataStruct->delta);
}