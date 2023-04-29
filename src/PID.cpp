#include "PID.h"
extern float deltaT_PID;

void parameter_calculation (PID_t *DataStruct){
	// DataStruct->ampha = 2*deltaT_PID*(DataStruct->KP)+(DataStruct->KI*deltaT_PID*deltaT_PID)+2*(DataStruct->KD);
	// DataStruct->betal = deltaT_PID*deltaT_PID*(DataStruct->KI) - 4*(DataStruct->KD) - 2*deltaT_PID*(DataStruct->KP);
	// DataStruct->gamma = 2*(DataStruct->KD);
	// DataStruct->delta = 2*deltaT_PID;
	float K = DataStruct->KP;
	float tI= DataStruct->KP/DataStruct->KI;
	float tD= DataStruct->KD/DataStruct->KP;

	DataStruct->q[0]=K*(1+tD/deltaT_PID);
	DataStruct->q[1]=-K*(1+2*tD/deltaT_PID-deltaT_PID/tI);
	DataStruct->q[2]=K*tD/deltaT_PID;
	DataStruct->q[3]=1;
	DataStruct->q[4]=0;
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
	DataStruct->u2 = 0;
}

void get_out (PID_t *DataStruct, double set_Angle, double current_Angle){
	DataStruct->e2 = DataStruct->e1;
	DataStruct->e1 = DataStruct->e;
	DataStruct->e  = set_Angle - current_Angle;
	DataStruct->u2 = DataStruct->u1;
	DataStruct->u1 = DataStruct->u;
	DataStruct->u=DataStruct->q[3]*DataStruct->u+DataStruct->q[4]*DataStruct->u1+DataStruct->q[0]*DataStruct->e+DataStruct->q[1]*DataStruct->e1+DataStruct->q[2]*DataStruct->e2;
}