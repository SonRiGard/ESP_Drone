#include <math.h>

typedef struct
{
	float ampha , betal, gamma, delta;
	volatile float u , u1, u2;
	volatile float e , e1, e2;//error
	float KP, KI, KD;
    volatile float set_point;

	volatile float set_point_angle;
	volatile float cur_point_angle;
	volatile float err_angle;		

	float q[5];
} PID_t;

void parameter_calculation (PID_t *DataStruct);
void setK (double KP, double KI, double KD, PID_t *DataStruct);
void get_out (PID_t *DataStruct, double set_Angle, double current_Angle);