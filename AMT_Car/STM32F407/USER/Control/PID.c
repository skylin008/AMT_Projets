#include "PID.h"
#include "BTModule.h"



void update_PID_INC(PID_typedef* PID,float target,float input)   //增量式PID
{
  float Error=target-input;           //误差=期望-当前
 
	float pTerm=PID->Kp*(Error-PID->lastError);   //比例项
	float iTerm=PID->Ki*(Error+PID->lastError)/2.0f;  //积分项
	float dTerm=PID->Kd*((Error-PID->lastError)-(PID->lastError-PID->prevError));  //微分项
	
	PID->Output+=pTerm+iTerm+dTerm;
	//
  PID->prevError=PID->lastError;
  PID->lastError=Error;	
}
