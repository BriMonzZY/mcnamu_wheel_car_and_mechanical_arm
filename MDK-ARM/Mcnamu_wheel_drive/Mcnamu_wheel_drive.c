#include "Mcnamu_wheel_drive.h"



#define DRV  /* 驱动类型选择 DRV HIP*/


/* 速度闭环的位置式PID */
double PIDCalc(PID *pp, double NextPoint)   
{
	double dError,                              /* 当前微分 */
				 Error;                               /* 偏差 */
	Error = pp->SetPoint - NextPoint;           /* 偏差值=设定值-输入值（当前值）*/
	
	/* 积分限幅 */
	if(pp->SumError > 4000) pp->SumError = 4000;
	if(pp->SumError < -4000) pp->SumError = -4000;
	
	pp->SumError += Error;                      /* 积分=积分+偏差  --偏差的累加 */
	dError = pp->LastError - pp->PrevError;     /* 当前微分 = 最后误差 - 之前误差 */
	pp->PrevError = pp->LastError;              /* 更新“之前误差” */
	pp->LastError = Error;                      /* 更新“最后误差” */
	return (pp->Kp * Error                      /* 比例项 = 比例常数 * 偏差 */
			+   pp->Ki *  pp->SumError              /* 积分项 = 积分常数 * 误差积分 */
			+   pp->Kd * dError                     /* 微分项 = 微分常数 * 当前微分 */
				 );
}





/* 电机方向转速驱动 */
#ifdef HIP

void wheel1_speed_set(float speed)
{
	if(speed >= 100) speed = 100;
	else if(speed < -100) speed = -100;
	speed = ((double)speed/100.0)*600;
	if(speed >= 0){
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (int)speed);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
	}
	else if(speed < 0){
		speed *= -1;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (int)speed);
	}
}

void wheel2_speed_set(float speed)
{
	if(speed >= 100) speed = 100;
	else if(speed <= -100) speed = -100;
	speed = ((double)speed/100.0)*600;
	if(speed >= 0){
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (int)speed);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
	}
	else {
		speed *= -1;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, (int)speed);
	}
}

void wheel3_speed_set(float speed)
{
	if(speed >= 100) speed = 100;
	else if(speed < -100) speed = -100;
	speed = ((double)speed/100.0)*600;
	if(speed >= 0){
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, (int)speed);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
	}
	else if(speed < 0){
		speed *= -1;
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, (int)speed);
	}
}

void wheel4_speed_set(float speed)
{
	if(speed >= 100) speed = 100;
	else if(speed < -100) speed = -100;
	speed = ((double)speed/100.0)*600;
	if(speed >= 0){
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, (int)speed);
		__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
	}
	else if(speed < 0){
		speed *= -1;
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, (int)speed);
	}
}

#endif



#ifdef DRV

/*	满速1000
		125时编码器值100
*/
void wheel1_speed_set(float speed)
{
	speed = -speed;  /* 正反方向调节 */
	if(speed >= 1000) speed = 1000;
	else if(speed < -1000) speed = -1000;
	if(speed >= 0){
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (int)speed);
		HAL_GPIO_WritePin(WHEEL1_DIR_GPIO_Port, WHEEL1_DIR_Pin, GPIO_PIN_SET);
	}
	else if(speed < 0){
		speed *= -1;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (int)speed);
		HAL_GPIO_WritePin(WHEEL1_DIR_GPIO_Port, WHEEL1_DIR_Pin, GPIO_PIN_RESET);
	}
}

void wheel2_speed_set(float speed)
{
	speed = -speed;  /* 正反方向调节 */
	if(speed >= 1000) speed = 1000;
	else if(speed <= -1000) speed = -1000;
	if(speed >= 0){
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (int)speed);
		HAL_GPIO_WritePin(WHEEL2_DIR_GPIO_Port, WHEEL2_DIR_Pin, GPIO_PIN_SET);
	}
	else {
		speed *= -1;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (int)speed);
		HAL_GPIO_WritePin(WHEEL2_DIR_GPIO_Port, WHEEL2_DIR_Pin, GPIO_PIN_RESET);
	}
}

void wheel3_speed_set(float speed)
{
	speed = -speed;  /* 正反方向调节 */
	if(speed >= 1000) speed = 1000;
	else if(speed < -1000) speed = -1000;
	if(speed >= 0){
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, (int)speed);
		HAL_GPIO_WritePin(WHEEL3_DIR_GPIO_Port, WHEEL3_DIR_Pin, GPIO_PIN_SET);
	}
	else if(speed < 0){
		speed *= -1;
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, (int)speed);
		HAL_GPIO_WritePin(WHEEL3_DIR_GPIO_Port, WHEEL3_DIR_Pin, GPIO_PIN_RESET);
	}
}

void wheel4_speed_set(float speed)
{
	speed = -speed;  /* 正反方向调节 */
	if(speed >= 1000) speed = 1000;
	else if(speed < -1000) speed = -1000;
	if(speed >= 0){
		__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, (int)speed);
		HAL_GPIO_WritePin(WHEEL4_DIR_GPIO_Port, WHEEL4_DIR_Pin, GPIO_PIN_SET);
	}
	else if(speed < 0){
		speed *= -1;
		__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, (int)speed);
		HAL_GPIO_WritePin(WHEEL4_DIR_GPIO_Port, WHEEL4_DIR_Pin, GPIO_PIN_RESET);
	}
}

#endif


#define pi 3.14159f

/* 全向速度解算 */
void all_directions(int v, int angle, int angular_velocity)
{
	/* 速度及偏转的角度和自传的角速度 */
	int vw1, vw2, vw3, vw4;  /* 最终求得的轮子的转速 */
	int vtx, vty;

	/* 当角速度是0的时候(无自旋的时候) */
	vtx = v*cos(pi*angle/180);
	vty = v*sin(pi*angle/180);
	vw1 = vty - vtx;
	vw2 = vty + vtx;
	vw3 = vty - vtx;
	vw4 = vty + vtx;
	
	wheel_speed_1.SetPoint = vw1;
	wheel_speed_2.SetPoint = vw2;
	wheel_speed_3.SetPoint = vw3;
	wheel_speed_4.SetPoint = vw4;
	
#if 0
	wheel1_speed_set(vw1);
	wheel2_speed_set(vw2);
	wheel3_speed_set(vw3);
	wheel4_speed_set(vw4);
#endif
	
	printf("%d %d %d %d\n", vw1, vw2, vw3, vw4);
}


