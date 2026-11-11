#include "AS5600.h"
#include <math.h>

/* Light LPF on the unwrapped mechanical angle; alpha = 0.8 at 1 ms. */
#define ANGLE_FILTER_TAU_S 0.00025f


int Sensor_DIR=1;    //传感器方向
int Motor_PP=7;    //电机极对数
extern float electrical_offset;

float pai=3.1415926f;
float last_measurement;
float total_angle = 0.0f;
static float filtered_angle = 0.0f;
uint8_t first_read=1;

float FOC_GetElectricalAngle(float mechanical_angle)
{
    float theta_e =
        Sensor_DIR * Motor_PP * mechanical_angle
        - electrical_offset;

    theta_e = fmodf(theta_e, 2*pai);

    if (theta_e < 0.0f)
        theta_e += 2*pai;

    return theta_e;
}


uint8_t as5600_read_byte(float *angle ,float *Mechanical_perspective,float dt){
	uint8_t data[2];
	float difference; 
	float measurement;
	HAL_StatusTypeDef status =HAL_I2C_Mem_Read(&hi2c2,AS5600_ADDR,AS5600_ANGLE_REG,I2C_MEMADD_SIZE_8BIT,data,2,10);
	if(status != HAL_OK)
	{
		return status;
	}

	uint16_t raw_data = (((uint16_t)data[0]<<8)|data[1])&0x0FFF;

	
	measurement = raw_data * 2*pai/4096.0f;



	if(first_read)
	{
		last_measurement = measurement;
		first_read = 0;

		filtered_angle = total_angle;
		*angle = filtered_angle;
		return HAL_OK;
	}

	difference = measurement-last_measurement ;      //将结果为2pai的数据改为结果为累加数据，适合pid控制
	if(difference>pai)
	{
		difference -= (2*pai);
	}
	if(difference<-pai)
	{
		difference += (2*pai);
	}

	total_angle += difference;

	last_measurement =measurement;

	float angle_alpha = dt / (ANGLE_FILTER_TAU_S + dt);
	filtered_angle += angle_alpha * (total_angle - filtered_angle);
	*angle = filtered_angle;
	
	//计算机械角速度
	float raw_v = difference / dt;

	float alpha = dt / (0.02f + dt);

	*Mechanical_perspective += alpha * (raw_v - *Mechanical_perspective);
	
	return HAL_OK;
}



