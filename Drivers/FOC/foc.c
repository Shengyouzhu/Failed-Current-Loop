#include "foc.h"
#include "CurrentLoop.h"

#define MOTOR_VBUS    12.0f
#define VOLTAGE_LIMIT CURRENT_LOOP_VOLTAGE_LIMIT

extern int Sensor_DIR;    //传感器方向
extern int Motor_PP;  

///////////
volatile float debug_iq = 0.0f;
volatile float debug_uq = 0.0f;
volatile float debug_ud = 0.0f;
volatile uint32_t debug_foc_count = 0;

/////////////

/*
	克拉克逆变换
	功能：将2维的电信号转换为3个相位的电信号
	输入：ialhpa ibeta
	输出：ia,ib,ic
*/
void Inv_Clark_transformer(FOC_Vector_t *cu )
{
		cu->a = cu->alpha;
		cu->b = (1.732f*cu->beta/2.0f)-(cu->alpha/2.0f);
		cu->c = -(1.732f*cu->beta/2.0f)-(cu->alpha/2.0f);
}

/*
	克拉克变换
	功能：将3个相位的电信号转换为2维的电信号
	输入：ia,ib,ic
	输出：ialhpa ibeta
*/
void Clark_transformer(FOC_Vector_t *cu )
{
    cu->alpha = (2.0f / 3.0f) *
                 (cu->a - 0.5f * cu->b
                         - 0.5f * cu->c);
    cu->beta = 0.577350269f *
                (cu->b - cu->c);
}

/*
	帕克变换
	功能：将2维的电信号转换为旋转的矢量
	输入：ialhpa ibeta
	输出：id iq theta_e
*/
void Pake_transformer(FOC_Vector_t *cu,float theta_e)
{
	  float sin_theta = sinf(theta_e);
    float cos_theta = cosf(theta_e);

    cu->d = cu->alpha * cos_theta
           + cu->beta  * sin_theta;

    cu->q = -cu->alpha * sin_theta
           + cu->beta  * cos_theta;
}

/*
	帕克逆变换
	功能：将旋转的矢量转换为2维的电信号
	输入：id iq theta_e
	输出：ialhpa ibeta
*/
void Inv_Pake_transformer(FOC_Vector_t *cu,float theta_e)
{
	  float sin_theta = sinf(theta_e);
    float cos_theta = cosf(theta_e);

     cu->alpha= cu->d * cos_theta - cu->q * sin_theta;
     cu->beta = cu->q * cos_theta + cu->d * sin_theta;
}


void FOC_Init(void)
{
	CurrentLoop_Init();
}



//施加固定方向的电压矢量
void FOC_SetAlignVoltage(float ud, float theta)
{
    FOC_Vector_t voltage = {0};

    voltage.d = ud;
    voltage.q = 0.0f;

    Inv_Pake_transformer(&voltage, theta);
    Inv_Clark_transformer(&voltage);

    SVPWM_SetPhaseVoltages(
        voltage.a,
        voltage.b,
        voltage.c,
        12.0f
    );
}

float electrical_offset = 0.0f;
//电角度校准
void FOC_CalibrateZero(float mechanical_angle)
{
    electrical_offset =
        Sensor_DIR * Motor_PP * mechanical_angle;

    electrical_offset = fmodf(
        electrical_offset, 2.0f * 3.1415926f
    );

    if (electrical_offset < 0.0f)
        electrical_offset += 2.0f * 3.1415926f;
}


void FOC_MotorControl(float ia, float ib, float ic,
                      float theta_e, float dt)
{
    FOC_Vector_t voltage = {0};

    // 1. 电流环：计算 Ud、Uq
    CurrentLoop_Update(
        ia, ib, ic, theta_e, dt, &voltage
    );

    // 2. 电压矢量限幅
    float magnitude = sqrtf(
        voltage.d * voltage.d +
        voltage.q * voltage.q
    );

    if (magnitude > VOLTAGE_LIMIT)
    {
        float scale = VOLTAGE_LIMIT / magnitude;
        voltage.d *= scale;
        voltage.q *= scale;
    }

    // 3. 逆Park变换
    Inv_Pake_transformer(&voltage, theta_e);

    // 4. 逆Clarke变换
    Inv_Clark_transformer(&voltage);

    // 5. SVPWM输出
    SVPWM_SetPhaseVoltages(
        voltage.a,
        voltage.b,
        voltage.c,
        MOTOR_VBUS
    );





// 记录PI输出的Uq
debug_uq = voltage.q;
debug_ud = voltage.d;

// 记录控制函数执行次数
debug_foc_count++;
}




