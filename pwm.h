#ifndef DRIVER_MOTOR_H_
#define DRIVER_MOTOR_H_
/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "Ifx_Types.h"
#include "IfxGtm_Tom_Pwm.h"

#include "IfxPort.h"
#include "IfxPort_PinMap.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
//각 모터의 en과 연결되는 pwm
#define MOTOR_FL IfxGtm_TOM0_2_TOUT104_P10_2_OUT
#define MOTOR_FR IfxGtm_TOM0_1_TOUT103_P10_1_OUT

#define MOTOR_COUNT 2

#define FORWARD 1
#define BACKWARD 0

// 정속
// 3100
#define MOTOR_PERIOD 50000
#define MOTOR_DUTY_MAX 40000//45000
#define MOTOR_DUTY_MIN 15000//10//35000//42000
#define MOTOR_DUTY_DEFAULT 0
#define MOTOR_DUTY_UNIT ((MOTOR_DUTY_MAX - MOTOR_DUTY_MIN) / 100)
//모터 출력은 0-100
//정속주행시 출력 25


//방향 설정용 디지털output 핀, 모터당 2개 - IN1, IN2
#define IN1_FL IfxPort_P14_0
#define IN2_FL IfxPort_P14_1

#define IN1_FR IfxPort_P15_6
#define IN2_FR IfxPort_P00_0

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/
typedef enum motor_index{
    INDEX_FL = 0,
    INDEX_FR = 1,
}motor_index_t;

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
void initMotor(void);
void _init_pwms(IfxGtm_Tom_Pwm_Driver* drivers[] , IfxGtm_Tom_Pwm_Config* configs[], int pwm_count);

//idx 0 left, 1 right :: forward 인자값 0 -> backward
void set_motor_dir(int motor_index, int forward);

void motor_stop(motor_index_t motor_index);
void motor_go(motor_index_t motor_index);

//파워는 0~100사이
void set_motor_power(motor_index_t motor_index, double dutyPercent);
void _setDutyCycle(IfxGtm_Tom_Pwm_Driver* driver , IfxGtm_Tom_Pwm_Config* config, uint32 _dutyCycle);

#endif /* DRIVER_MOTOR_H_ */
