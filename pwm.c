
/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/

//#include "bsw.h"
#include "pwm.h"
/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
IfxGtm_Tom_Pwm_Config motor_fl_tomConfig;                  /* Timer configuration structure                */
IfxGtm_Tom_Pwm_Driver motor_fl_tomDriver;                  /* Timer Driver structure                       */

IfxGtm_Tom_Pwm_Config motor_fr_tomConfig;                  /* Timer configuration structure                */
IfxGtm_Tom_Pwm_Driver motor_fr_tomDriver;                  /* Timer Driver structure                       */



static int last_direction[2] = {FORWARD,FORWARD};

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
void initMotor(void)
{
    /*********** PWM INIT *************/
    IfxGtm_enable(&MODULE_GTM);                                     /* Enable GTM                                   */
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_FXCLK);   /* Enable the FXU clock                         */

    /*MOTOR FL*/

    /* DIGITAL INIT FOR MOTOR DIRECTION */
    IfxPort_setPinModeOutput(IN1_FL.port, IN1_FL.pinIndex, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(IN2_FL.port, IN2_FL.pinIndex, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinHigh(IN1_FL.port, IN1_FL.pinIndex);
    IfxPort_setPinLow(IN2_FL.port, IN2_FL.pinIndex);

    /* Initialize the configuration structure with default parameters */
    IfxGtm_Tom_Pwm_initConfig(&motor_fl_tomConfig, &MODULE_GTM);

    motor_fl_tomConfig.tom = MOTOR_FL.tom;                                   /* Select the TOM depending on the LED          */
    motor_fl_tomConfig.tomChannel = MOTOR_FL.channel;                        /* Select the channel depending on the LED      */
    motor_fl_tomConfig.period = MOTOR_PERIOD;                                /* Set the timer period                         */
    motor_fl_tomConfig.dutyCycle = MOTOR_DUTY_DEFAULT;
    motor_fl_tomConfig.pin.outputPin = &MOTOR_FL;                            /* Set the LED port pin as output               */
    motor_fl_tomConfig.synchronousUpdateEnabled = TRUE;                      /* Enable synchronous update                    */




    /*MOTOR FR*/

    /* DIGITAL INIT FOR MOTOR DIRECTION */
    IfxPort_setPinModeOutput(IN1_FR.port, IN1_FR.pinIndex, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(IN2_FR.port, IN2_FR.pinIndex, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinHigh(IN1_FR.port, IN1_FR.pinIndex);
    IfxPort_setPinLow(IN2_FR.port, IN2_FR.pinIndex);

    /* Initialize the configuration structure with default parameters */
    IfxGtm_Tom_Pwm_initConfig(&motor_fr_tomConfig, &MODULE_GTM);

    motor_fr_tomConfig.tom = MOTOR_FR.tom;                                   /* Select the TOM depending on the LED          */
    motor_fr_tomConfig.tomChannel = MOTOR_FR.channel;                        /* Select the channel depending on the LED      */
    motor_fr_tomConfig.period = MOTOR_PERIOD;                                /* Set the timer period                         */
    motor_fr_tomConfig.dutyCycle = MOTOR_DUTY_DEFAULT;
    motor_fr_tomConfig.pin.outputPin = &MOTOR_FR;                            /* Set the LED port pin as output               */
    motor_fr_tomConfig.synchronousUpdateEnabled = TRUE;                      /* Enable synchronous update                    */




    /////////////////////////

    IfxGtm_Tom_Pwm_Driver* drivers[MOTOR_COUNT];
    drivers[0] = &motor_fl_tomDriver;
    drivers[1] = &motor_fr_tomDriver;

    IfxGtm_Tom_Pwm_Config* configs[MOTOR_COUNT];
    configs[0] = &motor_fl_tomConfig;
    configs[1] = &motor_fr_tomConfig;

    _init_pwms(drivers, configs, MOTOR_COUNT);

    return;
}
void _init_pwms(IfxGtm_Tom_Pwm_Driver* drivers[] , IfxGtm_Tom_Pwm_Config* configs[], int pwm_count)
{
    //하나의 tom모듈 안에 있는 채널들만 사용하는 경우에 유효함 리팩토링 필요.
    uint8 i;
    for(i = 0; i < pwm_count; i++)
    {
        drivers[i]->gtm      = configs[i]->gtm;
        drivers[i]->tomIndex = configs[i]->tom;

        Ifx_GTM_TOM *tomSFR = &configs[i]->gtm->TOM[configs[i]->tom];
        drivers[i]->tom        = tomSFR;
        drivers[i]->tomChannel = configs[i]->tomChannel;

        if (configs[i]->tomChannel <= 7)
        {
            drivers[i]->tgc[0] = IfxGtm_Tom_Ch_getTgcPointer(drivers[i]->tom, 0);
            drivers[i]->tgc[1] = IfxGtm_Tom_Ch_getTgcPointer(drivers[i]->tom, 1);
        }
        else
        {
            drivers[i]->tgc[0] = IfxGtm_Tom_Ch_getTgcPointer(drivers[i]->tom, 1);
            drivers[i]->tgc[1] = NULL_PTR; /* NOTE currently no concatenation between TOMs */
        }

        /* Initialize the timer part */
        if (configs[i]->synchronousUpdateEnabled == 1)
        {
            drivers[i]->tgc[0]->GLB_CTRL.U |= IfxGtm_Tom_Tgc_buildFeatureForChannel(configs[i]->tomChannel, TRUE, IFX_GTM_TOM_TGC0_GLB_CTRL_UPEN_CTRL0_OFF);
        }

        IfxGtm_Tom_Ch_setClockSource(tomSFR, configs[i]->tomChannel, configs[i]->clock);

        //IfxGtm_Tom_Tgc_setChannelForceUpdate
        uint32 regEnable, regReset;

        regEnable        = IfxGtm_Tom_Tgc_buildFeatureForChannel(configs[i]->tomChannel, TRUE, IFX_GTM_TOM_TGC0_FUPD_CTRL_FUPD_CTRL0_OFF);
        regReset         = IfxGtm_Tom_Tgc_buildFeatureForChannel(configs[i]->tomChannel, TRUE, IFX_GTM_TOM_TGC0_FUPD_CTRL_RSTCN0_CH0_OFF);

        drivers[i]->tgc[0]->FUPD_CTRL.U |= regEnable | (regReset << 16);
        /////


        IfxGtm_Tom_Ch_setSignalLevel(tomSFR, configs[i]->tomChannel, configs[i]->signalLevel);

        if (configs[i]->pin.outputPin != NULL_PTR)
        {
            IfxGtm_PinMap_setTomTout(configs[i]->pin.outputPin, configs[i]->pin.outputMode, configs[i]->pin.padDriver);
        }

        // enable and initialise interrupts if chosen
        if ((configs[i]->interrupt.ccu0Enabled == 1) || (configs[i]->interrupt.ccu1Enabled == 1))
        {
            IfxGtm_Tom_Ch_setNotification(tomSFR, configs[i]->tomChannel, configs[i]->interrupt.mode, configs[i]->interrupt.ccu0Enabled, configs[i]->interrupt.ccu1Enabled);

            volatile Ifx_SRC_SRCR *src;
            src = IfxGtm_Tom_Ch_getSrcPointer(configs[i]->gtm, configs[i]->tom, configs[i]->tomChannel);
            IfxSrc_init(src, configs[i]->interrupt.isrProvider, configs[i]->interrupt.isrPriority);
            IfxSrc_enable(src);
        }

        if (configs[i]->synchronousUpdateEnabled == 1)
        {
            IfxGtm_Tom_Ch_setCompareZeroShadow(tomSFR, configs[i]->tomChannel, configs[i]->period);
            IfxGtm_Tom_Ch_setCompareOneShadow(tomSFR, configs[i]->tomChannel, configs[i]->dutyCycle);

            drivers[i]->tgc[0]->GLB_CTRL.U = 1 << IFX_GTM_TOM_TGC0_GLB_CTRL_HOST_TRIG_OFF;
        }
        else
        {
            IfxGtm_Tom_Ch_setCompareZero(tomSFR, configs[i]->tomChannel, configs[i]->period);
            IfxGtm_Tom_Ch_setCompareOne(tomSFR, configs[i]->tomChannel, configs[i]->dutyCycle);
        }
    }

    //TOM0 채널 1,2,3,4 사용
    uint16 enableMask = 0;
    for(i = 0; i < pwm_count; i++)
    {
        enableMask |= (0x1 << (configs[i]->tomChannel));
    }

    uint16 disableMask = 0;

    //enable channel, *같은 tom모듈에 있는 채널만 쓰기 때문에 drivers[0]->tgc[0]는 하드코딩
    IfxGtm_Tom_Tgc_enableChannels(drivers[0]->tgc[0],enableMask, disableMask, TRUE);
    //enable channelOutput *같은 tom모듈에 있는 채널만 쓰기 때문에 drivers[0]->tgc[0]는 하드코딩
    IfxGtm_Tom_Tgc_enableChannelsOutput(drivers[0]->tgc[0],enableMask, disableMask, TRUE);

    //채널들 enable설정한 것 적용하는 트리거
    IfxGtm_Tom_Tgc_trigger(drivers[0]->tgc[0]);

    return;
}

void motor_go(motor_index_t motor_index){
    set_motor_dir(motor_index,last_direction[motor_index]);

    return;
}

void motor_stop(motor_index_t motor_index)
{
    set_motor_dir(motor_index, last_direction[motor_index]);
    switch(motor_index)
    {
        case INDEX_FL:
            IfxPort_setPinLow(IN1_FL.port, IN1_FL.pinIndex);
            IfxPort_setPinLow(IN2_FL.port, IN2_FL.pinIndex);
            break;
        case INDEX_FR:
            IfxPort_setPinLow(IN1_FR.port, IN1_FR.pinIndex);
            IfxPort_setPinLow(IN2_FR.port, IN2_FR.pinIndex);
            break;
        default:
            break;
    };

    return;
}
void set_motor_dir(int motor_index, int forward){ //idx 0 : left, 1 : right
    switch(motor_index)
        {
            case INDEX_FL:
                if(forward){
                    IfxPort_setPinLow(IN1_FL.port, IN1_FL.pinIndex);
                    IfxPort_setPinHigh(IN2_FL.port, IN2_FL.pinIndex);
                }
                else{
                    IfxPort_setPinHigh(IN1_FL.port, IN1_FL.pinIndex);
                    IfxPort_setPinLow(IN2_FL.port, IN2_FL.pinIndex);
                }
                break;
            case INDEX_FR:
                if(forward){
                    IfxPort_setPinLow(IN1_FR.port, IN1_FR.pinIndex);
                    IfxPort_setPinHigh(IN2_FR.port, IN2_FR.pinIndex);
                }
                else{
                    IfxPort_setPinHigh(IN1_FR.port, IN1_FR.pinIndex);
                    IfxPort_setPinLow(IN2_FR.port, IN2_FR.pinIndex);
                }
                break;

            default:
                break;
        };

    last_direction[motor_index] = forward;

    return;
}
//0~100
void set_motor_power(motor_index_t motor_index, double dutyPercent)
{
    //dutyPercent
    // 0 : 정지가 아니라 최소 파워 의미
    // 100 : 최대 파워
    uint32 dutyCycle = 0;

    if(dutyPercent < 0 || dutyPercent > 100)
    {
        //do nothing
    }
    else
    {
        //calculate dutycycle from Percent
        dutyCycle = MOTOR_DUTY_MIN + (uint32)(MOTOR_DUTY_UNIT * dutyPercent);

        switch(motor_index)
        {
            case INDEX_FL:
                _setDutyCycle(&motor_fr_tomDriver, &motor_fr_tomConfig, dutyCycle);
                break;
            case INDEX_FR:
                _setDutyCycle(&motor_fl_tomDriver, &motor_fl_tomConfig, dutyCycle);
                break;
            default:
                break;
        };
    }

    return;
}
void _setDutyCycle(IfxGtm_Tom_Pwm_Driver* driver , IfxGtm_Tom_Pwm_Config* config, uint32 _dutyCycle)
{
    if(driver == NULL || config == NULL)
    {
        return;
    }

    //disable update
    driver->tgc[0]->GLB_CTRL.U |= IfxGtm_Tom_Tgc_buildFeatureForChannel(config->tomChannel, FALSE, IFX_GTM_TOM_TGC0_GLB_CTRL_UPEN_CTRL0_OFF);

    //set duty
    config->dutyCycle = _dutyCycle;
    IfxGtm_Tom_Ch_setCompareOneShadow(&config->gtm->TOM[config->tom], config->tomChannel, config->dutyCycle);

    //enable update
    driver->tgc[0]->GLB_CTRL.U |= IfxGtm_Tom_Tgc_buildFeatureForChannel(config->tomChannel, TRUE, IFX_GTM_TOM_TGC0_GLB_CTRL_UPEN_CTRL0_OFF);

    return;
}
