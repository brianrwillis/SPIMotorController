/*****************************************************************************************
 * PWM Module
 * This module obtains PWM rate values from the user interface and verifies the rate if
 * it's valid between 0 to 99 percent. It also changes the speed of the motor at a
 * voltage level.
 *
 * Nathan Gomez, 03/15/18
 ****************************************************************************************/
#include "MCUType.h"
#include "app_cfg.h"
#include "os.h"
#include "K65TWR_GPIO.h"
#include "PWM.h"

/*****************************************************************************************
 * Defined Constants
 ****************************************************************************************/

/*****************************************************************************************
* Allocate task control blocks
*****************************************************************************************/
static OS_TCB PWMTaskTCB;
/*****************************************************************************************
* Allocate task stack space
*****************************************************************************************/
static CPU_STK PWMTaskStk[APP_CFG_PWM_TASK_STK_SIZE];
/*****************************************************************************************
* Function Prototypes
*****************************************************************************************/
static void PWMTask(void *p_arg);
/*****************************************************************************************
* Private resources
*****************************************************************************************/
static OS_SEM PWMChgFlag;
static INT16U pwm_value = 0;
/*****************************************************************************************
* PWMInit()
* Initializes the clocks, the Flex Timer(FTM), and the PWM outputs. The FTM requires channel
* mode select, rising edge capture, and make the PWM channels independent. The PWM initialization
* was able to send a few pulse widths for one time only, but for a continuous time.
*
* Nathan Gomez, 03/15/18
*****************************************************************************************/
void PWMInit(void){

	OS_ERR os_err;

	/*Try dealing with the pulse width (duty cycle) and the period later*/

	SIM_SCGC3 |= SIM_SCGC3_FTM3(1);  //Enable clock in FTM3 for PWM0 and PWM3 (found in K65 Tower datasheet)
	SIM_SCGC5 |= SIM_SCGC5_PORTE(1); //Enable clock from Port E

	PORTE_PCR5 |= PORT_PCR_MUX(6);   //Enable channel 5 of Port E
	PORTE_PCR8 |= PORT_PCR_MUX(6);   //Enable channel 8 of Port E

	SIM_SOPT8 &= SIM_SOPT8_FTM3OCH0SRC(0);   //Output at pin FTM3 Channel 0 (FTM modulation)
	SIM_SOPT8 &= SIM_SOPT8_FTM3OCH3SRC(0);   //Output at pin FTM3 Channel 3 (FTM modulation)

	FTM3_MODE |= FTM_MODE_WPDIS(1); //Disable write protection
	FTM3_SC |= FTM_SC_TOF(1);     //FTM counter overflows at every clock edge
	FTM3_SC |= FTM_SC_CLKS(1);    //System clock
	FTM3_SC |= FTM_SC_CPWMS(0);   //Center-Aligned PWM Select; FTM Counter operates in Up Counting mode
	FTM3_SC |= FTM_SC_PS(2);      //Prescale Factor divide by 4

	FTM3_CNTIN |= FTM_CNTIN_INIT(0); //Counter initial value
	FTM3_CNT |= FTM_CNT_COUNT(0);    //Counter value
	FTM3_MOD |= FTM_MOD_MOD(0);      //Modulo value

	FTM3_C0V |= FTM_CnV_VAL(1); //Channel 0 value
	FTM3_C3V |= FTM_CnV_VAL(1); //Channel 3 value

	FTM3_C0SC |= FTM_CnSC_ELSB(1); //Rising edge capture at Channel 0 (input capture)
	FTM3_C0SC |= FTM_CnSC_ELSA(0);

	FTM3_C3SC |= FTM_CnSC_ELSB(1); //Rising edge capture at Channel 3 (input capture)
    FTM3_C3SC |= FTM_CnSC_ELSA(0);

    FTM3_C0SC |= FTM_CnSC_MSB(1);  //Channel mode select on Channel 0 (for Edge-Aligned PWM)

    FTM3_C3SC |= FTM_CnSC_MSB(1);  //Channel mode select on Channel 3 (for Edge-Aligned PWM)

    FTM3_COMBINE |= FTM_COMBINE_COMBINE0(0); //Channels 0 and 3 are independent
    FTM3_COMBINE |= FTM_COMBINE_DECAPEN0(0); //Disable dual edge capture

    FTM3_MODE |= FTM_MODE_FTMEN(1);    //Enable FTM

    FTM3_QDCTRL |= FTM_QDCTRL_QUADEN(0); //Quadrature Decoder Mode disabled

    OSSemCreate(&PWMChgFlag, "PWM Change Flag Semaphore", 0, &os_err); /*Create PWM change semaphore flag*/
    while(os_err != OS_ERR_NONE){			/*Error Trap*/
    }

	OSTaskCreate(&PWMTaskTCB,                  /*Create the time task*/
	            "PWM Task ",
	            PWMTask,
	            (void *) 0,
	            APP_CFG_PWM_TASK_PRIO,
	            &PWMTaskStk[0],
	            (APP_CFG_PWM_TASK_STK_SIZE / 10u),
	            APP_CFG_PWM_TASK_STK_SIZE,
	            0,
	            0,
	            (void *) 0,
	            (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
	            &os_err);
	while(os_err != OS_ERR_NONE){               /*Error Trap*/
	}
}

/*****************************************************************************************
* PWMTask()
* Obtains the PWM rate value from the user interface and sends it to the PWM rate block for
* verification.
*
* Nathan Gomez, 03/15/18
*****************************************************************************************/
void PWMTask(void *p_arg){

	OS_ERR os_err;

	(void)p_arg;

	OSSemPend(&PWMChgFlag,0,OS_OPT_PEND_BLOCKING,(void*)0,&os_err); /*Send a semaphore signal to run the time display task*/
    while(os_err != OS_ERR_NONE){           /* Error Trap */
    }
}
/*****************************************************************************************
* PWMRate()
* Checks if the PWM rate is within the range between 1 and 99 percent. If the PWM rate is
* 0 percent, it's ignored.
*
* Return: pwm_value
*
* Nathan Gomez, 03/15/18
*****************************************************************************************/
INT16U PWMRate(void){

	OS_ERR os_err;

	OSSemPost(&PWMChgFlag,OS_OPT_POST_1,&os_err);	/*Signal the time semaphore flag to call the time display task*/ //>>>should be after mutex post
    while(os_err != OS_ERR_NONE){    /*Error Trap*/
    }
    return pwm_value;
}
