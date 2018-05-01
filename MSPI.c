/*****************************************************************************************
* MSPI.c - SPI Operated Motor Controller
* 		   A uC/OS program that controls a motor using a MC33879 via SPI.
*          Utilliazes PWM to control the speed of the motor.
*          Written for K65 Tower Board.
*
* 03/04/2018, Rod Mesecar
*****************************************************************************************/
#include "MCUType.h"
#include "app_cfg.h"
#include "os.h"
#include "K65TWR_GPIO.h"
#include "uCOSKey.h"
#include "LcdLayered.h"
#include "SPI.h"
#include "PWM.h"

/*****************************************************************************************
 * Defined Constants
 ****************************************************************************************/
#define UI_TASK_MSG_Q_SIZE 0x5U // Message Queue Size for UI task

 // Messages Codes
#define NO_FAULT 0x00U // Message from MC33879 indicating no errors
#define EMERGENCY_STOP 0x0000U // Writing this to the MC33879 disconnects all outputs
// **NOTE** : Upper 8 bits are used to Control some of the Fault detection, may want to change the value. See data sheet of MC33879 for details.

 // UI Positions
#define STATUS_ROW 1U // Row 1
#define FIRST_COL 1U // Col 1
#define UI_ROW 2U // Row 2
#define FAULT_MSG_OUT_COLLEM 8U // Location where the Specific output in fault will be displayed on LCD
#define UI_PWM_MSG_COL 10U // Location where the PWM level is displayed on LCD
#define UI_OUT_COL 5U // Location of where active/setting output is on LCD
#define UI_PWM_WRITE 14 // Where the PWM level is written while setting the PWM
#define UI_PWM_TENS 15u // Users cursor location for setting PWM (Tens place)
#define UI_PWM_ONES 16u // Users cursor location for setting PWM (Ones place)

 // Values relating to cursor functions from uCOSKey
#define CURSOR_ON 1
#define CURSOR_OFF 0
#define CURSOR_BLINK 1

// Masks for setting/reading SPI Messages
#define OUTPUT_ONE_MASK 1U
#define OUTPUT_TWO_MASK 2U
#define OUTPUT_THR_MASK 4U
#define OUTPUT_FOU_MASK 8U
#define OUTPUT_FIV_MASK 16U
#define OUTPUT_SIX_MASK 32U
#define OUTPUT_SEV_MASK 64U
#define OUTPUT_EGT_MASK 128U

// Strings to display on the LCD
#define OUT_ONE_MSG "OUT1"
#define OUT_TWO_MSG "OUT2"
#define OUT_THR_MSG "OUT3"
#define OUT_FOU_MSG "OUT4"
#define OUT_FIV_MSG "OUT5"
#define OUT_SIX_MSG "OUT6"
#define OUT_SEV_MSG "OUT7"
#define OUT_EGT_MSG "OUT8"
#define MULIT_FAULT_MSG "Many" // For when there are more than one fault
#define FAULT_MSG "Fault: "
#define SET_MSG "SET:"
#define PWM_MSG "PWM%"
#define NO_OUTPUT_MSG "Outputs Off"

/*****************************************************************************************
* Allocate task control blocks
*****************************************************************************************/
static OS_TCB AppTaskStartTCB;
static OS_TCB UITaskTCB;
static OS_TCB UISPISrvTaskTCB;
static OS_TCB UIKeySrvTaskTCB;

/*****************************************************************************************
* Allocate task stack space
*****************************************************************************************/
static CPU_STK AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];
static CPU_STK UITaskStk[APP_CFG_UITASK_STK_SIZE];
static CPU_STK UISPISrvTaskStk[APP_CFG_UISPISRV_TASK_STK_SIZE];
static CPU_STK UIKeySrvTaskStk[APP_CFG_UIKEYSRV_TASK_STK_SIZE];

/*****************************************************************************************
* Mutexes and Semaphores
*****************************************************************************************/
static OS_MUTEX PwmRateKey; // Consider relocating to PWM module
static OS_SEM NewPwmRate; // Consider relocating to PWM module

/*****************************************************************************************
* Function Prototypes
*****************************************************************************************/
// Tasks
static void AppStartTask(void *p_arg);
static void UITask(void *p_arg);
static void UIKeySrvTask(void *p_arg);
static void UISPISrvTask(void *p_arg);

// Non-Tasks
static void getPwmRate(INT8U *passpwm, OS_ERR *os_err); // Consider relocating to PWM module
static void setPwmRate(INT8U *passpwm, OS_ERR *os_err); // Consider relocating to PWM module


/*****************************************************************************************
* Private resources
*****************************************************************************************/
static INT8U pwmrate; //  Hold the duty cycle to send to PWM Module:  Consider relocating to PWM module

/*****************************************************************************************
 * Enumerated Types - Used in the UI
 *****************************************************************************************/
typedef enum {RUNNING, ADJUST, FAULT} SYS_STATE; // States of the system
typedef enum {OUT, TENS, ONES} SETTING_STATE; // States withing Setting

/*****************************************************************************************
* main()
*****************************************************************************************/
void main(void){
    OS_ERR os_err;
    CPU_IntDis();                                               //Disable all interrupts, OS will enable them
    OSInit(&os_err);                                            //Initialize uC/OS-III
    while(os_err != OS_ERR_NONE){}                              //Error Trap

    OSTaskCreate(&AppTaskStartTCB,                              //Address of TCB assigned to task
                 "Start Task",                                  //Name you want to give the task
                 AppStartTask,                                  //Address of the task itself
                 (void *) 0,                                    //p_arg is not used so null ptr
                 APP_CFG_TASK_START_PRIO,                       //Priority you assign to the task
                 &AppTaskStartStk[0],                           //Base address of task’s stack
                 (APP_CFG_TASK_START_STK_SIZE/10u),             //Watermark limit for stack growth
                 APP_CFG_TASK_START_STK_SIZE,                   //Stack size
                 0,                                             //Size of task message queue
                 0,                                             //Time quanta for round robin
                 (void *) 0,                                    //Extension pointer is not used
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),   //Options
                 &os_err);                                      //Ptr to error code destination
    while(os_err != OS_ERR_NONE){}                              //Error Trap

    OSStart(&os_err);                                           //Start multitasking (i.e. give control to uC/OS)
    while(os_err != OS_ERR_NONE){}                              //Error Trap
}

/*****************************************************************************************
* This should run once and be suspended. Could restart everything by resuming.
* (Resuming not tested)
* Todd Morton, 01/06/2016
* Modified for Lab 4: Rod Mesecar, 03/02/2018
*****************************************************************************************/
static void AppStartTask(void *p_arg) {
    OS_ERR os_err;
    (void)p_arg;                                //Avoid compiler warning for unused variable
    OS_CPU_SysTickInitFreq(DEFAULT_SYSTEM_CLOCK);

    //Initialize peripherals
    SPIInit();
    LcdInit();
    KeyInit();
    PWMInit();
    GpioDBugBitsInit();

    // Create Semaphores
    OSMutexCreate(&PwmRateKey, "PWM Rate Key", &os_err); // Consider relocating to PWM module
    OSSemCreate(&NewPwmRate, "New PWM Rate Flag", 0, &os_err); // Consider relocating to PWM module

    // Create Tasks
    OSTaskCreate(&UITaskTCB,
                 "UI Task",
                 UITask,
                 (void *) 0,
                 APP_CFG_UITASK_PRIO,
                 &UITaskStk[0],
                 (APP_CFG_UITASK_STK_SIZE / 10u),
                 APP_CFG_UITASK_STK_SIZE,
				 UI_TASK_MSG_Q_SIZE, // Task Msg Queue
                 0,
                 (void *) 0,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);
    while(os_err != OS_ERR_NONE){}              //Error Trap

    OSTaskCreate(&UIKeySrvTaskTCB,
				 "UI Key Service Task",
				 UIKeySrvTask,
				 (void *) 0,
				 APP_CFG_UIKEYSRV_TASK_PRIO,
				 &UIKeySrvTaskStk[0],
				 (APP_CFG_UIKEYSRV_TASK_STK_SIZE / 10u),
				 APP_CFG_UIKEYSRV_TASK_STK_SIZE,
				 0,
				 0,
				 (void *) 0,
				 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 &os_err);
	while(os_err != OS_ERR_NONE){}              //Error Trap

	OSTaskCreate(&UISPISrvTaskTCB,
				 "UI SPI Service Task",
				 UISPISrvTask,
				 (void *) 0,
				 APP_CFG_UISPISRV_TASK_PRIO,
				 &UISPISrvTaskStk[0],
				 (APP_CFG_UISPISRV_TASK_STK_SIZE / 10u),
				 APP_CFG_UISPISRV_TASK_STK_SIZE,
				 0,
				 0,
				 (void *) 0,
				 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 &os_err);
	while(os_err != OS_ERR_NONE){}              //Error Trap

    OSTaskSuspend((OS_TCB *)0, &os_err);
    while(os_err != OS_ERR_NONE){}              //Error Trap
}



/*****************************************************************************************
* getPWMRate() - Copies the contents of PWMRate into the location of the passed pointer
*  03/04/2018 Rod Mesecar
*  Consider relocating to PWM module
*****************************************************************************************/
void getPwmRate(INT8U *passpwm, OS_ERR *os_err){
	OSMutexPend(&PwmRateKey, 0, OS_OPT_PEND_BLOCKING, (CPU_TS *)0, os_err);
	*passpwm = pwmrate;
	OSMutexPost(&PwmRateKey, OS_OPT_POST_NONE, os_err);
}



/*****************************************************************************************
* setPWMRate() - Copies the contents of the location of the passed pointer into PwmRate
* 03/04/2018 Rod Mesecar
* Consider relocating to PWM module
*****************************************************************************************/
static void setPwmRate(INT8U *passpwm, OS_ERR *os_err){
	OSMutexPend(&PwmRateKey, 0, OS_OPT_PEND_BLOCKING, (CPU_TS *)0, os_err);
    pwmrate = *passpwm;
	OSMutexPost(&PwmRateKey, OS_OPT_POST_NONE, os_err);
	OSSemPost(&NewPwmRate, OS_OPT_POST_1, os_err);
}

/*****************************************************************************************
* UITask() - Controls the user interface
* 03/07/2018 Rod Mesecar
*****************************************************************************************/
static void UITask(void *p_arg){
    OS_ERR os_err;
    SYS_STATE currentstate = RUNNING; // System State
    SETTING_STATE currentplace = OUT; // Setting State - Only used when in SETTING State
    INT16U copiedmsg = 0; // Local copy of the passed message
    INT16U *queuedmsg; // Pointer to the message
    OS_MSG_SIZE msg_size; // Message Size, Used to determining Sender
    INT8U whichOutput; // The Selected Output (used for Setting)
    INT8U nextPWMRate; // What to send to the PWM Module
    INT16U nextSpiMsg; // What to send to the SPI Module
    (void)p_arg;

    //Preset Screen
    LcdDispString(STATUS_ROW, FIRST_COL, UI_LAYER, NO_OUTPUT_MSG); // No Output
    LcdShowLayer(UI_LAYER);

    while(1){

    	DB1_TURN_OFF(); // Debug Pin off
    	queuedmsg = OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &msg_size, (CPU_TS *)0, &os_err); //pend on message queue
    	while(os_err != OS_ERR_NONE){}      //Error Trap
    	copiedmsg = *queuedmsg; // Copy the message contence
    	DB1_TURN_ON(); // Debug Pin On

		// Use msg_size to figure out source
    		if (msg_size == 1){ // This would be a keypress (1 keypress == 1 byte)
    			if (copiedmsg == D_KEY){ // This is the emergency stop key. Pressing "D" will stop all motors
    				setSpiData(EMERGENCY_STOP, &os_err); //Send Ox0000 to the SPI Module
    				setPwmRate((INT8U)EMERGENCY_STOP, &os_err); // Set the PWM Rate to 0
    				LcdDispClrLine(STATUS_ROW,UI_LAYER);
    				LcdDispString(STATUS_ROW, FIRST_COL, UI_LAYER, NO_OUTPUT_MSG); // Update Output Status
    				LcdHideLayer(FAULT_LAYER); // Hide Fault Message (if any)
    				LcdShowLayer(UI_LAYER); //
    				currentstate = RUNNING; // Change System State

    			}else if((currentstate == RUNNING) && (copiedmsg != D_KEY)){
    				if (copiedmsg == A_KEY){ // A is pressed, user wants to change output
    					// Change State
    					currentstate = ADJUST;
    					// Update UI
    					LcdDispString(UI_ROW, FIRST_COL, UI_LAYER, SET_MSG);
    					LcdCursor(UI_ROW,UI_OUT_COL, UI_LAYER, CURSOR_ON, CURSOR_BLINK);

    					// Reset Values
    					currentplace = OUT;
    					whichOutput = 0;
						nextPWMRate =0;
						nextSpiMsg = 0;

    				}else{ // Don't Care About other key presses in this state
    					// Do nothing
    				}
    			}else if((currentstate == ADJUST) && (copiedmsg != D_KEY)){
    				switch (currentplace){
    					case OUT: // Select which Output to use
    						if (copiedmsg == FOUR_KEY){
    							// Update UI
    							LcdDispString(UI_ROW, UI_OUT_COL, UI_LAYER,OUT_FOU_MSG);
    							LcdDispString(UI_ROW, UI_PWM_MSG_COL, UI_LAYER, PWM_MSG);
    							LcdCursor(UI_ROW,UI_PWM_TENS, UI_LAYER, CURSOR_ON, CURSOR_BLINK);
    							LcdShowLayer(UI_LAYER);

    							// Update Values
    							 whichOutput = 4;
    							 nextSpiMsg  = OUTPUT_ONE_MASK;

    							 // Change SETTING State
    							 currentplace = TENS;

    						}else if(copiedmsg == SEV_KEY) {
    							// Update UI
    							LcdDispString(UI_ROW, UI_OUT_COL, UI_LAYER,OUT_SEV_MSG);
    							LcdDispString(UI_ROW, UI_PWM_MSG_COL, UI_LAYER, PWM_MSG);
    							LcdCursor(UI_ROW,UI_PWM_TENS, UI_LAYER, CURSOR_ON, CURSOR_BLINK);
    							LcdShowLayer(UI_LAYER);

    							// Update Values
    							whichOutput = 7;
    							nextSpiMsg  = OUTPUT_THR_MASK;

    							// Change State
    							currentplace = TENS;

    						}else{ // No other Valid output is valid at this time. Add 'else if's following the layout above to add more outputs.
    							// Do nothing
    						}
    					break;
    					case TENS: // Set the tens place of the PWM rate
    						if ((copiedmsg >= ZERO_KEY) && (copiedmsg <= NIN_KEY)){
    							//Change only the tens place of value
    							nextPWMRate = 10*(copiedmsg-NUMBER_KEY_TO_DEC_FACTOR) + (nextPWMRate - (10*(nextPWMRate/10)));

    							// Update UI
    							LcdDispDecByte(UI_ROW,UI_PWM_WRITE, UI_LAYER, nextPWMRate, 0);
								LcdCursor(UI_ROW,UI_PWM_ONES, UI_LAYER, CURSOR_ON, CURSOR_BLINK);
								LcdShowLayer(UI_LAYER);

								//Change SETTING State
								currentplace = ONES;

							}else if (copiedmsg == B_KEY){ // Backspace, go back and clear
								currentplace = OUT;
								nextPWMRate = 0;
								LcdCursor(UI_ROW,UI_OUT_COL, UI_LAYER, CURSOR_ON, CURSOR_BLINK);
							}else{
								// Do Nothing
							}
    					break;
    					case ONES: // Set the Ones's place
    						if ((copiedmsg >= ZERO_KEY) && (copiedmsg <= NIN_KEY)){
    							nextPWMRate = (copiedmsg-NUMBER_KEY_TO_DEC_FACTOR) + (10*(nextPWMRate/10));
								LcdDispDecByte(UI_ROW,UI_PWM_WRITE, UI_LAYER, nextPWMRate, 0);
								LcdCursor(UI_ROW,UI_PWM_ONES, UI_LAYER,CURSOR_ON,CURSOR_BLINK);

							}else if (copiedmsg == B_KEY){ // Backspace, go back to tens place
								currentplace = TENS;
								LcdCursor(UI_ROW,UI_PWM_TENS, UI_LAYER,CURSOR_ON,CURSOR_BLINK);

							}else if(copiedmsg == A_KEY){ // User accepts current value displayed
								 if (nextPWMRate ==0) // Don't want PWM, just Full output
								 {
									 setSpiData(&nextSpiMsg,&os_err);
								 }else if (nextPWMRate !=0) // Want a PWM Rate, Can't have SPI going (See MC33879 datasheet for Details: INS5 and INS6)
								 {
									 setSpiData(EMERGENCY_STOP,&os_err); // Send SPI Stop Message
									 setPwmRate(&(nextPWMRate), &os_err); // Send the PWM rate to PWM module
								 }

								//Update status Bar
								LcdDispClear(UI_LAYER);
								if (whichOutput == 4){
									LcdDispString(STATUS_ROW, FIRST_COL ,UI_LAYER, OUT_FOU_MSG);
								}else if(whichOutput == 7){
									LcdDispString(STATUS_ROW, FIRST_COL ,UI_LAYER, OUT_SEV_MSG);
								}else{ // Add more 'else if's to add more outputs
									// Do noting
								}
								LcdDispString(STATUS_ROW, UI_PWM_MSG_COL ,UI_LAYER,PWM_MSG);
								LcdDispDecByte(STATUS_ROW, UI_PWM_WRITE,UI_LAYER, nextPWMRate,0);
								LcdCursor(UI_ROW,FIRST_COL, UI_LAYER, CURSOR_OFF, CURSOR_OFF);

								// change system state
								currentstate = RUNNING;
							}else{
								// Do nothing
							}
						break;
    				}
    			}else{ // Should Never Get here
    				// Do nothing
    			}
    		}else if (msg_size == 2){ // This would be a SPI message (16bit SPI Message = 2 bytes)
    			if ((copiedmsg == NO_FAULT) && (currentstate == FAULT)){ // No error, Return to Normal status
    				LcdHideLayer(FAULT_LAYER);
    				LcdShowLayer(UI_LAYER);
    				currentstate = RUNNING;

    			}else { // Fault Detected
    				LcdHideLayer(UI_LAYER); //Hide Status Layer

    				LcdDispString(STATUS_ROW,FIRST_COL,FAULT_LAYER, FAULT_MSG); // Display Fault message

    				// Figure which motor is at fault
					if(copiedmsg == OUTPUT_ONE_MASK){ // Output 1 is at fault
						LcdDispString(STATUS_ROW,FAULT_MSG_OUT_COLLEM,FAULT_LAYER,OUT_ONE_MSG);
					}else if (copiedmsg == OUTPUT_TWO_MASK){// Output 2 is at fault
						LcdDispString(STATUS_ROW,FAULT_MSG_OUT_COLLEM,FAULT_LAYER,OUT_TWO_MSG);
					}else if (copiedmsg == OUTPUT_THR_MASK){// Output 3 is at fault
						LcdDispString(STATUS_ROW,FAULT_MSG_OUT_COLLEM,FAULT_LAYER,OUT_THR_MSG);
					}else if (copiedmsg == OUTPUT_FOU_MASK){// Output 4 is at fault
						LcdDispString(STATUS_ROW,FAULT_MSG_OUT_COLLEM,FAULT_LAYER,OUT_FOU_MSG);
					}else if (copiedmsg == OUTPUT_FIV_MASK){// Output 5 is at fault
						LcdDispString(STATUS_ROW,FAULT_MSG_OUT_COLLEM,FAULT_LAYER,OUT_FIV_MSG);
					}else if (copiedmsg == OUTPUT_SIX_MASK){// Output 6 is at fault
						LcdDispString(STATUS_ROW,FAULT_MSG_OUT_COLLEM,FAULT_LAYER,OUT_SIX_MSG);
					}else if (copiedmsg == OUTPUT_SEV_MASK){// Output 7 is at fault
						LcdDispString(STATUS_ROW,FAULT_MSG_OUT_COLLEM,FAULT_LAYER,OUT_SEV_MSG);
					}else if (copiedmsg == OUTPUT_EGT_MASK){// Output 8 is at fault
						LcdDispString(STATUS_ROW,FAULT_MSG_OUT_COLLEM,FAULT_LAYER,OUT_EGT_MSG);
					}else{ // Multiple Faults
						LcdDispString(STATUS_ROW,FAULT_MSG_OUT_COLLEM,FAULT_LAYER,MULIT_FAULT_MSG);
					}

					LcdShowLayer(FAULT_LAYER);
					currentstate = FAULT; // Update State
    			}
    		}else { // This should never happen
    			// Do Nothing
    		}
    }
}


/*****************************************************************************************
* UIKeySrvTask() - Pends on KeyPend and updates UITaskMsgQ. Gives keypad data to UITask().
*
* 02/09/2018, Rod Mesecar
* 02/16/2018, Modified for UITask(). Brian Willis
* 03/04/2018, Header corrected. Rod Mesecar
*****************************************************************************************/
static void UIKeySrvTask(void *p_arg){
    OS_ERR os_err;
    (void)p_arg;
    INT8U keypress = 0;
    OS_MSG_SIZE msg_size = sizeof(keypress);

    while(1){
    	DB2_TURN_OFF();
        keypress = KeyPend(0, &os_err);     //Wait for key press
        while(os_err != OS_ERR_NONE){}      //Error Trap
        DB2_TURN_OFF();

        OSTaskQPost(&UITaskTCB, &keypress, msg_size, OS_OPT_POST_FIFO, &os_err);    //Place keypress into queue
        while(os_err != OS_ERR_NONE){}      //Error Trap
    }
}


/*****************************************************************************************
* UISPISrvTask() - Pends on SPIPend and updates UITaskMsgQ. Gives keypad data to UITask().
*
* 02/09/2018, Rod Mesecar
* 02/16/2018, Modified for UITask(). Brian Willis
* 03/04/2018, Modified for receiving from SPI module, header updated appropriately, Rod Mesecar
*****************************************************************************************/
static void UISPISrvTask(void *p_arg){
    OS_ERR os_err;
    (void)p_arg;
    INT16U spimsg = 0;
    OS_MSG_SIZE msg_size = sizeof(spimsg);

    while(1){
    	DB3_TURN_OFF();
        spimsg = SPIPend(0, &os_err);     //Wait for SPI Message
        while(os_err != OS_ERR_NONE){}      //Error Trap
        DB3_TURN_OFF();

        OSTaskQPost(&UITaskTCB, &spimsg, msg_size, OS_OPT_POST_FIFO, &os_err);    //Place message into queue
        while(os_err != OS_ERR_NONE){}      //Error Trap
    }
}
