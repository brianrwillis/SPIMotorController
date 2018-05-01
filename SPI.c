/********************************************************************
* SPI.c - Module for controlling SPI peripheral
* 03/20/2018 Brian Willis
********************************************************************/
#include "MCUType.h"
#include "app_cfg.h"
#include "os.h"
#include "K65TWR_GPIO.h"
#include "SPI.h"

static void SPITask(void *p_arg);

static OS_MUTEX SpiDataKey;
static OS_SEM NewSpiData;

//Private resources
static OS_TCB spiTaskTCB;                                  //Allocate SPI Task control block
static CPU_STK spiTaskStk[APP_CFG_SPITASK_STK_SIZE];       //Allocate SPI Task stack space
static OS_SEM spiFaultFlag;                                //Allocate space for Fault Detection Semaphore
static INT8U spiFault = 0;
static INT16U spiMsg;


/*****************************************************************************************
* SPIInit() - Initializes the SPI peripheral
*****************************************************************************************/
void SPIInit(){
    OS_ERR os_err;

    SIM_SCGC6 |= SIM_SCGC6_SPI1_MASK;               //Turn on SPI1 clock
    SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;              //Turn on PORTE clock

    PORTE_PCR2 = PORT_PCR_MUX(2);                   //SCK:  B7  - PTE2
    PORTE_PCR4 = PORT_PCR_MUX(2);                   //SS:   B9  - PTE4
    PORTE_PCR1 = PORT_PCR_MUX(2);                   //MOSI: B11 - PTE1
    PORTE_PCR3 = PORT_PCR_MUX(2);                   //MISO: B10 - PTE3

    SPI1_CTAR0 |= SPI_CTAR_PBR(0);                  //Set prescalar to 2 and scalar to 8, achieves baud rate
    SPI1_CTAR0 |= SPI_CTAR_BR(3);                   //of ~3.6MHz for a protocol clock of ~60MHz
    SPI1_CTAR0 |= SPI_CTAR_FMSZ(15);                //Set transfer size to 16 bits

    SPI1_MCR &= SPI_MCR_HALT(0);                    //Disable halt mode
    SPI1_MCR |= SPI_MCR_MSTR(1);                    //Enable Master Mode
    SPI1_MCR |= SPI_MCR_PCSIS(1);                   //Set SS inactive state to 1

    //Dummy transmission to set Transfer Complete Flag with SS 1
    SPI1_PUSHR = SPI_PUSHR_TXDATA(0x0000) | SPI_PUSHR_PCS(1);

    OSSemCreate(&spiFaultFlag,                      //Create SPI Fault Flag
               "Time Change Flag",
               0,
               &os_err);

    //Create Mutexs (Moved From MSPI to SPI ~Rod)
    OSMutexCreate(&SpiDataKey, "SPI Data Key", &os_err);
    OSSemCreate(&NewSpiData, "New SPI Data Flag", 0, &os_err);


    while(os_err != OS_ERR_NONE){}                  //Error Trap
    OSTaskCreate(&spiTaskTCB,                       //Create SPI Task
                 "SPI Task",
                 SPITask,
                 (void *) 0,
                 APP_CFG_SPITASK_PRIO,
                 &spiTaskStk[0],
                 (APP_CFG_SPITASK_STK_SIZE / 10u),
                 APP_CFG_SPITASK_STK_SIZE,
                 0,
                 0,
                 (void *) 0,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);
    while(os_err != OS_ERR_NONE){}                  //Error Trap
}


/*****************************************************************************************
* getSpiData() - Copies the contents of SpiData into the location of the passed pointer
* ~Rod Mesecar
*****************************************************************************************/
void getSpiData(INT16U *passMsg, OS_ERR *os_err){
    OSMutexPend(&NewSpiData, 0, OS_OPT_PEND_BLOCKING, (CPU_TS *)0, os_err);
    *passMsg = spiMsg;
    OSMutexPost(&NewSpiData, OS_OPT_POST_NONE, os_err);
}

/*****************************************************************************************
* setSpiMsg() - Copies the contents of the location of the passed pointer into SpiData
* ~Rod Mesecar
*****************************************************************************************/
void setSpiData(INT16U *passmsg, OS_ERR *os_err){
    OSMutexPend(&SpiDataKey, 0, OS_OPT_PEND_BLOCKING, (CPU_TS *)0, os_err);
    spiMsg = *passmsg;
    OSMutexPost(&SpiDataKey, OS_OPT_POST_NONE, os_err);
    OSSemPost(&NewSpiData, OS_OPT_POST_1, os_err);
}

/*****************************************************************************************
* SPIPend() - Pends on the SPI fault detection semaphore
*****************************************************************************************/
INT8U SPIPend(INT16U tout, OS_ERR *os_err){
    //Might need to be interrupt from digital input pin
    OSSemPend(&spiFaultFlag, tout, OS_OPT_PEND_BLOCKING, (CPU_TS *)0, os_err);
    return spiFault;
}

/*****************************************************************************************
 * SPITask() - Controls the SPI peripheral
 *****************************************************************************************/
static void SPITask(void *p_arg){
    OS_ERR os_err;
    INT16U newMsg;
    (void)p_arg;

    while(1){
        getSpiData(&newMsg,&os_err);
        while(os_err != OS_ERR_NONE){}                              //Error Trap

        while((SPI1_SR & SPI_SR_TCF_MASK) == 0){}                   //Wait for previous data to transmit
        SPI1_SR |= SPI_SR_TCF(1);                                   //Reset Transfer Complete Flag
        SPI1_PUSHR = SPI_PUSHR_TXDATA(newMsg) | SPI_PUSHR_PCS(1);   //Push data to transmit with SS 1
    }
}
