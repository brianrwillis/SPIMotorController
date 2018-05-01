/********************************************************************
* KeyUcos.h - The header file for the keypad module, KeyUcos.c
* 02/15/2006 Todd Morton
*
* Added Key Definitions for each keypad and a convertion factor
* 03/08/18 Rod Mesecar
*********************************************************************
* Public Resources
********************************************************************/
#ifndef UC_KEY_DEF
#define UC_KEY_DEF

//key define
#define ONE_KEY 0x31
#define TWO_KEY 0x32
#define THR_KEY 0x33
#define FOUR_KEY 0x34
#define FIV_KEY 0x35
#define SIX_KEY 0x36
#define SEV_KEY 0x37
#define EGT_KEY 0x38
#define NIN_KEY 0x39
#define ZERO_KEY 0x30
#define STAR_KEY 0x2A
#define NUM_KEY 0x23
#define A_KEY 0x11
#define B_KEY 0x12
#define C_KEY 0x13
#define D_KEY 0x14
#define NUMBER_KEY_TO_DEC_FACTOR 48 // Taking any of the numbered keys and subtracting this (48) will turn the value into the decimal printed on key

INT8U KeyPend(INT16U tout, OS_ERR *os_err); /* Pend on key press*/
                             /* tout - semaphore timeout           */
                             /* *err - destination of err code     */
                             /* Error codes are identical to a semaphore */

void KeyInit(void);             /* Keypad Initialization    */

#endif
