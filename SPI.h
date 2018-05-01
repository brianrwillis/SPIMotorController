/********************************************************************
* SPI.h - Header file for SPI module
* 03/20/2018 Brian Willis
********************************************************************/
#ifndef SPI_H_
#define SPI_H_

void SPIInit(void);
INT8U SPIPend(INT16U tout, OS_ERR *os_err);
void getSpiData(INT16U *passMsg, OS_ERR *os_err);
void setSpiData(INT16U *passmsg, OS_ERR *os_err);

#endif
