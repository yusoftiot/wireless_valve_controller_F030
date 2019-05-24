/*
 * spi.h
 *
 *  Created on: Feb 10, 2018
 *      Author: yuri
 */

#ifndef SPI_H_
#define SPI_H_

//#define SPI_MODE_MASTER 		1

void initSPI(int bits, int mode);
uint8_t spi_transfer(uint8_t data);

#endif /* SPI_H_ */
