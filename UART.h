/*
 * UART.h
 *
 *  Created on: May 10, 2020
 *      Author: Zoltan Gere
 */

#ifndef UART_H_
#define UART_H_

/**************************************************************************
 * @brief  Initialize UART interface, 9600 baud, 8, None, 1, LSB first
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void UARTInit();

/**************************************************************************
 * @brief  Send a null-terminated string to UART
 *
 * @param  const char text
 *
 * @return none
 *************************************************************************/
void Send(const char * text);

/**************************************************************************
 * @brief   Receive string from UART
 *          UART is read until CR/LF read or buffer full
 *          String terminates with null
 *
 * @param   char text       Text buffer
 *          int bufSize     Size of the buffer
 *
 * @return none
 *************************************************************************/
void Receive(char * text, int bufSize);

#endif /* UART_H_ */
