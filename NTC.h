/*
 * NTC.h
 *
 *  @brief  Handling the temperature measurement with the on board NTC thermistor
 *
 *  Created on: May 10, 2020
 *      Author: Zoltan Gere
 */

#ifndef NTC_H_
#define NTC_H_

/**********************************************************************//**
 * @brief  Take 50 ADC Measurement
 *          Calculate average
 *
 * @param  none
 *
 * @return Average of 50 temperature value
 *************************************************************************/
unsigned int Take50ADCMeas(void);

/**********************************************************************//**
 * @brief  Take ADC Measurement
 *          Temperature is written to ADCResult by ISR
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void TakeADCMeas(void);

/**********************************************************************//**
 * @brief  Setup thermistor
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void SetupThermistor(void);

/**********************************************************************//**
 * @brief  ShutDownTherm
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void ShutDownTherm(void);

/**************************************************************************
 * @brief  MeasureTemp
 *          Perform NTC temperature measurement
 *
 * @param  none
 *
 * @return unsigned int Temperature
 *************************************************************************/
unsigned int MeasureTemp(void);

#endif /* NTC_H_ */
