/*
 * NTC_LUT.h
 *
 *  Created on: Apr 15, 2021
 *      Author: Zoltan Gere
 */

#ifndef NTC_LUT_H_
#define NTC_LUT_H_

/**********************************************************************//**
 * @brief  Look up the temperature associated to the resistance
 *         From a look-up table closest temperature values are taken
 *         Then the temperature is interpolated from the values
 *
 * @param  The measured NTC resistance
 *
 * @return The calculated temperature * 100
 *************************************************************************/
int LookUpTEMP(long int res);

#endif /* NTC_LUT_H_ */
