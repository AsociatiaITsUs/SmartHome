#ifndef SMART_THERMOSTAT_H
#define SMART_THERMOSTAT_H
/******************************************************************************
 * COPYRIGHT  : Easythings
 * PROJECT    : smart thermostat - heater valve
 *
 * DESCRIPTION: start thermostat that can control the heater water flow and the
 *              central heating device.
 * FILENAME   : smartThermostat.h
 *
 * PLATFORM DEPENDANT [yes/no]: no
 *
 * CURRENT REVISION: 1.0
 * DATE            : 2018/101/12
 * AUTHOR          :
 *
 *******************************************************************************
 * Revision |  Author               |   Change description
 * _________|_______________________|___________________________________________
 *   1.0       Dediu VD                  Initial revision
 *
 ******************************************************************************/

/*******************************************************************************
 *                      MISRA comments
 ***********************START MISRA comments***********************************/

/***********************END MISRA comments*************************************/


/*******************************************************************************
 *                     Include Section
 *   HEADER                   |              PURPOSE OF INCLUSION
 ***********************START Include Section**********************************/

/***********************END Include Section************************************/


/*******************************************************************************
*                           Global Macros                                      *
************************START Global Macros************************************/
/* RX and TX buffer sizes in bytes */
#define RX_BUFFER_SIZE 32
#define TX_BUFFER_SIZE 32


/***********************END Global Macros**************************************/


/*******************************************************************************
*                      Global Data Types                                       *
************************START Global Data Types********************************/
typedef enum
{
  OPEN = 0,
  CLOSE
}eRequestType;

/***********************END Global Data Types**********************************/


/*******************************************************************************
*                      Global Data                                             *
************************START Global Data**************************************/

/***********************END Global Data****************************************/


/*******************************************************************************
*                      Global Functions                                        *
************************START Global Functions*********************************/

/***********************END Global Functions***********************************/

#endif
