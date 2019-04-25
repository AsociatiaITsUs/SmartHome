/******************************************************************************
 * COPYRIGHT  : Easythings
 * PROJECT    : smart thermostat - heater valve
 *
 * DESCRIPTION: start thermostat that can control the heater water flow and the
 *              central heating device.
 * FILENAME   : smartThermostat.c
 *
 * PLATFORM DEPENDANT [yes/no]: yes
 *
 * CURRENT REVISION: 1.0
 * DATE            : 2018/101/12
 * AUTHOR          :
 ******************************************************************************/
 /* Revision |  Author               |   Change description
 * _________|_______________________|___________________________________________
 *   1.0       Dediu VD                  Initial revision
 ******************************************************************************/

/*******************************************************************************
 *                      MISRA comments
 ***********************START MISRA comments***********************************/

/***********************END MISRA comments*************************************/

/*******************************************************************************
 *                      Include Section
 *          HEADER             |               PURPOSE OF INCLUSION
 ***********************START Include Section**********************************/ 
#include "../BSW/Automatic_Heater_Valve/ahv_prj/ahv_main/AutomaticHeaterValve.h"
#include "../BSW/LIB/platform_types/platform_types.h"
#include "../BSW/LIB/crc/crc_check.h"

static boolean getGatewayRequests(uint8_t* rxBuff);
static void boardPinoutCfg(void);
static void learnTapRange(void);
static void controlHeaterTap(eRequestType request);
static boolean checkBatteryStatus(void);
static void sendGatewayResponses(uint8_t* txBuff);

{
  /* board pinout configuration */
  boardPinoutCfg();
  /* check the range of the valve 0 - 100%  - adaptation phase.
  Should only be done once...*/
  learnTapRange();
}

void loop() 
{
  boolean isGatewayReqValid;
  uint8_t gatewayReqs[RX_BUFFER_SIZE];
  uint8_t gatewayResps[TX_BUFFER_SIZE];
  
  /* check for gateway inputs */
  isGatewayReqValid = getGatewayRequests(&gatewayReqs);
  if(isGatewayReqValid == true)
  {
    /* in case the request is valid and it is addressed to this sensor 
    - perform some action */
    
  }
  /* check battery status */
  gatewayResps[BAT_STAT_POS] = checkBatteryStatus();
  /* send information to the gateway */
  sendGatewayResponses(&gatewayResps);
}


static void sendGatewayResponses(uint8_t* gatewayResps)
{
  //construct the response
}

static boolean getGatewayRequests(uint8_t* gatewayReqs)
{
  /* check message key and CRC */
  boolean msgValid = true;
  
  return msgValid;
}

static void boardPinoutCfg(void)
{
  /* set the inputs and outputs */
}

static void learnTapRange(void)
{
  /* at the first start, after installing the device this learning function must be
  triggered in order to learn the entire range of the tap - MIN = closed- MAX = opened
  /These values must then be stored persitantly.
  https://github.com/CuriousTech/ESP8266-HVAC/blob/master/Arduino/eeMem.cpp */
  
}

static void controlHeaterTap(eRequestType request)
{
  switch(request)
  {
    case OPEN:
      /* open the tap in order to increase the room temperature */
    break;
    case CLOSE:
      /* close the tap - only if the room temperature is reached and the central
      heater is still running */
    break;
    default:
      /* nothing to be done here */
    break;
  }
}

static boolean checkBatteryStatus(void)
{
  /* check the battery status using an ADC every hour or more in order to notify
  the user about a possible recharge need.
  This information will be forwarde to the user by the gateway */
  boolean isBatteryFull = true;

  return isBatteryFull;
}

  