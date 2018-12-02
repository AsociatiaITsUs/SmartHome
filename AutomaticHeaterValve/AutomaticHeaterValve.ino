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
#include"smartThermostat.h"

static boolean getGatewayRequests(uint8_t* rxBuff);
static void boardPinoutCfg(void);
static void learnTapRange(void);
static void controlHeaterTap(eRequestType request);
static boolean checkBatteryStatus(void);
static void sendGatewayResponses(uint8_t* txBuff);

static const uint8_t CRC_8_TABLE[256] = 
{
    0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
  157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
   35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
  190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
   70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
  219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
  101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
  248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
  140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
   17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
  175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
   50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
  202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
   87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
  233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
  116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53
};

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

  
static uint8_t calculateCrc8(const uint8_t *dataArray, const uint16_t length)
{
  uint16_t i;
  uint8_t CRC;
 
  CRC = 0;
  for (i = 0; i < length; i++)
    CRC = CRC_8_TABLE[CRC ^ dataArray[i]];
 
  return CRC;
}
