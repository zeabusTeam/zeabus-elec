#ifndef __ZBLIB_H
#define __ZBLIB_H

#include <stdint.h>

/* Power switch number for turn-on */
#define	ZB_PWR_SW1		0x01
#define	ZB_PWR_SW2		0x02
#define	ZB_PWR_SW3		0x04
#define	ZB_PWR_SW4		0x08
#define	ZB_PWR_SW5		0x10
#define	ZB_PWR_SW6		0x20
#define	ZB_PWR_SW7		0x40
#define	ZB_PWR_SW8		0x80

/* Solenoid number for turn-on */
#define	ZB_SOLENOID1	0x01
#define	ZB_SOLENOID2	0x02
#define	ZB_SOLENOID3	0x04
#define	ZB_SOLENOID4	0x08
#define	ZB_SOLENOID5	0x10
#define	ZB_SOLENOID6	0x20
#define	ZB_SOLENOID7	0x40
#define	ZB_SOLENOID8	0x80

/* Return status value */
typedef enum
{
	ZB_STAT_OK = 0,
	ZB_STAT_GENERAL_FAIL,
	ZB_STAT_REMOVE_MOD_FAIL,
	ZB_STAT_INIT_FAIL,
	ZB_STAT_NEVER_INIT,
	ZB_STAT_NO_DEVICE,
	ZB_STAT_PERIPHERAL_BRIDGE_FAIL,
	ZB_STAT_POWER_SW_FAIL
	ZB_STAT_COMM_WRITE_FAIL,
	ZB_STAT_COMM_READ_FAIL,
}ZB_Status_t;
/* Function calls. Unless specified, all functions return the status of the operation. */

/* Reset and initialize all FTxxxx chips. Then, prepare all setup for further operation */
ZB_Status_t ZB_Init();

/* Retrieve the error code of the previous operation. Some functions may return this error
code by default. */
ZB_Status_t ZB_GetErrorCode();
/* Retrieve the error message of the previous operation. This function is useful when
users want to print out the error message. */
const char* ZB_GetErrorString();

/* Turn-on and turn-off power switches as assigned by ucSWMask. Multiple switches
could be turn-on or -off by using logical OR operator. For example, ZB_PWR_SW1 | ZB_PWR_SW2 */
ZB_Status_t ZB_SW_TurnOn( uint8_t ucSWMask );
ZB_Status_t ZB_SW_TurnOff( uint8_t ucSWMask );

/* Turn-on and turn-off solenoid valves as assigned by ucSolMask. Multiple solenoids
could be turn-on or -off by using logical OR operator. For example, ZB_SOLENOID1 | ZB_SOLENOID2 */
ZB_Status_t ZB_Solenoid_TurnOn( uint8_t ucSolMask );
ZB_Status_t ZB_Solenoid_TurnOff( uint8_t ucSolMask );

/* Write a stream of data to the specified RS232 port (Com1 or Com2). The function returns 
the amount of actually written data. If the returned value differs from the aspected one.
User can get the error status from ZB_GetErrorCode or ZB_GetErrorString */
size_t ZB_Write_Com1( uint8_t* ucData, size_t usLen );
size_t ZB_Write_Com2( uint8_t* ucData, size_t usLen );

/* Read a stream of data to the specified RS232 port (Com1 or Com2). The function returns 
the amount of actually read data. If the returned value differs from the aspected one.
User can get the error status from ZB_GetErrorCode or ZB_GetErrorString */
size_t ZB_Read_Com1( uint8_t* ucData, size_t usMaxLen );
size_t ZB_Read_Com2( uint8_t* ucData, size_t usMaxLen );

#endif