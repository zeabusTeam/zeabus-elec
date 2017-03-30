#include <sys/syscall.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "ftd2xx.h"
#include "zblib.h"

typedef unsigned int uint_t;
typedef struct
	FT_HANDLE	xHandle;
	uint_t		wIsValid;
}xFTHandle_t, *pxFTHandle_t;

/* Descriptor string pre-programmed to Power Distributor and Peripheral Bridge modules */
static const char* cstPowerDistSerial= "PowerDist";
static const char* cstPeripheralSerial[] =
{
	"PeripheralBridge-A",
	"PeripheralBridge-B",
	"PeripheralBridge-C",
	"PeripheralBridge-D"
};

static uint32_t	ulIsInit = 0;	/* 0 means Init function never run or fail */
static ZB_STATUS_T xCurrentStatus = ZB_OK;	/* The status of the previous operation */
/* Error messages according to the error code */
static const char* cstErrorString[] =
{
	"No error found",
	"General or unspecified failure",
	"Unable to delete kernel modules that already occupied the FTDI chips",
	"Initialization failed",
	"The library has never initialized",
	"Unable to find any supported device",
	"Unable to communicate with the peripheral module",
	"Unable to communicate with the power-switch module",
	"Writing to RS232 failed",
	"Reading from RS232 failed"
};

/* Storage of device handles. The element 0 is for FT232H. All others are for FT4232H, which
has 4 modules inside. */
static xFTHandle_t xFTDevHandle[5];

/* Reset and initialize all FTxxxx chips. Then, prepare all setup for further operation */
ZB_Status_t ZB_Init()
{
	int iSysCallStat;
	FT_STATUS xFTStatus;
	FT_DEVICE_LIST_INFO_NODE *xDevInfo;
	uint32_t ulTotalDevs, ulNumFoundDev;
	uint_t i;
	
	/* If all thing is initialized */
	if( ulIsInit == 1 )
	{
		return( ZB_OK );
	}
	
	/* Initialize all handle */
	for( i = 0; i < 5; i++ )
	{
		xFTDevHandle[ i ].wIsValid = 0;
	}
	
	/* Remove all kernel modules that occupied the FTDI chips */
	iSyscallStat = syscall( SYS_delete_module, "ftdi_sio", O_NONBLOCK | O_TRUNC );
	if( ( iSyscallStat != 0 ) && ( errno != ENOENT ) )
	{
		/* Unable to delete module FTDI_SIO from kernel */
		xCurrentStatus = ZB_STAT_REMOVE_MOD_FAIL;
		return( xCurrentStatus );
	}

	/* create the device information list */
	xFTStatus = FT_CreateDeviceInfoList( &ulTotalDevs );
	if ( ( xFTStatus != FT_OK ) || ( ulTotalDevs == 0 ) )
	{
		/* Unable to find any devices */
		xCurrentStatus = ZB_STAT_NO_DEVICE;
		return( xCurrentStatus );
	}
	
	/* We found some FTDI chips, then get the information of the devices */
    xDevInfo = (FT_DEVICE_LIST_INFO_NODE*)malloc( sizeof( FT_DEVICE_LIST_INFO_NODE) * ulTotalDevs );
	xFTStatus = FT_GetDeviceInfoList( xDevInfo, &ulTotalDevs );
	if ( xFTStatus != FT_OK )
	{
		/* Unable to find any devices */
		xCurrentStatus = ZB_STAT_NO_DEVICE;
		free( xDevInfo );	/* Free-up the previous allocation */
		return( xCurrentStatus );
	}

	/* Discover the Power Distributor and Peripheral Bridge. Also, open handles for them.
	We use linear search as, normally, the number of device should not small */
	ulNumFoundDev = 0;

	/* Attempt to open the Power Distributor module described by its serial number */
	xFTStatus = FT_OpenEx( cstPowerDistSerial, FT_OPEN_BY_SERIAL_NUMBER, &( xFTDevHandle[ 0 ].xHandle ) );
	if (xFTStatus == FT_OK)
	{
		/* success - device with serial number "PowerDist" is open */
		xFTDevHandle[ 0 ].wIsValid = 1;
		ulNumFoundDev++;	/* Increase successfully discovered device number */
	}

	/* Attempt to open each Peripheral Bridge module described by its serial number */
	for( i = 0; i < 4; i++ )
	{
		xFTStatus = FT_OpenEx( cstPeripheralSerial[ i ], FT_OPEN_BY_SERIAL_NUMBER, &( xFTDevHandle[ i + 1 ].xHandle ) );
		if (xFTStatus == FT_OK)
		{
			/* success - device with serial number "PeripheralBridge-?" is open */
			xFTDevHandle[ i + 1 ].wIsValid = 1;
			ulNumFoundDev++;	/* Increase successfully discovered device number */
		}
	}
	
	/* Free the memory allocated for the device information list */
	free( xDevInfo );
	
	/* Verify that all required devices are found and opened */
	if( ulNumFoundDev == 5 )
	{
		xCurrentStatus = ZB_OK;
	}
	else
	{
		if( xFTDevHandle[0].wIsValid == 0 )
		{
			xCurrentStatus = ZB_STAT_POWER_SW_FAIL;
		}
		else
		{
			xCurrentStatus = ZB_STAT_PERIPHERAL_BRIDGE_FAIL;
		}
	}
	
	ulIsInit = 1;
	return( xCurrentStatus );
}

/* Retrieve the error code of the previous operation. Some functions may return this error
code by default. */
ZB_Status_t ZB_GetErrorCode()
{
	return( xCurrentStatus );
}

/* Retrieve the error message of the previous operation. This function is useful when
users want to print out the error message. */
const char* ZB_GetErrorString()
{
	return( cstErrorString[ xCurrentStatus ] );
}

/* Turn-on and turn-off power switches as assigned by ucSWMask. Multiple switches
could be turn-on or -off by using logical OR operator. For example, ZB_PWR_SW1 | ZB_PWR_SW2 */
ZB_Status_t ZB_SW_TurnOn( uint8_t ucSWMask )
{
	uint8_t ucCommand[3];	/* Buffer for FT232H commands to turn on the switches */
	
	/* Check whether all devices are initialized */
	if( ulIsInit == 0 )
	{
		xCurrentStatus = ZB_STAT_NEVER_INIT;
		return( xCurrentStatus );
	}
	
	/* Verify the device handle */
	if( xFTDevHandle[ 0 ].wIsValid == 0 )
	{
		xCurrentStatus = ZB_STAT_POWER_SW_FAIL;
		return( xCurrentStatus );
	}
	
}
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

