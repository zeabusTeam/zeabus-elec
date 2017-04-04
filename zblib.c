#include <sys/syscall.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ftd2xx.h"
#include "zblib.h"

/* ===================================================
 * Constants
 * ===================================================
 */
#define USB_BULK_SIZE	512

/* FTDI Command */
#define MSB_RISING_EDGE_BYTE_IN		(0x20)
#define SET_DATA_LOW_BYTE			(0x80)
#define GET_DATA_LOW_BYTE			(0x81)
#define SET_DATA_HIGH_BYTE			(0x82)
#define GET_DATA_HIGH_BYTE			(0x83)
#define SET_CLK_RATE				(0x86)
#define	DISABLE_CLK_DIV				(0x8A)
#define	DISABLE_THREE_PHASE 		(0x8D)
#define DISABLE_ADAPTIVE_CLK		(0x97)

/* Pin direction for each port. Bit value 1 means the corresponding pin is output.
 * For FT232H: 
 *  - Low-byte is unused and kept all MPSSE and GPIO pins as output.
 *  - High-byte is used as output GPIO pins used for power switches.
 * For FT4232H:
 *  - Module A:
 *    + Low nibble pins are used for SPI connection with only DI pin configured as an input
 *    + High nibble pins are output GPIO pins used for solenoid valves.
 *  - Module B:
 *    + Low nibble pins are unused and kept as output.
 *    + High nibble pins are output GPIO pins used for solenoid valves.
 *  - Module C is used as a UART with 8n1 format and without any flow-control.
 *  - Module D is used as a UART with 8n1 format and without any flow-control.
 */
#define DIR_232H_HI					(0xFF)
#define DIR_232H_LO					(0xFF)
#define DIR_4232H_A					(0xFB)
#define DIR_4232H_B					(0xFF)
#define DIR_4232H_C					(0xFD)
#define DIR_4232H_D					(0xFD)

/* The values on idle state of the port pins. For MPSSE, the CLK, DI, and CS pin is normally high */
#define IDLE_MPSSE					(0x0B)
#define IDLE_GPIO					(0x00)
#define IDLE_UART					(0xFD)

/* Descriptor string pre-programmed to Power Distributor and Peripheral Bridge modules */
static const char* cstPowerDistSerial= "PowerDist";
static const char* cstPeripheralSerial[] =
{
	"PeripheralBridge-A",
	"PeripheralBridge-B",
	"PeripheralBridge-C",
	"PeripheralBridge-D"
};

/* Error messages according to the error code */
static const char* cstErrorString[] =
{
	"No error found",
	"General or unspecified failure",
	"Unable to delete kernel modules that already occupied the FTDI chips",
	"Initialization failed",
	"The library has never initialized",
	"Unable to find any supported device",
	"Unable to read the pressure sensor",
	"Unable to communicate with the peripheral module",
	"Unable to communicate with the power-switch module",
	"Writing to RS232 failed",
	"Reading from RS232 failed"
};

/* ===================================================
 * Type definitions
 * ===================================================
 */
typedef struct
{
	FT_HANDLE	xHandle;
	uint32_t	wIsValid;
}xFTHandle_t;


/* ===================================================
 * File-scope global variables
 * ===================================================
 */
static uint8_t	ucSwitchStat = 0;	/* Bit-wised status of power switches */
static uint8_t  ucSolenoidStat = 0;	/* Bit-wised status of solenoids */
static uint32_t	ulIsInit = 0;	/* 0 means Init function never run or fail */
static ZB_Status_t xCurrentStatus = ZB_STAT_OK;	/* The status of the previous operation */
/* Storage of device handles. The element 0 is for FT232H. All others are for FT4232H, which
has 4 modules inside. The wIsValid is firstly initialized with 0 means invalid */
static xFTHandle_t xFTDevHandle[5] = 
{
	{ .wIsValid = 0 },
	{ .wIsValid = 0 },
	{ .wIsValid = 0 },
	{ .wIsValid = 0 },
	{ .wIsValid = 0 }
};
static uint8_t aucUSBDataBuffer[USB_BULK_SIZE];

/* ===================================================
 * Local subroutines
 * ===================================================
 */
 
/* Initialize a device unit specified by xHandle to MPSSE mode (either I2C, SPI, or JTAG) */
static FT_STATUS InitMPSSE( FT_HANDLE xHandle )
{
	FT_STATUS xFTStatus;
	uint32_t dwNumBytesToDo, ulNumBytesDone;
	uint8_t ucEchoFound;
		
	/* Set initial status */
	xCurrentStatus = ZB_STAT_OK;
	
	/* Reset USB device */
	xFTStatus = FT_ResetDevice( xHandle );
	
	/* Purge USB receive buffer first by reading out all old data from receive buffer */
	xFTStatus |= FT_Purge( xHandle, ( FT_PURGE_TX | FT_PURGE_RX ) );
	
	/* Set USB request transfer sizes to 64K */
	xFTStatus |= FT_SetUSBParameters( xHandle, 65536, 65535);

	/* Disable event and error characters */
	xFTStatus |= FT_SetChars( xHandle, 0, 0, 0, 0);

	/* Sets the read and write timeouts in milliseconds */
	xFTStatus |= FT_SetTimeouts( xHandle, 3000, 3000);
		
	/* Set the latency timer to 1mS (default is 16mS) */
	xFTStatus |= FT_SetLatencyTimer( xHandle, 1);

	/* Reset controller */
	xFTStatus |= FT_SetBitMode( xHandle, 0x0, 0x00);
		
	/* Start MPSSE mode */
	xFTStatus |= FT_SetBitMode( xHandle, 0x0, 0x02);
		
	/* Disable Flow Control */
	xFTStatus |= FT_SetFlowControl( xHandle, FT_FLOW_NONE, 0, 0 );

	/* Wait 50ms for all operation to complete */
	(void)usleep( 50000 );
		
	/* If all above operations is successful, then synchronize MPSSE module 
	by sending a bogus command (0xAA). The device must response with "Bad Command" (0xFA)
	follow by the bogus command itself. */
	if( xFTStatus == FT_OK )
	{
		aucUSBDataBuffer[0] = 0xAA;
		/* Send off the BAD command */
		xFTStatus = FT_Write( xHandle, aucUSBDataBuffer, 1, &ulNumBytesDone );
		if( xFTStatus != FT_OK )
		{
			/* Fail to write a bogus command to the device */
			return( xFTStatus );
		}

		/* Wait for the response regardless of the timeout */
		do
		{
			xFTStatus = FT_GetQueueStatus( xHandle, &dwNumBytesToDo );
		} while ( ( dwNumBytesToDo == 0 ) && ( xFTStatus == FT_OK ) );

		if( xFTStatus != FT_OK )
		{
			/* Fail to check for response from the device */
			return( xFTStatus );
		}

		/* Read the response from the buffer */
		xFTStatus = FT_Read(xHandle, &aucUSBDataBuffer, dwNumBytesToDo, &ulNumBytesDone);

		if( xFTStatus != FT_OK )
		{
			/* Fail to read the response from the device */
			return( xFTStatus );
		}

		/* Check whether "Bad Command" and the bogus command received */
		ucEchoFound = 0;
		for( uint32_t i = 0; i < ( ulNumBytesDone - 1 ); i++ )
		{
			if( ( aucUSBDataBuffer[i] == 0xFA ) && ( aucUSBDataBuffer[ i + 1 ] == 0xAA ) )
			{
				ucEchoFound = 1;
				break;
			}
		}
			
		/* If no "Bad Command" was detected, the synchronization failed meaning that the unit failed */
		if( ucEchoFound == 0 )
		{
			xFTStatus = FT_IO_ERROR;
		}
	}
		
	return( xFTStatus );
}

/* Initialize a device unit specified by xHandle to MPSSE mode (either I2C, SPI, or JTAG) */
static FT_STATUS InitUART( FT_HANDLE xHandle, uint32_t ulBaudRate )
{
	FT_STATUS xFTStatus;
	uint32_t dwNumBytesToDo, ulNumBytesDone;
	uint8_t ucEchoFound;

	/* Set initial status */
	xCurrentStatus = ZB_STAT_OK;
	
	/* Reset USB device */
	xFTStatus = FT_ResetDevice( xHandle );
		
	/* Purge USB receive buffer first by reading out all old data from receive buffer */
	xFTStatus |= FT_Purge( xHandle, ( FT_PURGE_TX | FT_PURGE_RX ) );
		
	/* Set USB request transfer sizes to 64K */
	xFTStatus |= FT_SetUSBParameters( xHandle, 65536, 65535);

	/* Disable event and error characters */
	xFTStatus |= FT_SetChars( xHandle, 0, 0, 0, 0);

	/* Sets the read and write timeouts in milliseconds */
	xFTStatus |= FT_SetTimeouts( xHandle, 3000, 3000);
		
	/* Set the latency timer to 1mS (default is 16mS) */
	xFTStatus |= FT_SetLatencyTimer( xHandle, 1);
		
	/* Reset controller */
	xFTStatus |= FT_SetBitMode( xHandle, 0x0, 0x00);
		
	/* Set Baud-Rate */
	xFTStatus |= FT_SetBaudRate( xHandle, ulBaudRate );
		
	/* Set 8 data bits, 1 stop bit and no parity */
	xFTStatus |= FT_SetDataCharacteristics( xHandle, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE );
		
	/* Disable Flow Control */
	xFTStatus |= FT_SetFlowControl( xHandle, FT_FLOW_NONE, 0, 0 );

	/* Wait 50ms for all operation to complete */
	(void)usleep( 50000 );
		
	/* If all above operations is successful, then synchronize MPSSE module 
	by sending a bogus command (0xAA). The device must response with "Bad Command" (0xFA)
	follow by the bogus command itself. */
	if( xFTStatus == FT_OK )
	{
		aucUSBDataBuffer[0] = 0xAA;
		/* Send off the BAD command */
		xFTStatus = FT_Write( xHandle, aucUSBDataBuffer, 1, &ulNumBytesDone );
		if( xFTStatus != FT_OK )
		{
			/* Fail to write a bogus command to the device */
			return( xFTStatus );
		}

		/* Wait for the response regardless of the timeout */
		do
		{
			xFTStatus = FT_GetQueueStatus( xHandle, &dwNumBytesToDo );
		} while ( ( dwNumBytesToDo == 0 ) && ( xFTStatus == FT_OK ) );

		if( xFTStatus != FT_OK )
		{
			/* Fail to check for response from the device */
			return( xFTStatus );
		}

		/* Read the response from the buffer */
		xFTStatus = FT_Read(xHandle, &aucUSBDataBuffer, dwNumBytesToDo, &ulNumBytesDone);

		if( xFTStatus != FT_OK )
		{
			/* Fail to read the response from the device */
			return( xFTStatus );
		}

		/* Check whether "Bad Command" and the bogus command received */
		ucEchoFound = 0;
		for( uint32_t i = 0; i < ( ulNumBytesDone - 1 ); i++ )
		{
			if( ( aucUSBDataBuffer[i] == 0xFA ) && ( aucUSBDataBuffer[ i + 1 ] == 0xAA ) )
			{
				ucEchoFound = 1;
				break;
			}
		}
		
		/* If no "Bad Command" was detected, the synchronization failed meaning that the unit failed */
		if( ucEchoFound == 0 )
		{
			xFTStatus = FT_IO_ERROR;
		}
	}
		
	return( xFTStatus );
}

/* Update power switch according to the ucSwitchStat value */
static ZB_Status_t UpdateSwitch()
{
	FT_STATUS xFTStatus;
	uint32_t ulDataWritten;
	
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
	
	/* Set initial status */
	xCurrentStatus = ZB_STAT_OK;
	
	aucUSBDataBuffer[0] = SET_DATA_HIGH_BYTE;
	aucUSBDataBuffer[1] = ucSwitchStat;
	aucUSBDataBuffer[2] = DIR_232H_HI;
	xFTStatus = FT_Write( xFTDevHandle[0].xHandle, aucUSBDataBuffer, 3, &ulDataWritten );
	
	/* If switch-control failed */
	if( xFTStatus != FT_OK || ulDataWritten != 3 )
	{
		xCurrentStatus = ZB_STAT_POWER_SW_FAIL;
	}
	return( xCurrentStatus );
}

/* Update solenoid according to the ucSwitchStat value */
static ZB_Status_t UpdateSolenoid()
{
	FT_STATUS xFTStatus;
	uint32_t ulDataWritten;
	uint8_t ucPortData;
	
	/* Check whether all devices are initialized */
	if( ulIsInit == 0 )
	{
		xCurrentStatus = ZB_STAT_NEVER_INIT;
		return( xCurrentStatus );
	}
	
	/* Verify the device handle */
	if( ( xFTDevHandle[ 1 ].wIsValid == 0 ) || ( xFTDevHandle[ 2 ].wIsValid == 0 ) )
	{
		xCurrentStatus = ZB_STAT_PERIPHERAL_BRIDGE_FAIL;
		return( xCurrentStatus );
	}
	
	/* Set initial status */
	xCurrentStatus = ZB_STAT_OK;
	
	/* Send out the solenoid command */
	/* High nibble */
	ucPortData = ( ucSolenoidStat & 0xF0 ) | IDLE_MPSSE;	/* Set solenoid value only on high nibble while the other is idle value */
	aucUSBDataBuffer[0] = SET_DATA_LOW_BYTE;
	aucUSBDataBuffer[1] = ucPortData;
	aucUSBDataBuffer[2] = DIR_4232H_A;
	/* Low nibble */
	ucPortData = ( ucSolenoidStat << 4 ) | IDLE_MPSSE;	/* Set solenoid value only on high nibble while the other is idle value */
	aucUSBDataBuffer[3] = SET_DATA_LOW_BYTE;
	aucUSBDataBuffer[4] = ucPortData;
	aucUSBDataBuffer[5] = DIR_4232H_B;
	xFTStatus = FT_Write( xFTDevHandle[1].xHandle, aucUSBDataBuffer, 6, &ulDataWritten );
	
	/* If switch-control failed */
	if( xFTStatus != FT_OK || ulDataWritten != 6 )
	{
		xCurrentStatus = ZB_STAT_POWER_SW_FAIL;
	}
	return( xCurrentStatus );
}

/* Get the amount of data in Rx Buffer. These data should be read out by FT_Read later. */
static uint32_t CheckWaitingData( uint32_t ulIndex )
{
	FT_STATUS xFTStatus;
	uint32_t ulAmountData;

	/* Check the receive buffer - it should contain the data */
	xFTStatus = FT_GetQueueStatus( xFTDevHandle[ ulIndex ].xHandle, &ulAmountData );
	
	if( xFTStatus != FT_OK )
	{
		xCurrentStatus = ZB_STAT_PERIPHERAL_BRIDGE_FAIL;
		ulAmountData = 0;
	}
	else
	{
		xCurrentStatus = ZB_STAT_OK;
	}
	
	return( ulAmountData );
}

/* Write a stream of data to the specified handle index. 
The function returns the amount of actually written data. 
If the returned value differs from the aspected one, user can get the error status from 
ZB_GetErrorCode or ZB_GetErrorString */
static uint32_t WriteCom( uint32_t ulIndex, uint8_t* ucData, uint32_t ulLen )
{
	FT_STATUS xFTStatus;
	uint32_t ulDataWritten, ulAccWritten;
	
	/* Check whether all devices are initialized */
	if( ulIsInit == 0 )
	{
		xCurrentStatus = ZB_STAT_NEVER_INIT;
		return( 0 );
	}
	
	/* Verify the device handle */
	if( xFTDevHandle[ ulIndex ].wIsValid == 0 )
	{
		xCurrentStatus = ZB_STAT_PERIPHERAL_BRIDGE_FAIL;
		return( 0 );
	}
	
	/* Set initial status */
	xCurrentStatus = ZB_STAT_OK;
	
	/* Loop writing data when the amount of written data is still less than the required one 
	and FT_Write still return FT_OK */
	ulAccWritten = 0;
	do
	{
		xFTStatus = FT_Write( xFTDevHandle[ ulIndex ].xHandle, ucData + ulAccWritten, ulLen, &ulDataWritten );
		if( xFTStatus != FT_OK )
		{
			xCurrentStatus = ZB_STAT_COMM_WRITE_FAIL;
			break;
		}
		ulLen -= ulDataWritten;
		ulAccWritten += ulDataWritten;
	}while( ulLen > 0 );
	
	return( ulAccWritten );
}

/* Read a stream of data to the specified opened handle.
The function returns the amount of actually read data. 
If the returned value differs from the aspected one, user can get the error status from 
ZB_GetErrorCode or ZB_GetErrorString */
static uint32_t ReadCom( uint32_t ulIndex, uint8_t* ucData, uint32_t ulMaxLen )
{
	FT_STATUS xFTStatus;
	uint32_t ulDataToRead, ulTotalReceived;
	
	/* Check whether all devices are initialized */
	if( ulIsInit == 0 )
	{
		xCurrentStatus = ZB_STAT_NEVER_INIT;
		return( 0 );
	}
	
	/* Verify the device handle */
	if( xFTDevHandle[ ulIndex ].wIsValid == 0 )
	{
		xCurrentStatus = ZB_STAT_PERIPHERAL_BRIDGE_FAIL;
		return( 0 );
	}

	/* Check if there are any data waiting. There may be an error occurred with none */
	ulDataToRead = CheckWaitingData( ulIndex );
	if( ulDataToRead == 0 )
	{
		return( 0 );
	}
		
	/* If available amount of data is bigger than available buffer size, we limit the reading to the buffer size */
	if( ulDataToRead > ulMaxLen )
	{
		ulDataToRead = ulMaxLen;
	}
	
	/* Read the data */
	xFTStatus = FT_Read( xFTDevHandle[ ulIndex ].xHandle, ucData, ulDataToRead, &ulTotalReceived );

	/* The input buffer should contain 2 bytes of ADC data */
	if ( ( xFTStatus != FT_OK ) || ( ulTotalReceived != ulDataToRead ) )
	{
		xCurrentStatus = ZB_STAT_COMM_READ_FAIL;
	}
	else
	{
		xCurrentStatus = ZB_STAT_OK;
	}
	
	return( ulTotalReceived );  /* Return the actual amount of received data */
}

/* ===================================================
 * Global API subroutines
 * ===================================================
 */

/* Reset and initialize all FTxxxx chips. Then, prepare all setup for further operation */
ZB_Status_t ZB_Init( uint32_t ulComm1BaudRate, uint32_t ulComm2BaudRate )
{
	long iSysCallStat;
	FT_STATUS xFTStatus;
	FT_DEVICE_LIST_INFO_NODE *xDevInfo;
	uint32_t ulTotalDevs, ulNumFoundDev;
	uint32_t i, ulNumBytesDone;
	
	/* Initialize all handle */
	for( i = 0; i < 5; i++ )
	{
		/* Close all opened handles */
		if( xFTDevHandle[ i ].wIsValid == 1 )
		{
			FT_Close( xFTDevHandle[ i ].xHandle );
		}
		xFTDevHandle[ i ].wIsValid = 0;
	}
	
	/* Reset port status */
	ucSwitchStat = 0;
	ucSolenoidStat = 0;
	
	/* Remove all kernel modules that occupied the FTDI chips */
	iSysCallStat = syscall( SYS_delete_module, "ftdi_sio", O_NONBLOCK | O_TRUNC );
	if( ( iSysCallStat != 0 ) && ( errno != ENOENT ) )
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
	xFTStatus = FT_OpenEx( (void*)( cstPowerDistSerial ), FT_OPEN_BY_SERIAL_NUMBER, &( xFTDevHandle[ 0 ].xHandle ) );
	if (xFTStatus == FT_OK)
	{
		/* success - device with serial number "PowerDist" is open */
		xFTDevHandle[ 0 ].wIsValid = 1;
		ulNumFoundDev++;	/* Increase successfully discovered device number */
	}

	/* Attempt to open each Peripheral Bridge module described by its serial number */
	for( i = 0; i < 4; i++ )
	{
		xFTStatus = FT_OpenEx( (void*)( cstPeripheralSerial[ i ] ), FT_OPEN_BY_SERIAL_NUMBER, &( xFTDevHandle[ i + 1 ].xHandle ) );
		if (xFTStatus == FT_OK)
		{
			/* success - device with serial number "PeripheralBridge-?" is open */
			xFTDevHandle[ i + 1 ].wIsValid = 1;
			ulNumFoundDev++;	/* Increase successfully discovered device number */
		}
	}
	
	/* Free the memory allocated for the device information list */
	free( xDevInfo );
	
	/* Set the mode of each controller unit. As:
	   - The Power Distributor unit should be set as MSSP mode.
	   - The PeripheralBridge-A unit should be set to MSSP mode with SPI function.
	   - The PeripheralBridge-B unit should be set to MSSP mode.
	   - The PeripheralBridge-C unit should be set to UART mode without any flow-control.
	   - The PeripheralBridge-D unit should be set to UART mode without any flow-control.
	*/
	
	/* Setup the Power Distributor. The FT232H contains a two-byte unit. We use only
	the high byte to control power switches and leave the low byte including its MPSSE engine
	unused. */
	if( xFTDevHandle[0].wIsValid == 1 )
	{
		xFTStatus = InitMPSSE( xFTDevHandle[0].xHandle );

		/* If there all above operation completed successfully. */
		ulNumBytesDone = 0;		/* Initialize the amount of data to be sent */
		if (xFTStatus == FT_OK)
		{
			/* Setup all pin states. We do not have to set communication speed because we use only GPIO pins */
			aucUSBDataBuffer[0] = SET_DATA_LOW_BYTE;
			aucUSBDataBuffer[1] = IDLE_MPSSE; /* Pin state (Output data) */
			aucUSBDataBuffer[2] = DIR_232H_LO; /* All pins are output (bit value 1 means the corresponding pin is output) */
			aucUSBDataBuffer[3] = SET_DATA_HIGH_BYTE;
			aucUSBDataBuffer[4] = IDLE_GPIO;	/* Output data */
			aucUSBDataBuffer[5] = DIR_232H_HI; /* All pins are output (bit value 1 means the corresponding pin is output) */
			
			/* Write out the commands (2 commands at once) */
			xFTStatus = FT_Write( xFTDevHandle[0].xHandle, aucUSBDataBuffer, 6, &ulNumBytesDone );

			/* Wait for 30ms for USB operation to complete */
			(void)usleep( 30000 );
		}
		
		/* Check whether all initialization completed successfully */
		if( ( xFTStatus != FT_OK ) || ( ulNumBytesDone != 6 ) )
		{
			/* If somethings failed, we cannot use this port */
			ulNumFoundDev--;	/* Decrease number of usable unit */
			(void)FT_Close( xFTDevHandle[0].xHandle );
			xFTDevHandle[0].wIsValid = 0;
		}
	}
	
	/* Setup the PeripheralBridge-A. This unit contains only low-significant byte of data.
	The low nibble is use to connect to Analog-to-Digital chip via SPI protocol. The high
	nibble is use as GPIO pins to control solenoids */
	if( xFTDevHandle[1].wIsValid == 1 )
	{
		xFTStatus = InitMPSSE( xFTDevHandle[1].xHandle );

		/* If there all above operation completed successfully. */
		ulNumBytesDone = 0;		/* Initialize the amount of data to be sent */
		if (xFTStatus == FT_OK)
		{
			/* Send consecutive commands as:
			 * 1. Disable 1/5 clock divisor. Therefore, the SPI clock is computed from 60MHz.
			 * 2. Disable adaptive clocking.
			 * 3. Disable three-phase clocking used only in i2c.
			 * 4. Set SPI clock following the formular 60MHz / ( ( 1 + ValueH:ValueL ) * 2 ). (0x0007 means 3.75MHz)
			 * 5. Set pin states and directions. The pins are: (MSB)GPIO3:0, CS, DI, DO, CLK(LSB)
			 */
			aucUSBDataBuffer[0] = DISABLE_CLK_DIV;
			aucUSBDataBuffer[1] = DISABLE_ADAPTIVE_CLK;
			aucUSBDataBuffer[2] = DISABLE_THREE_PHASE;
			aucUSBDataBuffer[3] = SET_CLK_RATE; /* Set to 3.75MHz */
			aucUSBDataBuffer[4] = 0x07;		/* Lo-Byte */
			aucUSBDataBuffer[5] = 0;		/* Hi-Byte */
			aucUSBDataBuffer[6] = SET_DATA_LOW_BYTE;
			aucUSBDataBuffer[7] = IDLE_MPSSE; /* Pin state (Output data). CLK, DI, and CS are normally high. */
			aucUSBDataBuffer[8] = DIR_4232H_A; /* All pins are output except the DI pin (bit value 1 means the corresponding pin is output) */
			
			/* Write out the commands (2 commands at once) */
			xFTStatus = FT_Write( xFTDevHandle[1].xHandle, aucUSBDataBuffer, 9, &ulNumBytesDone );
			
			/* Wait for 30ms for USB operation to complete */
			(void)usleep( 30000 );
		}
		
		/* Check whether all initialization completed successfully */
		if( ( xFTStatus != FT_OK ) || ( ulNumBytesDone != 9 ) )
		{
			/* If somethings failed, we cannot use this port */
			ulNumFoundDev--;	/* Decrease number of usable unit */
			(void)FT_Close( xFTDevHandle[1].xHandle );
			xFTDevHandle[1].wIsValid = 0;
		}
	}
	
	/* Setup the PeripheralBridge-B. This unit contains only low-significant byte of data.
	The low nibble is unused. The high nibble is use as GPIO pins to control solenoids */
	if( xFTDevHandle[2].wIsValid == 1 )
	{
		xFTStatus = InitMPSSE( xFTDevHandle[2].xHandle );

		/* If there all above operation completed successfully. */
		ulNumBytesDone = 0;		/* Initialize the amount of data to be sent */
		if (xFTStatus == FT_OK)
		{
			/* Setup all pin states. We do not have to set communication speed because we use only GPIO pins */
			aucUSBDataBuffer[0] = SET_DATA_LOW_BYTE;
			aucUSBDataBuffer[1] = IDLE_GPIO; /* Pin state (Output data) */
			aucUSBDataBuffer[2] = DIR_4232H_B; /* All pins are output (bit value 1 means the corresponding pin is output) */
			
			/* Write out the commands (2 commands at once) */
			xFTStatus = FT_Write( xFTDevHandle[2].xHandle, aucUSBDataBuffer, 3, &ulNumBytesDone );
			
			/* Wait for 30ms for USB operation to complete */
			(void)usleep( 30000 );
		}
		
		/* Check whether all initialization completed successfully */
		if( ( xFTStatus != FT_OK ) || ( ulNumBytesDone != 3 ) )
		{
			/* If somethings failed, we cannot use this port */
			ulNumFoundDev--;	/* Decrease number of usable unit */
			(void)FT_Close( xFTDevHandle[2].xHandle );
			xFTDevHandle[2].wIsValid = 0;
		}
	}
	
	/* Setup the PeripheralBridge-C */
	if( xFTDevHandle[3].wIsValid == 1 )
	{
		xFTStatus = InitUART( xFTDevHandle[3].xHandle, ulComm1BaudRate );

		/* If there all above operation completed successfully. */
		ulNumBytesDone = 0;		/* Initialize the amount of data to be sent */
		if (xFTStatus == FT_OK)
		{
			/* Setup all pin states. We do not have to set communication speed because we use only GPIO pins */
			aucUSBDataBuffer[0] = SET_DATA_LOW_BYTE;
			aucUSBDataBuffer[1] = IDLE_UART; /* Pin state (Output data) */
			aucUSBDataBuffer[2] = DIR_4232H_C; /* All pins except bit-1 (RX) are output (bit value 1 means the corresponding pin is output) */
			
			/* Write out the commands (2 commands at once) */
			xFTStatus = FT_Write( xFTDevHandle[3].xHandle, aucUSBDataBuffer, 3, &ulNumBytesDone );
			
			/* Wait for 30ms for USB operation to complete */
			(void)usleep( 30000 );
		}
		
		/* Check whether all initialization completed successfully */
		if( ( xFTStatus != FT_OK ) || ( ulNumBytesDone != 3 ) )
		{
			/* If somethings failed, we cannot use this port */
			ulNumFoundDev--;	/* Decrease number of usable unit */
			(void)FT_Close( xFTDevHandle[3].xHandle );
			xFTDevHandle[3].wIsValid = 0;
		}
	}

	/* Setup the PeripheralBridge-D */
	if( xFTDevHandle[4].wIsValid == 1 )
	{
		xFTStatus = InitUART( xFTDevHandle[3].xHandle, ulComm2BaudRate );

		/* If there all above operation completed successfully. */
		ulNumBytesDone = 0;		/* Initialize the amount of data to be sent */
		if (xFTStatus == FT_OK)
		{
			/* Setup all pin states. We do not have to set communication speed because we use only GPIO pins */
			aucUSBDataBuffer[0] = SET_DATA_LOW_BYTE;
			aucUSBDataBuffer[1] = IDLE_UART; /* Pin state (Output data) */
			aucUSBDataBuffer[2] = DIR_4232H_D; /* All pins except bit-1 (RX) are output (bit value 1 means the corresponding pin is output) */
			
			/* Write out the commands (2 commands at once) */
			xFTStatus = FT_Write( xFTDevHandle[3].xHandle, aucUSBDataBuffer, 3, &ulNumBytesDone );
			
			/* Wait for 30ms for USB operation to complete */
			(void)usleep( 30000 );
		}
		
		/* Check whether all initialization completed successfully */
		if( ( xFTStatus != FT_OK ) || ( ulNumBytesDone != 3 ) )
		{
			/* If somethings failed, we cannot use this port */
			ulNumFoundDev--;	/* Decrease number of usable unit */
			(void)FT_Close( xFTDevHandle[3].xHandle );
			xFTDevHandle[3].wIsValid = 0;
		}
	}

	/* Verify that all required devices are found and opened */
	if( ulNumFoundDev == 5 )
	{
		xCurrentStatus = ZB_STAT_OK;
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

/* Return the maximum data size that can be send at once (i.e. USB packet size) */
uint32_t ZB_GetMaxDataSize()
{
	return( USB_BULK_SIZE );
}

/* Turn-on and turn-off power switches as assigned by ucSWMask. Multiple switches
could be turn-on or -off by using logical OR operator. For example, ZB_PWR_SW1 | ZB_PWR_SW2 */
ZB_Status_t ZB_SW_TurnOn( uint8_t ucSWMask )
{
	/* Send out the switch command */
	ucSwitchStat |= ucSWMask;	/* Include the switch status into the previous status. */
	return( UpdateSwitch() );
}

ZB_Status_t ZB_SW_TurnOff( uint8_t ucSWMask )
{
	/* Send out the switch command */
	ucSwitchStat &= ~(ucSWMask);  /* Include the switch status into the previous status. */
	return( UpdateSwitch() );
}

/* Turn-on and turn-off solenoid valves as assigned by ucSolMask. Multiple solenoids
could be turn-on or -off by using logical OR operator. For example, ZB_SOLENOID1 | ZB_SOLENOID2 */
ZB_Status_t ZB_Solenoid_TurnOn( uint8_t ucSolMask )
{
	/* Send out the solenoid command */
	ucSolenoidStat |= ucSolMask;	/* Include the solenoid status into the previous status. */
	return( UpdateSolenoid() );
}

ZB_Status_t ZB_Solenoid_TurnOff( uint8_t ucSolMask )
{
	/* Send out the solenoid command */
	ucSolenoidStat &= ~(ucSolMask);  /* Include the solenoid status into the previous status. */
	return( UpdateSolenoid() );
}

/* Write a stream of data to the specified RS232 port (Com1 or Com2). The function returns 
the amount of actually written data. If the returned value differs from the aspected one,
user can get the error status from ZB_GetErrorCode or ZB_GetErrorString */
uint32_t ZB_Write_Com1( uint8_t* ucData, uint32_t ulLen )
{
	return( WriteCom( 3, ucData, ulLen ) );
}

uint32_t ZB_Write_Com2( uint8_t* ucData, uint32_t ulLen )
{
	return( WriteCom( 4, ucData, ulLen ) );
}

/* Read a stream of data to the specified RS232 port (Com1 or Com2). The function returns 
the amount of actually read data. If the returned value differs from the aspected one,
user can get the error status from ZB_GetErrorCode or ZB_GetErrorString */
uint32_t ZB_Read_Com1( uint8_t* ucData, uint32_t ulMaxLen )
{
	return( ReadCom( 3, ucData, ulMaxLen ) );
}
uint32_t ZB_Read_Com2( uint8_t* ucData, uint32_t ulMaxLen )
{
	return( ReadCom( 4, ucData, ulMaxLen ) );
}

/* Read the value of the pressure sensor through the ADC chip. The ADC chip has 
10-bit precision; therefore, the valid returned value should be 0 - 1023.
Any other values outside this range indicates error of the reading. In error cases,
user must use the functions ZB_GetErrorCode or ZB_GetErrorString to determine the
error. */
uint16_t ZB_Read_Pressure()
{
	FT_STATUS xFTStatus;
	uint8_t ucSolenoidValue;
	uint32_t ulSizeDone;
	uint16_t usADCValue;
	
	/* Check whether all devices are initialized */
	if( ulIsInit == 0 )
	{
		xCurrentStatus = ZB_STAT_NEVER_INIT;
		return( 10000 );  /* The value outside the valid range */
	}
	
	/* Verify the device handle */
	if( xFTDevHandle[ 1 ].wIsValid == 0 )
	{
		xCurrentStatus = ZB_STAT_PERIPHERAL_BRIDGE_FAIL;
		return( 10000 );  /* The value outside the valid range */
	}

	/* Preserve the high nibble used for solenoids */
	ucSolenoidValue = ( ucSolenoidStat << 4 ) & 0xF0;
	
	/* Prepare the commands */
	
	/* Step 1: Set CS to low (active) */
	aucUSBDataBuffer[0] = SET_DATA_LOW_BYTE;
	aucUSBDataBuffer[1] = ucSolenoidValue | ( IDLE_MPSSE & 0x07 );
	
	/* Step 2: Get 16-bit data at rising edge clock without any data out. 
	The data size 0 means 1 byte and so on. */
	aucUSBDataBuffer[2] = MSB_RISING_EDGE_BYTE_IN;
	aucUSBDataBuffer[3] = 0x01;	/* Length low */
	aucUSBDataBuffer[4] = 0x00;	/* Length low */
	
	/* Step 3: Set CS back to high (idle) */
	aucUSBDataBuffer[5] = SET_DATA_LOW_BYTE;
	aucUSBDataBuffer[6] = ucSolenoidValue | IDLE_MPSSE;
	
	/* Write the commands */
	xFTStatus = FT_Write( xFTDevHandle[1].xHandle, aucUSBDataBuffer, 7, &ulSizeDone );
	
	if( ( xFTStatus != FT_OK ) || ( ulSizeDone != 7 ) )
	{
		xCurrentStatus = ZB_STAT_PRESSURE_SENSOR_FAIL;
		return( 10000 );  /* The value outside the valid range */
	}
	
	/* Check the receive buffer - it should contain the data */
	xFTStatus = FT_GetQueueStatus( xFTDevHandle[1].xHandle, &ulSizeDone );
	
	/* Read the data */
	xFTStatus |= FT_Read( xFTDevHandle[1].xHandle, aucUSBDataBuffer, 2, &ulSizeDone );

	/* The input buffer should contain 2 bytes of ADC data */
	if ( ( xFTStatus != FT_OK ) || ( ulSizeDone != 2 ) )
	{
		xCurrentStatus = ZB_STAT_PRESSURE_SENSOR_FAIL;
		return( 10000 );  /* The value outside the valid range */
	}
	
	/* Process the data. The input is MSB first with 3 leading zero-bits and 3 tailing zero-bits */
	usADCValue = (uint16_t)( aucUSBDataBuffer[ 0 ] ); /* High byte */
	usADCValue <<= 8;
	usADCValue |= (uint16_t)( aucUSBDataBuffer[ 1 ] ); /* Low byte */
	usADCValue >>= 3;	/* Suppress 3 tailing zero bits */

	return( usADCValue );
}
