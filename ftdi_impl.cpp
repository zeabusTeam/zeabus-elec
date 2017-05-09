#include <sys/syscall.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ftdi_impl.hpp"

using namespace Zeabus_Elec;

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

/*=====================================================================================*
 * Basic implementation for MSSP mode without any particular protocol (e.g. SPI, I2C). *
 *=====================================================================================*/

/* Initialize a device unit specified by xHandle to MPSSE mode (either I2C, SPI, or JTAG) */
ftdi_mssp_impl::ftdi_mssp_impl( FT_DEVICE xFTDevice, FT_HANDLE xHandle )
	: ftdi_impl( xFTDevice, xHandle )
{
	uint32_t dwNumBytesToDo, ulNumBytesDone;
	uint8_t ucEchoFound, aucUSBDataBuffer[2];
	
	/* Check for the supported device. MSSP exists only in FT2232D, FT2232H, FT232H, and FT4232H only */
	if( xFTDevice != FT_DEVICE_2232C &&		/* FT2232C/D/L */
		xFTDevice != FT_DEVICE_2232H &&		/* FT2232H */
		xFTDevice != FT_DEVICE_4232H &&		/* FT4232H */
		xFTDevice != FT_DEVICE_232H	)		/* FT232H */
	{
		xCurrentStatus_ = FT_NOT_SUPPORTED;
		return;
	}
		
	/* Reset USB device */
	xCurrentStatus_ = FT_ResetDevice( xHandle );
	
	/* Purge USB receive buffer first by reading out all old data from receive buffer */
	xCurrentStatus_ |= FT_Purge( xHandle, ( FT_PURGE_TX | FT_PURGE_RX ) );
	
	/* Set USB request transfer sizes to 64K */
	xCurrentStatus_ |= FT_SetUSBParameters( xHandle, 65536, 65535);

	/* Disable event and error characters */
	xCurrentStatus_ |= FT_SetChars( xHandle, 0, 0, 0, 0);

	/* Sets the read and write timeouts in milliseconds */
	xCurrentStatus_ |= FT_SetTimeouts( xHandle, 3000, 3000);
		
	/* Set the latency timer to 1mS (default is 16mS) */
	xCurrentStatus_ |= FT_SetLatencyTimer( xHandle, 1);

	/* Reset controller */
	xCurrentStatus_ |= FT_SetBitMode( xHandle, 0x0, 0x00);
		
	/* Start MPSSE mode */
	xCurrentStatus_ |= FT_SetBitMode( xHandle, 0x0, 0x02);
		
	/* Disable Flow Control */
	xCurrentStatus_ |= FT_SetFlowControl( xHandle, FT_FLOW_NONE, 0, 0 );

	/* Wait 50ms for all operation to complete */
	(void)usleep( 50000 );
		
	/* If all above operations is successful, then synchronize MPSSE module 
	by sending a bogus command (0xAA). The device must response with "Bad Command" (0xFA)
	follow by the bogus command itself. */
	if( xCurrentStatus_ == FT_OK )
	{
		aucUSBDataBuffer[0] = 0xAA;
		/* Send off the BAD command */
		xCurrentStatus_ = FT_Write( xHandle, aucUSBDataBuffer, 1, &ulNumBytesDone );
		if( xCurrentStatus_ != FT_OK )
		{
			/* Fail to write a bogus command to the device */
			return;
		}

		/* Wait for the response regardless of the timeout */
		do
		{
			xCurrentStatus_ = FT_GetQueueStatus( xHandle, &dwNumBytesToDo );
		} while ( ( dwNumBytesToDo == 0 ) && ( xCurrentStatus_ == FT_OK ) );

		if( xCurrentStatus_ != FT_OK )
		{
			/* Fail to check for response from the device */
			return;
		}

		/* Read the response from the buffer */
		xCurrentStatus_ = FT_Read(xHandle, &aucUSBDataBuffer, dwNumBytesToDo, &ulNumBytesDone);

		if( xCurrentStatus_ != FT_OK )
		{
			/* Fail to read the response from the device */
			return;
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
			xCurrentStatus_ = FT_IO_ERROR;
		}
	}
	
	if( xCurrentStatus_ == FT_OK )
	{
		/* All basic MSSP initialization completed successfully. However, we still 
		   cannot send any data to communication channel because we still not configured 
		   for the protocol yet. But, we can use GPIO pin now. Therefore, we set the
		   error status to other error for now. */
		xCurrentStatus_ = FT_OTHER_ERROR;
	}
}

/* Send data to communication channel */
inline uint32_t ftdi_mssp_impl::Send( uint8_t* , uint32_t )
{
	/* Plain MSSP mode is not ready for communication */
	xCurrentStatus_ = FT_NOT_SUPPORTED;
	return( 0 );
}

/* Receive data from communication channel */
inline uint32_t ftdi_mssp_impl::Receive( uint8_t* , uint32_t )
{
	/* Plain MSSP mode is not ready for communication */
	xCurrentStatus_ = FT_NOT_SUPPORTED;
	return( 0 );
}

/* Set I/O direction of GPIO pins (if available). The parameter is the mask. 0 = Input, 1 = Output */
FT_STATUS ftdi_mssp_impl::SetGPIODirection( uint16_t usData )
{
	uint8_t aucUSBDataBuffer[3];
	uint32_t ulNumBytesDone;
	
	usIODirection_ = usData;

	aucUSBDataBuffer[0] = SET_DATA_LOW_BYTE;
	aucUSBDataBuffer[1] = 0; /* Pin state (Output data) */
	aucUSBDataBuffer[2] =  static_cast<uint8_t>( usData & 0xFF ); /* Get only low byte */
	
	/* Write out the command */
	xCurrentStatus_ = FT_Write( xHandle_, aucUSBDataBuffer, 3, &ulNumBytesDone );
	
	if( ( xFTDevice_ != FT_DEVICE_4232H ) && ( xCurrentStatus_ == FT_OK ) )	/* FT4232H only has low-byte GPIO */
	{
		aucUSBDataBuffer[0] = SET_DATA_HIGH_BYTE;
		aucUSBDataBuffer[1] = 0;	/* Output data */
		aucUSBDataBuffer[2] = static_cast<uint8_t>( ( usData >> 8 ) & 0xFF ); /* High byte direction */
			
		/* Write out the command */
		xCurrentStatus_ |= FT_Write( xHandle_, aucUSBDataBuffer, 3, &ulNumBytesDone );
	}
	
	return( xCurrentStatus_ );
}

/* Out data to high-byte GPIO pins */
FT_STATUS ftdi_mssp_impl::SetHiGPIOData( uint8_t ucData )
{
	uint8_t aucUSBDataBuffer[3];
	uint32_t ulNumBytesDone;
	
	if( xFTDevice_ != FT_DEVICE_4232H )
	{
		xCurrentStatus_ = FT_NOT_SUPPORTED;		/* FT4232H only has low-byte GPIO */
	}
	else
	{
		aucUSBDataBuffer[0] = SET_DATA_HIGH_BYTE;
		aucUSBDataBuffer[1] = ucData;	/* Output data */
		aucUSBDataBuffer[2] = static_cast<uint8_t>( ( usIODirection_ >> 8 ) & 0xFF ); /* High byte direction */

		/* Write out the command */
		xCurrentStatus_ = FT_Write( xHandle_, aucUSBDataBuffer, 3, &ulNumBytesDone );
	}
	
	return( xCurrentStatus_ );
}

/* Out data to low-byte GPIO pins */
FT_STATUS ftdi_mssp_impl::SetLoGPIOData( uint8_t ucData )
{
	uint8_t aucUSBDataBuffer[3];
	uint32_t ulNumBytesDone;
	
	aucUSBDataBuffer[0] = SET_DATA_HIGH_BYTE;
	aucUSBDataBuffer[1] = ucData;	/* Output data */
	aucUSBDataBuffer[2] = static_cast<uint8_t>( usIODirection_ & 0xFF ); /* Low byte direction */

	/* Write out the command */
	xCurrentStatus_ = FT_Write( xHandle_, aucUSBDataBuffer, 3, &ulNumBytesDone );
	
	return( xCurrentStatus_ );
}

/* Read data from high-byte GPIO pins */
uint8_t ftdi_mssp_impl::ReadHiGPIOData()
{
	uint8_t aucUSBDataBuffer[2];
	uint32_t ulNumBytesDone;
	
	if( xFTDevice_ != FT_DEVICE_4232H )
	{
		xCurrentStatus_ = FT_NOT_SUPPORTED;		/* FT4232H has only low-byte GPIO */
	}
	else
	{
		aucUSBDataBuffer[0] = GET_DATA_HIGH_BYTE;

		/* Write out the command */
		xCurrentStatus_ = FT_Write( xHandle_, aucUSBDataBuffer, 1, &ulNumBytesDone );
		
		if( xCurrentStatus_ == FT_OK ) /* Command sent successfully */
		{
			xCurrentStatus_ |= FT_Read( xHandle_, aucUSBDataBuffer, 1, &ulNumBytesDone );
		}
	}
	
	if( xCurrentStatus_ == FT_OK )
	{
		return( aucUSBDataBuffer[0] );
	}
	else
	{
		return( 0 );
	}
}

/* Read data from low-byte GPIO pins */
uint8_t ftdi_mssp_impl::ReadLoGPIOData()
{
	uint8_t aucUSBDataBuffer[2];
	uint32_t ulNumBytesDone;
	
	aucUSBDataBuffer[0] = GET_DATA_LOW_BYTE;

	/* Write out the command */
	xCurrentStatus_ = FT_Write( xHandle_, aucUSBDataBuffer, 1, &ulNumBytesDone );
	
	if( xCurrentStatus_ == FT_OK ) /* Command sent successfully */
	{
		xCurrentStatus_ = FT_Read( xHandle_, aucUSBDataBuffer, 1, &ulNumBytesDone );
	}
	
	if( xCurrentStatus_ == FT_OK )
	{
		return( aucUSBDataBuffer[0] );
	}
	else
	{
		return( 0 );
	}
}
	
/*===================================================================*/

/*===================================================================*/
/* Implementation body of UART                                       */
/*===================================================================*/

/* Initialize a device unit specified by xHandle to MPSSE mode (either I2C, SPI, or JTAG) */
ftdi_uart_impl::ftdi_uart_impl( FT_DEVICE xFTDevice, FT_HANDLE xHandle, uint32_t ulBaudRate )
	: ftdi_impl( xFTDevice, xHandle )
{
	uint32_t dwNumBytesToDo, ulNumBytesDone;
	uint8_t ucEchoFound, aucUSBDataBuffer[2];

	/* Reset USB device */
	xCurrentStatus_ = FT_ResetDevice( xHandle );
		
	/* Purge USB receive buffer first by reading out all old data from receive buffer */
	xCurrentStatus_ |= FT_Purge( xHandle, ( FT_PURGE_TX | FT_PURGE_RX ) );
		
	/* Set USB request transfer sizes to 64K */
	xCurrentStatus_ |= FT_SetUSBParameters( xHandle, 65536, 65535);

	/* Disable event and error characters */
	xCurrentStatus_ |= FT_SetChars( xHandle, 0, 0, 0, 0);

	/* Sets the read and write timeouts in milliseconds */
	xCurrentStatus_ |= FT_SetTimeouts( xHandle, 3000, 3000);
		
	/* Set the latency timer to 1mS (default is 16mS) */
	xCurrentStatus_ |= FT_SetLatencyTimer( xHandle, 1);
		
	/* Reset controller */
	xCurrentStatus_ |= FT_SetBitMode( xHandle, 0x0, 0x00);
		
	/* Set Baud-Rate */
	xCurrentStatus_ |= FT_SetBaudRate( xHandle, ulBaudRate );
		
	/* Set 8 data bits, 1 stop bit and no parity */
	xCurrentStatus_ |= FT_SetDataCharacteristics( xHandle, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE );
		
	/* Disable Flow Control */
	xCurrentStatus_ |= FT_SetFlowControl( xHandle, FT_FLOW_NONE, 0, 0 );

	/* Wait 50ms for all operation to complete */
	(void)usleep( 50000 );
		
	/* If all above operations is successful, then synchronize MPSSE module 
	by sending a bogus command (0xAA). The device must response with "Bad Command" (0xFA)
	follow by the bogus command itself. */
	if( xCurrentStatus_ == FT_OK )
	{
		aucUSBDataBuffer[0] = 0xAA;
		/* Send off the BAD command */
		xCurrentStatus_ = FT_Write( xHandle, aucUSBDataBuffer, 1, &ulNumBytesDone );
		if( xCurrentStatus_ != FT_OK )
		{
			/* Fail to write a bogus command to the device */
			return;
		}

		/* Wait for the response regardless of the timeout */
		do
		{
			xCurrentStatus_ = FT_GetQueueStatus( xHandle, &dwNumBytesToDo );
		} while ( ( dwNumBytesToDo == 0 ) && ( xCurrentStatus_ == FT_OK ) );

		if( xCurrentStatus_ != FT_OK )
		{
			/* Fail to check for response from the device */
			return;
		}

		/* Read the response from the buffer */
		xCurrentStatus_ = FT_Read(xHandle, &aucUSBDataBuffer, dwNumBytesToDo, &ulNumBytesDone);

		if( xCurrentStatus_ != FT_OK )
		{
			/* Fail to read the response from the device */
			return;
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
			xCurrentStatus_ = FT_IO_ERROR;
		}
	}
}

/* Change baud rate */
FT_STATUS ftdi_uart_impl::SetBaudRate( uint32_t ulBaudRate )
{
	if( xCurrentStatus_ == FT_OK )
	{
		/* Set Baud-Rate */
		xCurrentStatus_ = FT_SetBaudRate( xHandle_, ulBaudRate );
	}
	
	return( xCurrentStatus_ );
}

/* Send data to communication channel */
uint32_t ftdi_uart_impl::Send( uint8_t* pucData, uint32_t ulLen )
{
	uint32_t ulDataWritten, ulAccWritten;
	
	/* Loop writing data when the amount of written data is still less than the required one 
	and FT_Write still return FT_OK */
	ulAccWritten = 0;
	do
	{
		xCurrentStatus_ = FT_Write( xHandle_, pucData + ulAccWritten, ulLen, &ulDataWritten );
		if( xCurrentStatus_ != FT_OK )
		{
			xCurrentStatus_ = FT_FAILED_TO_WRITE_DEVICE;
			break;
		}
		ulLen -= ulDataWritten;
		ulAccWritten += ulDataWritten;
	}while( ulLen > 0 );
	
	return( ulAccWritten );
}

/* Receive data from communication channel */
uint32_t ftdi_uart_impl::Receive( uint8_t* pucData, uint32_t ulMaxLen )
{
	uint32_t ulDataToRead, ulTotalReceived;
	
	/* Check if there are any data waiting. There may be an error occurred with none */
	xCurrentStatus_ = FT_GetQueueStatus( xHandle_, &ulDataToRead );
	
	if( xCurrentStatus_ != FT_OK )
	{
		ulDataToRead = 0;
	}

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
	xCurrentStatus_ = FT_Read( xHandle_, pucData, ulDataToRead, &ulTotalReceived );

	return( ulTotalReceived );  /* Return the actual amount of received data */
}

/* Set I/O direction of GPIO pins (if available). The parameter is the mask. 0 = Input, 1 = Output */
inline FT_STATUS ftdi_uart_impl::SetGPIODirection( uint16_t )
{
	/* UART mode does not have any GPIO */
	xCurrentStatus_ = FT_NOT_SUPPORTED;
	return( FT_NOT_SUPPORTED );
}

/* Out data to high-byte GPIO pins */
inline FT_STATUS ftdi_uart_impl::SetHiGPIOData( uint8_t )
{
	/* UART mode does not have any GPIO */
	xCurrentStatus_ = FT_NOT_SUPPORTED;
	return( FT_NOT_SUPPORTED );
}

/* Out data to low-byte GPIO pins */
inline FT_STATUS ftdi_uart_impl::SetLoGPIOData( uint8_t )
{
	/* UART mode does not have any GPIO */
	xCurrentStatus_ = FT_NOT_SUPPORTED;
	return( FT_NOT_SUPPORTED );
}

/* Read data from high-byte GPIO pins */
inline uint8_t ftdi_uart_impl::ReadHiGPIOData()
{
	/* UART mode does not have any GPIO */
	xCurrentStatus_ = FT_NOT_SUPPORTED;
	return( 0 );
}

/* Read data from low-byte GPIO pins */
inline uint8_t ftdi_uart_impl::ReadLoGPIOData()
{
	/* UART mode does not have any GPIO */
	xCurrentStatus_ = FT_NOT_SUPPORTED;
	return( 0 );
}
	
/*===================================================================*/
