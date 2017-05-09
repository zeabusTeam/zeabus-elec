#ifndef __FTDI_IMPL_H
#define __FTDI_IMPL_H

#include <stdint.h>
#include "ftd2xx.h"

/*=====================================================================================*/
/* This HPP file and its corresponding CPP file are the thin-layer implementation of   */
/* low level operation with FTDI chips. They are central operations for all FTDI chips */
/*=====================================================================================*/

const uint32_t	ulDefaultBaudRate = 115200;

namespace Zeabus_Elec
{

	/* The base class used for all implementation of FTDI modes */
	class ftdi_impl
	{
	public:
		ftdi_impl( FT_DEVICE xFTDevice, FT_HANDLE xHandle ) 
			: xFTDevice_(xFTDevice), xHandle_(xHandle), xCurrentStatus_( FT_OK ) {};

		FT_STATUS GetCurrentStatus() { return xCurrentStatus_; };
		void ClearError() { xCurrentStatus_ = FT_OK; };
		
		/* Abstract method */
		/* Send data to communication channel */
		virtual uint32_t Send( uint8_t* , uint32_t ) = 0;
		/* Receive data from communication channel */
		virtual uint32_t Receive( uint8_t* , uint32_t ) = 0;
		/* Set I/O direction of GPIO pins (if available). The parameter is the mask. 0 = Input, 1 = Output */
		virtual FT_STATUS SetGPIODirection( uint16_t ) = 0;
		/* Out data to high-byte GPIO pins */
		virtual FT_STATUS SetHiGPIOData( uint8_t ) = 0;
		/* Out data to low-byte GPIO pins */
		virtual FT_STATUS SetLoGPIOData( uint8_t ) = 0;
		/* Read data from high-byte GPIO pins */
		virtual uint8_t ReadHiGPIOData() = 0;
		/* Read data from low-byte GPIO pins */
		virtual uint8_t ReadLoGPIOData() = 0;
		
		/* Get the amount of data in Rx Buffer. These data should be read out by FT_Read later. */
		uint32_t CheckWaitingData()
		{
			uint32_t ulAmountData;
			
			if( xCurrentStatus_ != FT_OK )
			{
				return( 0 );
			}

			/* Check the receive buffer - it should contain the data */
			xCurrentStatus_ = FT_GetQueueStatus( xHandle_, &ulAmountData );
	
			if( xCurrentStatus_ != FT_OK )
			{
				ulAmountData = 0;
			}
	
			return( ulAmountData );
		};

	protected:
		FT_HANDLE xHandle_;
		FT_DEVICE xFTDevice_;
		FT_STATUS xCurrentStatus_;
	};
	/*===================================================================*/

	/* Basic implementation class for MSSP mode without any particular protocol (e.g. SPI, I2C). */
	class ftdi_mssp_impl : public ftdi_impl
	{
	public:
		ftdi_mssp_impl( FT_DEVICE xFTDevice, FT_HANDLE xHandle );
		
		/* Send data to communication channel */
		virtual uint32_t Send( uint8_t* pucData, uint32_t ulLen );
		/* Receive data from communication channel */
		virtual uint32_t Receive( uint8_t* pucData, uint32_t ulMaxLen );
		/* Set I/O direction of GPIO pins (if available). The parameter is the mask. 0 = Input, 1 = Output */
		virtual FT_STATUS SetGPIODirection( uint16_t usData);
		/* Out data to high-byte GPIO pins */
		virtual FT_STATUS SetHiGPIOData( uint8_t ucData );
		/* Out data to low-byte GPIO pins */
		virtual FT_STATUS SetLoGPIOData( uint8_t ucData );
		/* Read data from high-byte GPIO pins */
		virtual uint8_t ReadHiGPIOData();
		/* Read data from low-byte GPIO pins */
		virtual uint8_t ReadLoGPIOData();
	
	protected:
		uint16_t usIODirection_;
	};
	/*===================================================================*/
	
	/* Implementation class for UART mode */
	class ftdi_uart_impl : public ftdi_impl
	{
	public:
		ftdi_uart_impl( FT_DEVICE xFTDevice, FT_HANDLE xHandle, uint32_t ulBaudRate = ulDefaultBaudRate );
		
		/* Change baud rate */
		FT_STATUS SetBaudRate( uint32_t ulBaudRate );
		
		/* Send data to communication channel */
		virtual uint32_t Send( uint8_t* pucData, uint32_t ulLen );
		/* Receive data from communication channel */
		virtual uint32_t Receive( uint8_t* pucData, uint32_t ulMaxLen );
		/* Set I/O direction of GPIO pins (if available). The parameter is the mask. 0 = Input, 1 = Output */
		virtual FT_STATUS SetGPIODirection( uint16_t );
		/* Out data to high-byte GPIO pins */
		virtual FT_STATUS SetHiGPIOData( uint8_t );
		/* Out data to low-byte GPIO pins */
		virtual FT_STATUS SetLoGPIOData( uint8_t );
		/* Read data from high-byte GPIO pins */
		virtual uint8_t ReadHiGPIOData();
		/* Read data from low-byte GPIO pins */
		virtual uint8_t ReadLoGPIOData();
	
	protected:
	};
	/*===================================================================*/
}

#endif