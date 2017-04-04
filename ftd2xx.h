/*++

Copyright © 2001-2011 Future Technology Devices International Limited

THIS SOFTWARE IS PROVIDED BY FUTURE TECHNOLOGY DEVICES INTERNATIONAL LIMITED "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
FUTURE TECHNOLOGY DEVICES INTERNATIONAL LIMITED BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

FTDI DRIVERS MAY BE USED ONLY IN CONJUNCTION WITH PRODUCTS BASED ON FTDI PARTS.

FTDI DRIVERS MAY BE DISTRIBUTED IN ANY FORM AS uint32_t AS LICENSE INFORMATION IS NOT MODIFIED.

IF A CUSTOM VENDOR ID AND/OR PRODUCT ID OR DESCRIPTION STRING ARE USED, IT IS THE
RESPONSIBILITY OF THE PRODUCT MANUFACTURER TO MAINTAIN ANY CHANGES AND SUBSEQUENT WHQL
RE-CERTIFICATION AS A RESULT OF MAKING THESE CHANGES.


Module Name:

ftd2xx.h

Abstract:

Native USB device driver for FTDI FT232x, FT245x, FT2232x and FT4232x devices
FTD2XX library definitions

Environment:

kernel & user mode


--*/


#ifndef FTD2XX_H
#define FTD2XX_H

#include <stdint.h>

#ifdef _WIN32
// Compiling on Windows
#include <windows.h>

// The following ifdef block is the standard way of creating macros
// which make exporting from a DLL simpler.  All files within this DLL
// are compiled with the FTD2XX_EXPORTS symbol defined on the command line.
// This symbol should not be defined on any project that uses this DLL.
// This way any other project whose source files include this file see
// FTD2XX_API functions as being imported from a DLL, whereas this DLL
// sees symbols defined with this macro as being exported.

#ifdef FTD2XX_EXPORTS
#define FTD2XX_API __declspec(dllexport)
#elif defined(FTD2XX_STATIC)
// Avoid decorations when linking statically to D2XX.
#define FTD2XX_API
#else
#define FTD2XX_API __declspec(dllimport)
#endif

#else // _WIN32
// Compiling on non-Windows platform.
//#include "WinTypes.h"
#include <pthread.h>

typedef struct timeval SYSTEMTIME;
typedef struct timeval FILETIME;
typedef unsigned int BOOL;
#ifndef TRUE
#define TRUE    1
#endif
#ifndef FALSE
#define FALSE    0
#endif

typedef struct _OVERLAPPED {
    uint32_t Internal;
    uint32_t InternalHigh;
    uint32_t Offset;
    uint32_t OffsetHigh;
    void* hEvent;
} OVERLAPPED, *LPOVERLAPPED;

typedef struct _SECURITY_ATTRIBUTES {
    uint32_t nLength;
    void* lpSecurityDescriptor;
    BOOL bInheritHandle;
} SECURITY_ATTRIBUTES , *LPSECURITY_ATTRIBUTES;

// Substitute for HANDLE returned by Windows CreateEvent API.
// FT_SetEventNotification expects parameter 3 to be the address
// of one of these structures.
typedef struct _EVENT_HANDLE
{
    pthread_cond_t  eCondVar;
    pthread_mutex_t eMutex;
    int             iVar;
} EVENT_HANDLE;

//
// Modem Status Flags
//
#define MS_CTS_ON           ((uint32_t)0x0010)
#define MS_DSR_ON           ((uint32_t)0x0020)
#define MS_RING_ON          ((uint32_t)0x0040)
#define MS_RLSD_ON          ((uint32_t)0x0080)

//
// Error Flags
//
#define CE_RXOVER           0x0001  // Receive Queue overflow
#define CE_OVERRUN          0x0002  // Receive Overrun Error
#define CE_RXPARITY         0x0004  // Receive Parity Error
#define CE_FRAME            0x0008  // Receive Framing error
#define CE_BREAK            0x0010  // Break Detected
#define CE_TXFULL           0x0100  // TX Queue is full
#define CE_PTO              0x0200  // LPTx Timeout
#define CE_IOE              0x0400  // LPTx I/O Error
#define CE_DNS              0x0800  // LPTx Device not selected
#define CE_OOP              0x1000  // LPTx Out-Of-Paper
#define CE_MODE             0x8000  // Requested mode unsupported

//
// Events
//
#define EV_RXCHAR           0x0001  // Any Character received
#define EV_RXFLAG           0x0002  // Received certain character
#define EV_TXEMPTY          0x0004  // Transmit Queue Empty
#define EV_CTS              0x0008  // CTS changed state
#define EV_DSR              0x0010  // DSR changed state
#define EV_RLSD             0x0020  // RLSD changed state
#define EV_BREAK            0x0040  // BREAK received
#define EV_ERR              0x0080  // Line status error occurred
#define EV_RING             0x0100  // Ring signal detected
#define EV_PERR             0x0200  // Printer error occured
#define EV_RX80FULL         0x0400  // Receive buffer is 80 percent full
#define EV_EVENT1           0x0800  // Provider specific event 1
#define EV_EVENT2           0x1000  // Provider specific event 2

//
// Escape Functions
//
#define SETXOFF             1       // Simulate XOFF received
#define SETXON              2       // Simulate XON received
#define SETRTS              3       // Set RTS high
#define CLRRTS              4       // Set RTS low
#define SETDTR              5       // Set DTR high
#define CLRDTR              6       // Set DTR low
#define RESETDEV            7       // Reset device if possible
#define SETBREAK            8       // Set the device break line.
#define CLRBREAK            9       // Clear the device break line.

//
// PURGE function flags.
//
#define PURGE_TXABORT       0x0001  // Kill the pending/current writes to the comm port.
#define PURGE_RXABORT       0x0002  // Kill the pending/current reads to the comm port.
#define PURGE_TXCLEAR       0x0004  // Kill the transmit queue if there.
#define PURGE_RXCLEAR       0x0008  // Kill the typeahead buffer if there.

#ifndef INVALID_HANDLE_VALUE
#define INVALID_HANDLE_VALUE 0xFFFFFFFF
#endif

// No decorations needed.
#define FTD2XX_API

#endif // _WIN32

typedef void*		FT_HANDLE;
typedef uint32_t	FT_STATUS;

//
// Device status
//
enum {
	FT_OK,
	FT_INVALID_HANDLE,
	FT_DEVICE_NOT_FOUND,
	FT_DEVICE_NOT_OPENED,
	FT_IO_ERROR,
	FT_INSUFFICIENT_RESOURCES,
	FT_INVALID_PARAMETER,
	FT_INVALID_BAUD_RATE,

	FT_DEVICE_NOT_OPENED_FOR_ERASE,
	FT_DEVICE_NOT_OPENED_FOR_WRITE,
	FT_FAILED_TO_WRITE_DEVICE,
	FT_EEPROM_READ_FAILED,
	FT_EEPROM_WRITE_FAILED,
	FT_EEPROM_ERASE_FAILED,
	FT_EEPROM_NOT_PRESENT,
	FT_EEPROM_NOT_PROGRAMMED,
	FT_INVALID_ARGS,
	FT_NOT_SUPPORTED,
	FT_OTHER_ERROR,
	FT_DEVICE_LIST_NOT_READY,
};


#define FT_SUCCESS(status) ((status) == FT_OK)

//
// FT_OpenEx Flags
//

#define FT_OPEN_BY_SERIAL_NUMBER	1
#define FT_OPEN_BY_DESCRIPTION		2
#define FT_OPEN_BY_LOCATION			4

#define FT_OPEN_MASK (FT_OPEN_BY_SERIAL_NUMBER | \
                      FT_OPEN_BY_DESCRIPTION | \
                      FT_OPEN_BY_LOCATION)

//
// FT_ListDevices Flags (used in conjunction with FT_OpenEx Flags
//

#define FT_LIST_NUMBER_ONLY			0x80000000
#define FT_LIST_BY_INDEX			0x40000000
#define FT_LIST_ALL					0x20000000

#define FT_LIST_MASK (FT_LIST_NUMBER_ONLY|FT_LIST_BY_INDEX|FT_LIST_ALL)

//
// Baud Rates
//

#define FT_BAUD_300			300
#define FT_BAUD_600			600
#define FT_BAUD_1200		1200
#define FT_BAUD_2400		2400
#define FT_BAUD_4800		4800
#define FT_BAUD_9600		9600
#define FT_BAUD_14400		14400
#define FT_BAUD_19200		19200
#define FT_BAUD_38400		38400
#define FT_BAUD_57600		57600
#define FT_BAUD_115200		115200
#define FT_BAUD_230400		230400
#define FT_BAUD_460800		460800
#define FT_BAUD_921600		921600

//
// Word Lengths
//

#define FT_BITS_8			(uint8_t) 8
#define FT_BITS_7			(uint8_t) 7

//
// Stop Bits
//

#define FT_STOP_BITS_1		(uint8_t) 0
#define FT_STOP_BITS_2		(uint8_t) 2

//
// Parity
//

#define FT_PARITY_NONE		(uint8_t) 0
#define FT_PARITY_ODD		(uint8_t) 1
#define FT_PARITY_EVEN		(uint8_t) 2
#define FT_PARITY_MARK		(uint8_t) 3
#define FT_PARITY_SPACE		(uint8_t) 4

//
// Flow Control
//

#define FT_FLOW_NONE		0x0000
#define FT_FLOW_RTS_CTS		0x0100
#define FT_FLOW_DTR_DSR		0x0200
#define FT_FLOW_XON_XOFF	0x0400

//
// Purge rx and tx buffers
//
#define FT_PURGE_RX			1
#define FT_PURGE_TX			2

//
// Events
//

typedef void (*PFT_EVENT_HANDLER)(uint32_t,uint32_t);

#define FT_EVENT_RXunsigned char			1
#define FT_EVENT_MODEM_STATUS	2
#define FT_EVENT_LINE_STATUS	4

//
// Timeouts
//

#define FT_DEFAULT_RX_TIMEOUT	300
#define FT_DEFAULT_TX_TIMEOUT	300

//
// Device types
//

typedef uint32_t	FT_DEVICE;

enum {
	FT_DEVICE_BM,
	FT_DEVICE_AM,
	FT_DEVICE_100AX,
	FT_DEVICE_UNKNOWN,
	FT_DEVICE_2232C,
	FT_DEVICE_232R,
	FT_DEVICE_2232H,
	FT_DEVICE_4232H,
	FT_DEVICE_232H,
	FT_DEVICE_X_SERIES,
	FT_DEVICE_4222H_0,
	FT_DEVICE_4222H_1_2,
	FT_DEVICE_4222H_3,
    FT_DEVICE_4222_PROG,
};

//
// Bit Modes
//

#define FT_BITMODE_RESET					0x00
#define FT_BITMODE_ASYNC_BITBANG			0x01
#define FT_BITMODE_MPSSE					0x02
#define FT_BITMODE_SYNC_BITBANG				0x04
#define FT_BITMODE_MCU_HOST					0x08
#define FT_BITMODE_FAST_SERIAL				0x10
#define FT_BITMODE_CBUS_BITBANG				0x20
#define FT_BITMODE_SYNC_FIFO				0x40

//
// FT232R CBUS Options EEPROM values
//

#define FT_232R_CBUS_TXDEN					0x00	//	Tx Data Enable
#define FT_232R_CBUS_PWRON					0x01	//	Power On
#define FT_232R_CBUS_RXLED					0x02	//	Rx LED
#define FT_232R_CBUS_TXLED					0x03	//	Tx LED
#define FT_232R_CBUS_TXRXLED				0x04	//	Tx and Rx LED
#define FT_232R_CBUS_SLEEP					0x05	//	Sleep
#define FT_232R_CBUS_CLK48					0x06	//	48MHz clock
#define FT_232R_CBUS_CLK24					0x07	//	24MHz clock
#define FT_232R_CBUS_CLK12					0x08	//	12MHz clock
#define FT_232R_CBUS_CLK6					0x09	//	6MHz clock
#define FT_232R_CBUS_IOMODE					0x0A	//	IO Mode for CBUS bit-bang
#define FT_232R_CBUS_BITBANG_WR				0x0B	//	Bit-bang write strobe
#define FT_232R_CBUS_BITBANG_RD				0x0C	//	Bit-bang read strobe

//
// FT232H CBUS Options EEPROM values
//

#define FT_232H_CBUS_TRISTATE				0x00	//	Tristate
#define FT_232H_CBUS_TXLED					0x01	//	Tx LED
#define FT_232H_CBUS_RXLED					0x02	//	Rx LED
#define FT_232H_CBUS_TXRXLED				0x03	//	Tx and Rx LED
#define FT_232H_CBUS_PWREN					0x04	//	Power Enable
#define FT_232H_CBUS_SLEEP					0x05	//	Sleep
#define FT_232H_CBUS_DRIVE_0				0x06	//	Drive pin to logic 0
#define FT_232H_CBUS_DRIVE_1				0x07	//	Drive pin to logic 1
#define FT_232H_CBUS_IOMODE					0x08	//	IO Mode for CBUS bit-bang
#define FT_232H_CBUS_TXDEN					0x09	//	Tx Data Enable
#define FT_232H_CBUS_CLK30					0x0A	//	30MHz clock
#define FT_232H_CBUS_CLK15					0x0B	//	15MHz clock
#define FT_232H_CBUS_CLK7_5					0x0C	//	7.5MHz clock

//
// FT X Series CBUS Options EEPROM values
//

#define FT_X_SERIES_CBUS_TRISTATE			0x00	//	Tristate
#define FT_X_SERIES_CBUS_TXLED				0x01	//	Tx LED
#define FT_X_SERIES_CBUS_RXLED				0x02	//	Rx LED
#define FT_X_SERIES_CBUS_TXRXLED			0x03	//	Tx and Rx LED
#define FT_X_SERIES_CBUS_PWREN				0x04	//	Power Enable
#define FT_X_SERIES_CBUS_SLEEP				0x05	//	Sleep
#define FT_X_SERIES_CBUS_DRIVE_0			0x06	//	Drive pin to logic 0
#define FT_X_SERIES_CBUS_DRIVE_1			0x07	//	Drive pin to logic 1
#define FT_X_SERIES_CBUS_IOMODE				0x08	//	IO Mode for CBUS bit-bang
#define FT_X_SERIES_CBUS_TXDEN				0x09	//	Tx Data Enable
#define FT_X_SERIES_CBUS_CLK24				0x0A	//	24MHz clock
#define FT_X_SERIES_CBUS_CLK12				0x0B	//	12MHz clock
#define FT_X_SERIES_CBUS_CLK6				0x0C	//	6MHz clock
#define FT_X_SERIES_CBUS_BCD_CHARGER		0x0D	//	Battery charger detected
#define FT_X_SERIES_CBUS_BCD_CHARGER_N		0x0E	//	Battery charger detected inverted
#define FT_X_SERIES_CBUS_I2C_TXE			0x0F	//	I2C Tx empty
#define FT_X_SERIES_CBUS_I2C_RXF			0x10	//	I2C Rx full
#define FT_X_SERIES_CBUS_VBUS_SENSE			0x11	//	Detect VBUS
#define FT_X_SERIES_CBUS_BITBANG_WR			0x12	//	Bit-bang write strobe
#define FT_X_SERIES_CBUS_BITBANG_RD			0x13	//	Bit-bang read strobe
#define FT_X_SERIES_CBUS_TIMESTAMP			0x14	//	Toggle output when a USB SOF token is received
#define FT_X_SERIES_CBUS_KEEP_AWAKE			0x15	//	


// Driver types
#define FT_DRIVER_TYPE_D2XX		0
#define FT_DRIVER_TYPE_VCP		1



#ifdef __cplusplus
extern "C" {
#endif


	FTD2XX_API
		FT_STATUS  FT_Open(
		int deviceNumber,
		FT_HANDLE *pHandle
		);

	FTD2XX_API
		FT_STATUS  FT_OpenEx(
		void* pArg1,
		uint32_t Flags,
		FT_HANDLE *pHandle
		);

	FTD2XX_API 
		FT_STATUS  FT_ListDevices(
		void* pArg1,
		void* pArg2,
		uint32_t Flags
		);

	FTD2XX_API
		FT_STATUS  FT_Close(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		FT_STATUS  FT_Read(
		FT_HANDLE ftHandle,
		void* lpBuffer,
		uint32_t dwBytesToRead,
		uint32_t* lpBytesReturned
		);

	FTD2XX_API 
		FT_STATUS  FT_Write(
		FT_HANDLE ftHandle,
		void* lpBuffer,
		uint32_t dwBytesToWrite,
		uint32_t* lpBytesWritten
		);

	FTD2XX_API 
		FT_STATUS  FT_IoCtl(
		FT_HANDLE ftHandle,
		uint32_t dwIoControlCode,
		void* lpInBuf,
		uint32_t nInBufSize,
		void* lpOutBuf,
		uint32_t nOutBufSize,
		uint32_t* lpBytesReturned,
		LPOVERLAPPED lpOverlapped
		);

	FTD2XX_API
		FT_STATUS  FT_SetBaudRate(
		FT_HANDLE ftHandle,
		uint32_t BaudRate
		);

	FTD2XX_API
		FT_STATUS  FT_SetDivisor(
		FT_HANDLE ftHandle,
		uint16_t Divisor
		);

	FTD2XX_API
		FT_STATUS  FT_SetDataCharacteristics(
		FT_HANDLE ftHandle,
		uint8_t WordLength,
		uint8_t StopBits,
		uint8_t Parity
		);

	FTD2XX_API
		FT_STATUS  FT_SetFlowControl(
		FT_HANDLE ftHandle,
		uint16_t FlowControl,
		uint8_t XonChar,
		uint8_t XoffChar
		);

	FTD2XX_API
		FT_STATUS  FT_ResetDevice(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		FT_STATUS  FT_SetDtr(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		FT_STATUS  FT_ClrDtr(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		FT_STATUS  FT_SetRts(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		FT_STATUS  FT_ClrRts(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		FT_STATUS  FT_GetModemStatus(
		FT_HANDLE ftHandle,
		uint32_t *pModemStatus
		);

	FTD2XX_API
		FT_STATUS  FT_SetChars(
		FT_HANDLE ftHandle,
		uint8_t EventChar,
		uint8_t EventCharEnabled,
		uint8_t ErrorChar,
		uint8_t ErrorCharEnabled
		);

	FTD2XX_API
		FT_STATUS  FT_Purge(
		FT_HANDLE ftHandle,
		uint32_t Mask
		);

	FTD2XX_API
		FT_STATUS  FT_SetTimeouts(
		FT_HANDLE ftHandle,
		uint32_t ReadTimeout,
		uint32_t WriteTimeout
		);

	FTD2XX_API
		FT_STATUS  FT_GetQueueStatus(
		FT_HANDLE ftHandle,
		uint32_t *dwRxBytes
		);

	FTD2XX_API
		FT_STATUS  FT_SetEventNotification(
		FT_HANDLE ftHandle,
		uint32_t Mask,
		void* Param
		);

	FTD2XX_API
		FT_STATUS  FT_GetStatus(
		FT_HANDLE ftHandle,
		uint32_t *dwRxBytes,
		uint32_t *dwTxBytes,
		uint32_t *dwEventDWord
		);

	FTD2XX_API
		FT_STATUS  FT_SetBreakOn(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		FT_STATUS  FT_SetBreakOff(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		FT_STATUS  FT_SetWaitMask(
		FT_HANDLE ftHandle,
		uint32_t Mask
		);

	FTD2XX_API
		FT_STATUS  FT_WaitOnMask(
		FT_HANDLE ftHandle,
		uint32_t *Mask
		);

	FTD2XX_API
		FT_STATUS  FT_GetEventStatus(
		FT_HANDLE ftHandle,
		uint32_t *dwEventDWord
		);

	FTD2XX_API
		FT_STATUS  FT_ReadEE(
		FT_HANDLE ftHandle,
		uint32_t dwWordOffset,
		uint16_t* lpwValue
		);

	FTD2XX_API
		FT_STATUS  FT_WriteEE(
		FT_HANDLE ftHandle,
		uint32_t dwWordOffset,
		uint16_t wValue
		);

	FTD2XX_API
		FT_STATUS  FT_EraseEE(
		FT_HANDLE ftHandle
		);

	//
	// structure to hold program data for FT_EE_Program, FT_EE_ProgramEx, FT_EE_Read 
	// and FT_EE_ReadEx functions
	//
	typedef struct ft_program_data {

		uint32_t Signature1;			// Header - must be 0x00000000 
		uint32_t Signature2;			// Header - must be 0xffffffff
		uint32_t Version;				// Header - FT_PROGRAM_DATA version
		//			0 = original
		//			1 = FT2232 extensions
		//			2 = FT232R extensions
		//			3 = FT2232H extensions
		//			4 = FT4232H extensions
		//			5 = FT232H extensions

		uint16_t VendorId;				// 0x0403
		uint16_t ProductId;				// 0x6001
		char *Manufacturer;			// "FTDI"
		char *ManufacturerId;		// "FT"
		char *Description;			// "USB HS Serial Converter"
		char *SerialNumber;			// "FT000001" if fixed, or NULL
		uint16_t MaxPower;				// 0 < MaxPower <= 500
		uint16_t PnP;					// 0 = disabled, 1 = enabled
		uint16_t SelfPowered;			// 0 = bus powered, 1 = self powered
		uint16_t RemoteWakeup;			// 0 = not capable, 1 = capable
		//
		// Rev4 (FT232B) extensions
		//
		uint8_t Rev4;					// non-zero if Rev4 chip, zero otherwise
		uint8_t IsoIn;				// non-zero if in endpoint is isochronous
		uint8_t IsoOut;				// non-zero if out endpoint is isochronous
		uint8_t PullDownEnable;		// non-zero if pull down enabled
		uint8_t SerNumEnable;			// non-zero if serial number to be used
		uint8_t USBVersionEnable;		// non-zero if chip uses USBVersion
		uint16_t USBVersion;			// BCD (0x0200 => USB2)
		//
		// Rev 5 (FT2232) extensions
		//
		uint8_t Rev5;					// non-zero if Rev5 chip, zero otherwise
		uint8_t IsoInA;				// non-zero if in endpoint is isochronous
		uint8_t IsoInB;				// non-zero if in endpoint is isochronous
		uint8_t IsoOutA;				// non-zero if out endpoint is isochronous
		uint8_t IsoOutB;				// non-zero if out endpoint is isochronous
		uint8_t PullDownEnable5;		// non-zero if pull down enabled
		uint8_t SerNumEnable5;		// non-zero if serial number to be used
		uint8_t USBVersionEnable5;	// non-zero if chip uses USBVersion
		uint16_t USBVersion5;			// BCD (0x0200 => USB2)
		uint8_t AIsHighCurrent;		// non-zero if interface is high current
		uint8_t BIsHighCurrent;		// non-zero if interface is high current
		uint8_t IFAIsFifo;			// non-zero if interface is 245 FIFO
		uint8_t IFAIsFifoTar;			// non-zero if interface is 245 FIFO CPU target
		uint8_t IFAIsFastSer;			// non-zero if interface is Fast serial
		uint8_t AIsVCP;				// non-zero if interface is to use VCP drivers
		uint8_t IFBIsFifo;			// non-zero if interface is 245 FIFO
		uint8_t IFBIsFifoTar;			// non-zero if interface is 245 FIFO CPU target
		uint8_t IFBIsFastSer;			// non-zero if interface is Fast serial
		uint8_t BIsVCP;				// non-zero if interface is to use VCP drivers
		//
		// Rev 6 (FT232R) extensions
		//
		uint8_t UseExtOsc;			// Use External Oscillator
		uint8_t HighDriveIOs;			// High Drive I/Os
		uint8_t EndpointSize;			// Endpoint size
		uint8_t PullDownEnableR;		// non-zero if pull down enabled
		uint8_t SerNumEnableR;		// non-zero if serial number to be used
		uint8_t InvertTXD;			// non-zero if invert TXD
		uint8_t InvertRXD;			// non-zero if invert RXD
		uint8_t InvertRTS;			// non-zero if invert RTS
		uint8_t InvertCTS;			// non-zero if invert CTS
		uint8_t InvertDTR;			// non-zero if invert DTR
		uint8_t InvertDSR;			// non-zero if invert DSR
		uint8_t InvertDCD;			// non-zero if invert DCD
		uint8_t InvertRI;				// non-zero if invert RI
		uint8_t Cbus0;				// Cbus Mux control
		uint8_t Cbus1;				// Cbus Mux control
		uint8_t Cbus2;				// Cbus Mux control
		uint8_t Cbus3;				// Cbus Mux control
		uint8_t Cbus4;				// Cbus Mux control
		uint8_t RIsD2XX;				// non-zero if using D2XX driver
		//
		// Rev 7 (FT2232H) Extensions
		//
		uint8_t PullDownEnable7;		// non-zero if pull down enabled
		uint8_t SerNumEnable7;		// non-zero if serial number to be used
		uint8_t ALSlowSlew;			// non-zero if AL pins have slow slew
		uint8_t ALSchmittInput;		// non-zero if AL pins are Schmitt input
		uint8_t ALDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t AHSlowSlew;			// non-zero if AH pins have slow slew
		uint8_t AHSchmittInput;		// non-zero if AH pins are Schmitt input
		uint8_t AHDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t BLSlowSlew;			// non-zero if BL pins have slow slew
		uint8_t BLSchmittInput;		// non-zero if BL pins are Schmitt input
		uint8_t BLDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t BHSlowSlew;			// non-zero if BH pins have slow slew
		uint8_t BHSchmittInput;		// non-zero if BH pins are Schmitt input
		uint8_t BHDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t IFAIsFifo7;			// non-zero if interface is 245 FIFO
		uint8_t IFAIsFifoTar7;		// non-zero if interface is 245 FIFO CPU target
		uint8_t IFAIsFastSer7;		// non-zero if interface is Fast serial
		uint8_t AIsVCP7;				// non-zero if interface is to use VCP drivers
		uint8_t IFBIsFifo7;			// non-zero if interface is 245 FIFO
		uint8_t IFBIsFifoTar7;		// non-zero if interface is 245 FIFO CPU target
		uint8_t IFBIsFastSer7;		// non-zero if interface is Fast serial
		uint8_t BIsVCP7;				// non-zero if interface is to use VCP drivers
		uint8_t PowerSaveEnable;		// non-zero if using BCBUS7 to save power for self-powered designs
		//
		// Rev 8 (FT4232H) Extensions
		//
		uint8_t PullDownEnable8;		// non-zero if pull down enabled
		uint8_t SerNumEnable8;		// non-zero if serial number to be used
		uint8_t ASlowSlew;			// non-zero if A pins have slow slew
		uint8_t ASchmittInput;		// non-zero if A pins are Schmitt input
		uint8_t ADriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t BSlowSlew;			// non-zero if B pins have slow slew
		uint8_t BSchmittInput;		// non-zero if B pins are Schmitt input
		uint8_t BDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t CSlowSlew;			// non-zero if C pins have slow slew
		uint8_t CSchmittInput;		// non-zero if C pins are Schmitt input
		uint8_t CDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t DSlowSlew;			// non-zero if D pins have slow slew
		uint8_t DSchmittInput;		// non-zero if D pins are Schmitt input
		uint8_t DDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t ARIIsTXDEN;			// non-zero if port A uses RI as RS485 TXDEN
		uint8_t BRIIsTXDEN;			// non-zero if port B uses RI as RS485 TXDEN
		uint8_t CRIIsTXDEN;			// non-zero if port C uses RI as RS485 TXDEN
		uint8_t DRIIsTXDEN;			// non-zero if port D uses RI as RS485 TXDEN
		uint8_t AIsVCP8;				// non-zero if interface is to use VCP drivers
		uint8_t BIsVCP8;				// non-zero if interface is to use VCP drivers
		uint8_t CIsVCP8;				// non-zero if interface is to use VCP drivers
		uint8_t DIsVCP8;				// non-zero if interface is to use VCP drivers
		//
		// Rev 9 (FT232H) Extensions
		//
		uint8_t PullDownEnableH;		// non-zero if pull down enabled
		uint8_t SerNumEnableH;		// non-zero if serial number to be used
		uint8_t ACSlowSlewH;			// non-zero if AC pins have slow slew
		uint8_t ACSchmittInputH;		// non-zero if AC pins are Schmitt input
		uint8_t ACDriveCurrentH;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t ADSlowSlewH;			// non-zero if AD pins have slow slew
		uint8_t ADSchmittInputH;		// non-zero if AD pins are Schmitt input
		uint8_t ADDriveCurrentH;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t Cbus0H;				// Cbus Mux control
		uint8_t Cbus1H;				// Cbus Mux control
		uint8_t Cbus2H;				// Cbus Mux control
		uint8_t Cbus3H;				// Cbus Mux control
		uint8_t Cbus4H;				// Cbus Mux control
		uint8_t Cbus5H;				// Cbus Mux control
		uint8_t Cbus6H;				// Cbus Mux control
		uint8_t Cbus7H;				// Cbus Mux control
		uint8_t Cbus8H;				// Cbus Mux control
		uint8_t Cbus9H;				// Cbus Mux control
		uint8_t IsFifoH;				// non-zero if interface is 245 FIFO
		uint8_t IsFifoTarH;			// non-zero if interface is 245 FIFO CPU target
		uint8_t IsFastSerH;			// non-zero if interface is Fast serial
		uint8_t IsFT1248H;			// non-zero if interface is FT1248
		uint8_t FT1248CpolH;			// FT1248 clock polarity - clock idle high (1) or clock idle low (0)
		uint8_t FT1248LsbH;			// FT1248 data is LSB (1) or MSB (0)
		uint8_t FT1248FlowControlH;	// FT1248 flow control enable
		uint8_t IsVCPH;				// non-zero if interface is to use VCP drivers
		uint8_t PowerSaveEnableH;		// non-zero if using ACBUS7 to save power for self-powered designs
		
	} FT_PROGRAM_DATA, *PFT_PROGRAM_DATA;

	FTD2XX_API
		FT_STATUS  FT_EE_Program(
		FT_HANDLE ftHandle,
		PFT_PROGRAM_DATA pData
		);

	FTD2XX_API
		FT_STATUS  FT_EE_ProgramEx(
		FT_HANDLE ftHandle,
		PFT_PROGRAM_DATA pData,
		char *Manufacturer,
		char *ManufacturerId,
		char *Description,
		char *SerialNumber
		);

	FTD2XX_API
		FT_STATUS  FT_EE_Read(
		FT_HANDLE ftHandle,
		PFT_PROGRAM_DATA pData
		);

	FTD2XX_API
		FT_STATUS  FT_EE_ReadEx(
		FT_HANDLE ftHandle,
		PFT_PROGRAM_DATA pData,
		char *Manufacturer,
		char *ManufacturerId,
		char *Description,
		char *SerialNumber
		);

	FTD2XX_API
		FT_STATUS  FT_EE_UASize(
		FT_HANDLE ftHandle,
		uint32_t* lpdwSize
		);

	FTD2XX_API
		FT_STATUS  FT_EE_UAWrite(
		FT_HANDLE ftHandle,
		int8_t* pucData,
		uint32_t dwDataLen
		);

	FTD2XX_API
		FT_STATUS  FT_EE_UARead(
		FT_HANDLE ftHandle,
		int8_t* pucData,
		uint32_t dwDataLen,
		uint32_t* lpdwBytesRead
		);


	typedef struct ft_eeprom_header {
		FT_DEVICE deviceType;		// FTxxxx device type to be programmed
		// Device descriptor options
		uint16_t VendorId;				// 0x0403
		uint16_t ProductId;				// 0x6001
		uint8_t SerNumEnable;			// non-zero if serial number to be used
		// Config descriptor options
		uint16_t MaxPower;				// 0 < MaxPower <= 500
		uint8_t SelfPowered;			// 0 = bus powered, 1 = self powered
		uint8_t RemoteWakeup;			// 0 = not capable, 1 = capable
		// Hardware options
		uint8_t PullDownEnable;		// non-zero if pull down in suspend enabled
	} FT_EEPROM_HEADER, *PFT_EEPROM_HEADER;


	// FT232B EEPROM structure for use with FT_EEPROM_Read and FT_EEPROM_Program
	typedef struct ft_eeprom_232b {
		// Common header
		FT_EEPROM_HEADER common;	// common elements for all device EEPROMs
	} FT_EEPROM_232B, *PFT_EEPROM_232B;


	// FT2232 EEPROM structure for use with FT_EEPROM_Read and FT_EEPROM_Program
	typedef struct ft_eeprom_2232 {
		// Common header
		FT_EEPROM_HEADER common;	// common elements for all device EEPROMs
		// Drive options
		uint8_t AIsHighCurrent;		// non-zero if interface is high current
		uint8_t BIsHighCurrent;		// non-zero if interface is high current
		// Hardware options
		uint8_t AIsFifo;				// non-zero if interface is 245 FIFO
		uint8_t AIsFifoTar;			// non-zero if interface is 245 FIFO CPU target
		uint8_t AIsFastSer;			// non-zero if interface is Fast serial
		uint8_t BIsFifo;				// non-zero if interface is 245 FIFO
		uint8_t BIsFifoTar;			// non-zero if interface is 245 FIFO CPU target
		uint8_t BIsFastSer;			// non-zero if interface is Fast serial
		// Driver option
		uint8_t ADriverType;			// 
		uint8_t BDriverType;			// 
	} FT_EEPROM_2232, *PFT_EEPROM_2232;


	// FT232R EEPROM structure for use with FT_EEPROM_Read and FT_EEPROM_Program
	typedef struct ft_eeprom_232r {
		// Common header
		FT_EEPROM_HEADER common;	// common elements for all device EEPROMs
		// Drive options
		uint8_t IsHighCurrent;		// non-zero if interface is high current
		// Hardware options
		uint8_t UseExtOsc;			// Use External Oscillator
		uint8_t InvertTXD;			// non-zero if invert TXD
		uint8_t InvertRXD;			// non-zero if invert RXD
		uint8_t InvertRTS;			// non-zero if invert RTS
		uint8_t InvertCTS;			// non-zero if invert CTS
		uint8_t InvertDTR;			// non-zero if invert DTR
		uint8_t InvertDSR;			// non-zero if invert DSR
		uint8_t InvertDCD;			// non-zero if invert DCD
		uint8_t InvertRI;				// non-zero if invert RI
		uint8_t Cbus0;				// Cbus Mux control
		uint8_t Cbus1;				// Cbus Mux control
		uint8_t Cbus2;				// Cbus Mux control
		uint8_t Cbus3;				// Cbus Mux control
		uint8_t Cbus4;				// Cbus Mux control
		// Driver option
		uint8_t DriverType;			// 
	} FT_EEPROM_232R, *PFT_EEPROM_232R;


	// FT2232H EEPROM structure for use with FT_EEPROM_Read and FT_EEPROM_Program
	typedef struct ft_eeprom_2232h {
		// Common header
		FT_EEPROM_HEADER common;	// common elements for all device EEPROMs
		// Drive options
		uint8_t ALSlowSlew;			// non-zero if AL pins have slow slew
		uint8_t ALSchmittInput;		// non-zero if AL pins are Schmitt input
		uint8_t ALDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t AHSlowSlew;			// non-zero if AH pins have slow slew
		uint8_t AHSchmittInput;		// non-zero if AH pins are Schmitt input
		uint8_t AHDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t BLSlowSlew;			// non-zero if BL pins have slow slew
		uint8_t BLSchmittInput;		// non-zero if BL pins are Schmitt input
		uint8_t BLDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t BHSlowSlew;			// non-zero if BH pins have slow slew
		uint8_t BHSchmittInput;		// non-zero if BH pins are Schmitt input
		uint8_t BHDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		// Hardware options
		uint8_t AIsFifo;				// non-zero if interface is 245 FIFO
		uint8_t AIsFifoTar;			// non-zero if interface is 245 FIFO CPU target
		uint8_t AIsFastSer;			// non-zero if interface is Fast serial
		uint8_t BIsFifo;				// non-zero if interface is 245 FIFO
		uint8_t BIsFifoTar;			// non-zero if interface is 245 FIFO CPU target
		uint8_t BIsFastSer;			// non-zero if interface is Fast serial
		uint8_t PowerSaveEnable;		// non-zero if using BCBUS7 to save power for self-powered designs
		// Driver option
		uint8_t ADriverType;			// 
		uint8_t BDriverType;			// 
	} FT_EEPROM_2232H, *PFT_EEPROM_2232H;


	// FT4232H EEPROM structure for use with FT_EEPROM_Read and FT_EEPROM_Program
	typedef struct ft_eeprom_4232h {
		// Common header
		FT_EEPROM_HEADER common;	// common elements for all device EEPROMs
		// Drive options
		uint8_t ASlowSlew;			// non-zero if A pins have slow slew
		uint8_t ASchmittInput;		// non-zero if A pins are Schmitt input
		uint8_t ADriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t BSlowSlew;			// non-zero if B pins have slow slew
		uint8_t BSchmittInput;		// non-zero if B pins are Schmitt input
		uint8_t BDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t CSlowSlew;			// non-zero if C pins have slow slew
		uint8_t CSchmittInput;		// non-zero if C pins are Schmitt input
		uint8_t CDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t DSlowSlew;			// non-zero if D pins have slow slew
		uint8_t DSchmittInput;		// non-zero if D pins are Schmitt input
		uint8_t DDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		// Hardware options
		uint8_t ARIIsTXDEN;			// non-zero if port A uses RI as RS485 TXDEN
		uint8_t BRIIsTXDEN;			// non-zero if port B uses RI as RS485 TXDEN
		uint8_t CRIIsTXDEN;			// non-zero if port C uses RI as RS485 TXDEN
		uint8_t DRIIsTXDEN;			// non-zero if port D uses RI as RS485 TXDEN
		// Driver option
		uint8_t ADriverType;			// 
		uint8_t BDriverType;			// 
		uint8_t CDriverType;			// 
		uint8_t DDriverType;			// 
	} FT_EEPROM_4232H, *PFT_EEPROM_4232H;


	// FT232H EEPROM structure for use with FT_EEPROM_Read and FT_EEPROM_Program
	typedef struct ft_eeprom_232h {
		// Common header
		FT_EEPROM_HEADER common;	// common elements for all device EEPROMs
		// Drive options
		uint8_t ACSlowSlew;			// non-zero if AC bus pins have slow slew
		uint8_t ACSchmittInput;		// non-zero if AC bus pins are Schmitt input
		uint8_t ACDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t ADSlowSlew;			// non-zero if AD bus pins have slow slew
		uint8_t ADSchmittInput;		// non-zero if AD bus pins are Schmitt input
		uint8_t ADDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		// CBUS options
		uint8_t Cbus0;				// Cbus Mux control
		uint8_t Cbus1;				// Cbus Mux control
		uint8_t Cbus2;				// Cbus Mux control
		uint8_t Cbus3;				// Cbus Mux control
		uint8_t Cbus4;				// Cbus Mux control
		uint8_t Cbus5;				// Cbus Mux control
		uint8_t Cbus6;				// Cbus Mux control
		uint8_t Cbus7;				// Cbus Mux control
		uint8_t Cbus8;				// Cbus Mux control
		uint8_t Cbus9;				// Cbus Mux control
		// FT1248 options
		uint8_t FT1248Cpol;			// FT1248 clock polarity - clock idle high (1) or clock idle low (0)
		uint8_t FT1248Lsb;			// FT1248 data is LSB (1) or MSB (0)
		uint8_t FT1248FlowControl;	// FT1248 flow control enable
		// Hardware options
		uint8_t IsFifo;				// non-zero if interface is 245 FIFO
		uint8_t IsFifoTar;			// non-zero if interface is 245 FIFO CPU target
		uint8_t IsFastSer;			// non-zero if interface is Fast serial
		uint8_t IsFT1248	;			// non-zero if interface is FT1248
		uint8_t PowerSaveEnable;		// 
		// Driver option
		uint8_t DriverType;			// 
	} FT_EEPROM_232H, *PFT_EEPROM_232H;


	// FT X Series EEPROM structure for use with FT_EEPROM_Read and FT_EEPROM_Program
	typedef struct ft_eeprom_x_series {
		// Common header
		FT_EEPROM_HEADER common;	// common elements for all device EEPROMs
		// Drive options
		uint8_t ACSlowSlew;			// non-zero if AC bus pins have slow slew
		uint8_t ACSchmittInput;		// non-zero if AC bus pins are Schmitt input
		uint8_t ACDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		uint8_t ADSlowSlew;			// non-zero if AD bus pins have slow slew
		uint8_t ADSchmittInput;		// non-zero if AD bus pins are Schmitt input
		uint8_t ADDriveCurrent;		// valid values are 4mA, 8mA, 12mA, 16mA
		// CBUS options
		uint8_t Cbus0;				// Cbus Mux control
		uint8_t Cbus1;				// Cbus Mux control
		uint8_t Cbus2;				// Cbus Mux control
		uint8_t Cbus3;				// Cbus Mux control
		uint8_t Cbus4;				// Cbus Mux control
		uint8_t Cbus5;				// Cbus Mux control
		uint8_t Cbus6;				// Cbus Mux control
		// UART signal options
		uint8_t InvertTXD;			// non-zero if invert TXD
		uint8_t InvertRXD;			// non-zero if invert RXD
		uint8_t InvertRTS;			// non-zero if invert RTS
		uint8_t InvertCTS;			// non-zero if invert CTS
		uint8_t InvertDTR;			// non-zero if invert DTR
		uint8_t InvertDSR;			// non-zero if invert DSR
		uint8_t InvertDCD;			// non-zero if invert DCD
		uint8_t InvertRI;				// non-zero if invert RI
		// Battery Charge Detect options
		uint8_t BCDEnable;			// Enable Battery Charger Detection
		uint8_t BCDForceCbusPWREN;	// asserts the power enable signal on CBUS when charging port detected
		uint8_t BCDDisableSleep;		// forces the device never to go into sleep mode
		// I2C options
		uint16_t I2CSlaveAddress;		// I2C slave device address
		uint32_t I2CDeviceId;			// I2C device ID
		uint8_t I2CDisableSchmitt;	// Disable I2C Schmitt trigger
		// FT1248 options
		uint8_t FT1248Cpol;			// FT1248 clock polarity - clock idle high (1) or clock idle low (0)
		uint8_t FT1248Lsb;			// FT1248 data is LSB (1) or MSB (0)
		uint8_t FT1248FlowControl;	// FT1248 flow control enable
		// Hardware options
		uint8_t RS485EchoSuppress;	// 
		uint8_t PowerSaveEnable;		// 
		// Driver option
		uint8_t DriverType;			// 
	} FT_EEPROM_X_SERIES, *PFT_EEPROM_X_SERIES;


	FTD2XX_API
		FT_STATUS  FT_EEPROM_Read(
		FT_HANDLE ftHandle,
		void *eepromData,
		uint32_t eepromDataSize,
		char *Manufacturer,
		char *ManufacturerId,
		char *Description,
		char *SerialNumber
		);


	FTD2XX_API
		FT_STATUS  FT_EEPROM_Program(
		FT_HANDLE ftHandle,
		void *eepromData,
		uint32_t eepromDataSize,
		char *Manufacturer,
		char *ManufacturerId,
		char *Description,
		char *SerialNumber
		);


	FTD2XX_API
		FT_STATUS  FT_SetLatencyTimer(
		FT_HANDLE ftHandle,
		uint8_t ucLatency
		);

	FTD2XX_API
		FT_STATUS  FT_GetLatencyTimer(
		FT_HANDLE ftHandle,
		int8_t* pucLatency
		);

	FTD2XX_API
		FT_STATUS  FT_SetBitMode(
		FT_HANDLE ftHandle,
		uint8_t ucMask,
		uint8_t ucEnable
		);

	FTD2XX_API
		FT_STATUS  FT_GetBitMode(
		FT_HANDLE ftHandle,
		int8_t* pucMode
		);

	FTD2XX_API
		FT_STATUS  FT_SetUSBParameters(
		FT_HANDLE ftHandle,
		uint32_t ulInTransferSize,
		uint32_t ulOutTransferSize
		);

	FTD2XX_API
		FT_STATUS  FT_SetDeadmanTimeout(
		FT_HANDLE ftHandle,
		uint32_t ulDeadmanTimeout
		);

#ifndef _WIN32
	// Extra functions for non-Windows platforms to compensate
	// for lack of .INF file to specify Vendor and Product IDs.

	FTD2XX_API
		FT_STATUS FT_SetVIDPID(
		uint32_t dwVID, 
		uint32_t dwPID
		);
			
	FTD2XX_API
		FT_STATUS FT_GetVIDPID(
		uint32_t * pdwVID, 
		uint32_t * pdwPID
		);

	FTD2XX_API
		FT_STATUS  FT_GetDeviceLocId(
		FT_HANDLE ftHandle,
		uint32_t* lpdwLocId
		);
#endif // _WIN32        

	FTD2XX_API
		FT_STATUS  FT_GetDeviceInfo(
		FT_HANDLE ftHandle,
		FT_DEVICE *lpftDevice,
		uint32_t* lpdwID,
		unsigned char* SerialNumber,
		unsigned char* Description,
		void* Dummy
		);

	FTD2XX_API
		FT_STATUS  FT_StopInTask(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		FT_STATUS  FT_RestartInTask(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		FT_STATUS  FT_SetResetPipeRetryCount(
		FT_HANDLE ftHandle,
		uint32_t dwCount
		);

	FTD2XX_API
		FT_STATUS  FT_ResetPort(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		FT_STATUS  FT_CyclePort(
		FT_HANDLE ftHandle
		);


	//
	// Win32-type functions
	//

	FTD2XX_API
		FT_HANDLE  FT_W32_CreateFile(
		const char*					lpszName,
		uint32_t					dwAccess,
		uint32_t					dwShareMode,
		LPSECURITY_ATTRIBUTES	lpSecurityAttributes,
		uint32_t					dwCreate,
		uint32_t					dwAttrsAndFlags,
		void*					hTemplate
		);

	FTD2XX_API
		BOOL  FT_W32_CloseHandle(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		BOOL  FT_W32_ReadFile(
		FT_HANDLE ftHandle,
		void* lpBuffer,
		uint32_t nBufferSize,
		uint32_t* lpBytesReturned,
		LPOVERLAPPED lpOverlapped
		);

	FTD2XX_API
		BOOL  FT_W32_WriteFile(
		FT_HANDLE ftHandle,
		void* lpBuffer,
		uint32_t nBufferSize,
		uint32_t* lpBytesWritten,
		LPOVERLAPPED lpOverlapped
		);

	FTD2XX_API
		uint32_t  FT_W32_GetLastError(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		BOOL  FT_W32_GetOverlappedResult(
		FT_HANDLE ftHandle,
		LPOVERLAPPED lpOverlapped,
		uint32_t* lpdwBytesTransferred,
		BOOL bWait
		);

	FTD2XX_API
		BOOL  FT_W32_CancelIo(
		FT_HANDLE ftHandle
		);


	//
	// Win32 COMM API type functions
	//
	typedef struct _FTCOMSTAT {
		uint32_t fCtsHold : 1;
		uint32_t fDsrHold : 1;
		uint32_t fRlsdHold : 1;
		uint32_t fXoffHold : 1;
		uint32_t fXoffSent : 1;
		uint32_t fEof : 1;
		uint32_t fTxim : 1;
		uint32_t fReserved : 25;
		uint32_t cbInQue;
		uint32_t cbOutQue;
	} FTCOMSTAT, *LPFTCOMSTAT;

	typedef struct _FTDCB {
		uint32_t DCBlength;			/* sizeof(FTDCB)						*/
		uint32_t BaudRate;				/* Baudrate at which running			*/
		uint32_t fBinary: 1;			/* Binary Mode (skip EOF check)			*/
		uint32_t fParity: 1;			/* Enable parity checking				*/
		uint32_t fOutxCtsFlow:1;		/* CTS handshaking on output			*/
		uint32_t fOutxDsrFlow:1;		/* DSR handshaking on output			*/
		uint32_t fDtrControl:2;		/* DTR Flow control						*/
		uint32_t fDsrSensitivity:1;	/* DSR Sensitivity						*/
		uint32_t fTXContinueOnXoff: 1;	/* Continue TX when Xoff sent			*/
		uint32_t fOutX: 1;				/* Enable output X-ON/X-OFF				*/
		uint32_t fInX: 1;				/* Enable input X-ON/X-OFF				*/
		uint32_t fErrorChar: 1;		/* Enable Err Replacement				*/
		uint32_t fNull: 1;				/* Enable Null stripping				*/
		uint32_t fRtsControl:2;		/* Rts Flow control						*/
		uint32_t fAbortOnError:1;		/* Abort all reads and writes on Error	*/
		uint32_t fDummy2:17;			/* Reserved								*/
		uint16_t wReserved;				/* Not currently used					*/
		uint16_t XonLim;				/* Transmit X-ON threshold				*/
		uint16_t XoffLim;				/* Transmit X-OFF threshold				*/
		uint8_t ByteSize;				/* Number of bits/byte, 4-8				*/
		uint8_t Parity;				/* 0-4=None,Odd,Even,Mark,Space			*/
		uint8_t StopBits;				/* 0,1,2 = 1, 1.5, 2					*/
		char XonChar;				/* Tx and Rx X-ON character				*/
		char XoffChar;				/* Tx and Rx X-OFF character			*/
		char ErrorChar;				/* Error replacement char				*/
		char EofChar;				/* End of Input character				*/
		char EvtChar;				/* Received Event character				*/
		uint16_t wReserved1;			/* Fill for now.						*/
	} FTDCB, *LPFTDCB;

	typedef struct _FTTIMEOUTS {
		uint32_t ReadIntervalTimeout;			/* Maximum time between read chars.	*/
		uint32_t ReadTotalTimeoutMultiplier;	/* Multiplier of characters.		*/
		uint32_t ReadTotalTimeoutConstant;		/* Constant in milliseconds.		*/
		uint32_t WriteTotalTimeoutMultiplier;	/* Multiplier of characters.		*/
		uint32_t WriteTotalTimeoutConstant;	/* Constant in milliseconds.		*/
	} FTTIMEOUTS,*LPFTTIMEOUTS;


	FTD2XX_API
		BOOL  FT_W32_ClearCommBreak(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		BOOL  FT_W32_ClearCommError(
		FT_HANDLE ftHandle,
		uint32_t* lpdwErrors,
		LPFTCOMSTAT lpftComstat
		);

	FTD2XX_API
		BOOL  FT_W32_EscapeCommFunction(
		FT_HANDLE ftHandle,
		uint32_t dwFunc
		);

	FTD2XX_API
		BOOL  FT_W32_GetCommModemStatus(
		FT_HANDLE ftHandle,
		uint32_t* lpdwModemStatus
		);

	FTD2XX_API
		BOOL  FT_W32_GetCommState(
		FT_HANDLE ftHandle,
		LPFTDCB lpftDcb
		);

	FTD2XX_API
		BOOL  FT_W32_GetCommTimeouts(
		FT_HANDLE ftHandle,
		FTTIMEOUTS *pTimeouts
		);

	FTD2XX_API
		BOOL  FT_W32_PurgeComm(
		FT_HANDLE ftHandle,
		uint32_t dwMask
		);

	FTD2XX_API
		BOOL  FT_W32_SetCommBreak(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		BOOL  FT_W32_SetCommMask(
		FT_HANDLE ftHandle,
		uint32_t ulEventMask
		);

	FTD2XX_API
		BOOL  FT_W32_GetCommMask(
		FT_HANDLE ftHandle,
		uint32_t* lpdwEventMask
		);

	FTD2XX_API
		BOOL  FT_W32_SetCommState(
		FT_HANDLE ftHandle,
		LPFTDCB lpftDcb
		);

	FTD2XX_API
		BOOL  FT_W32_SetCommTimeouts(
		FT_HANDLE ftHandle,
		FTTIMEOUTS *pTimeouts
		);

	FTD2XX_API
		BOOL  FT_W32_SetupComm(
		FT_HANDLE ftHandle,
		uint32_t dwReadBufferSize,
		uint32_t dwWriteBufferSize
		);

	FTD2XX_API
		BOOL  FT_W32_WaitCommEvent(
		FT_HANDLE ftHandle,
		uint32_t* pulEvent,
		LPOVERLAPPED lpOverlapped
		);


	//
	// Device information
	//

	typedef struct _ft_device_list_info_node {
		uint32_t Flags;
		uint32_t Type;
		uint32_t ID;
		uint32_t LocId;
		char SerialNumber[16];
		char Description[64];
		FT_HANDLE ftHandle;
	} FT_DEVICE_LIST_INFO_NODE;

	// Device information flags
	enum {
		FT_FLAGS_OPENED = 1,
		FT_FLAGS_HISPEED = 2
	};


	FTD2XX_API
		FT_STATUS  FT_CreateDeviceInfoList(
		uint32_t* lpdwNumDevs
		);

	FTD2XX_API
		FT_STATUS  FT_GetDeviceInfoList(
		FT_DEVICE_LIST_INFO_NODE *pDest,
		uint32_t* lpdwNumDevs
		);

	FTD2XX_API
		FT_STATUS  FT_GetDeviceInfoDetail(
		uint32_t dwIndex,
		uint32_t* lpdwFlags,
		uint32_t* lpdwType,
		uint32_t* lpdwID,
		uint32_t* lpdwLocId,
		void* lpSerialNumber,
		void* lpDescription,
		FT_HANDLE *pftHandle
		);


	//
	// Version information
	//

	FTD2XX_API
		FT_STATUS  FT_GetDriverVersion(
		FT_HANDLE ftHandle,
		uint32_t* lpdwVersion
		);

	FTD2XX_API
		FT_STATUS  FT_GetLibraryVersion(
		uint32_t* lpdwVersion
		);


	FTD2XX_API
		FT_STATUS  FT_Rescan(
		void
		);

	FTD2XX_API
		FT_STATUS  FT_Reload(
		uint16_t wVid,
		uint16_t wPid
		);

	FTD2XX_API
		FT_STATUS  FT_GetComPortNumber(
		FT_HANDLE ftHandle,
		int32_t*	lpdwComPortNumber
		);


	//
	// FT232H additional EEPROM functions
	//

	FTD2XX_API
		FT_STATUS  FT_EE_ReadConfig(
		FT_HANDLE ftHandle,
		uint8_t ucAddress,
		int8_t* pucValue
		);

	FTD2XX_API
		FT_STATUS  FT_EE_WriteConfig(
		FT_HANDLE ftHandle,
		uint8_t ucAddress,
		uint8_t ucValue
		);

	FTD2XX_API
		FT_STATUS  FT_EE_ReadECC(
		FT_HANDLE ftHandle,
		uint8_t ucOption,
		uint16_t* lpwValue
		);

	FTD2XX_API
		FT_STATUS  FT_GetQueueStatusEx(
		FT_HANDLE ftHandle,
		uint32_t *dwRxBytes
		);

	FTD2XX_API
		FT_STATUS  FT_ComPortIdle(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		FT_STATUS  FT_ComPortCancelIdle(
		FT_HANDLE ftHandle
		);

	FTD2XX_API
		FT_STATUS  FT_VendorCmdGet(
		FT_HANDLE ftHandle,
		uint8_t Request,
		uint8_t *Buf,
		uint16_t Len
		);

	FTD2XX_API
		FT_STATUS  FT_VendorCmdSet(
		FT_HANDLE ftHandle,
		uint8_t Request,
		uint8_t *Buf,
		uint16_t Len
		);

	FTD2XX_API
		FT_STATUS  FT_VendorCmdGetEx(
		FT_HANDLE ftHandle,
		uint16_t wValue,
		uint8_t *Buf,
		uint16_t Len
		);

	FTD2XX_API
		FT_STATUS  FT_VendorCmdSetEx(
		FT_HANDLE ftHandle,
		uint16_t wValue,
		uint8_t *Buf,
		uint16_t Len
		);

#ifdef __cplusplus
}
#endif


#endif	/* FTD2XX_H */

