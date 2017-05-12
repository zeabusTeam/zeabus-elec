#include <ros/ros.h>
#include <zeabus_elec_ros_peripheral_bridge/barometer.h>
#include <zeabus_elec_ros_peripheral_bridge/solenoid.h>
#include <zeabus_elec_ros_peripheral_bridge/senddata.h>
#include <zeabus_elec_ros_peripheral_bridge/receivedata.h>
#include <sys/syscall.h>
#include <fcntl.h>
#include <stdlib.h>
#include <ftdi_impl.hpp>

/* ===================================================
 * File-scope global variables
 * ===================================================
 */
static const char* cstPeripheralSerial[] =
{
	"PeripheralBridge-A",
	"PeripheralBridge-B",
	"PeripheralBridge-C",
	"PeripheralBridge-D"
};

static FT_HANDLE	xHandle[4];		/* Storage of device handles. FT4232 has 4 sub-modules */
static Zeabus_Elec::ftdi_impl *pxMsspA, *pxMsspB, *pxUartA, *pxUartB;

/* ===================================================
 * ROS service subroutines
 * ===================================================
 */

/* Set the solenoid switches. We have maximum 8 switches. The low-nibble 4 switches 
is controlled by MSSP_A and the other 4 switches is controlled by MSSP_B */
bool ZeabusElec_SetSolenoid(zeabus_elec_ros_peripheral_bridge::solenoid::Request  &req,
        zeabus_elec_ros_peripheral_bridge::solenoid::Response &res)
{
	uint8_t swNibble;
	
	/* Low nibble */
	swNibble = req.solenoidState & 0x0F;
	res.retStat = pxMsspA->SetHiGPIOData( swNibble );

	/* High nibble */
	swNibble = req.solenoidState >> 4;
	res.retStat |= pxMsspB->SetHiGPIOData( swNibble );
	if( res.retStat == FT_OK )
	{
		return( true );
	}
	else
	{
		return( false );
	}
}

/* Read barometer value. The barometer is connected by SPI protocol through MSSP_A */
bool ZeabusElec_GetPressure(zeabus_elec_ros_peripheral_bridge::barometer::Request  &req,
        zeabus_elec_ros_peripheral_bridge::barometer::Response &res)
{
	uint8_t aucVal[3]; /* Data buffer. The raw data are in 2 bytes as a stream */
	FT_STATUS xFTStatus;
	
	/* To read barometer value, we need to send a 16-bit dummy data first */
	xFTStatus = pxMsspA->Send( aucVal, 2 );
	if( xFTStatus != FT_OK )
	{
		res.barometerVal = 0;
		return( false );
	}
	
	/* Read the actual 2-byte raw data. The actual data is 10 bits plus some unused bit as:
		   0 0 0 D D D D D  D D D D D x x x
		where the first byte is the MSB. */
	xFTStatus = pxMsspA->Receive( aucVal, 2 );
	if( xFTStatus != FT_OK )
	{
		res.barometerVal = 0;
		return( false );
	}
	res.barometerVal = (uint16_t)( aucVal[ 0 ] ); /* High byte */
	res.barometerVal <<= 8;
	res.barometerVal |= (uint16_t)( aucVal[ 1 ] ); /* Low byte */
	res.barometerVal >>= 3;	/* Suppress 3 tailing zero bits */

	return( true );	/* All success */	
}

bool ZeabusElec_SendComm1(zeabus_elec_ros_peripheral_bridge::senddata::Request  &req,
        zeabus_elec_ros_peripheral_bridge::senddata::Response &res)
{
	uint8_t* pucBuffer;
	
	pucBuffer = (uint8_t*)malloc( req.len );
	for( int i = 0; i < req.len; i++ )
	{
		pucBuffer[i] = req.buffer[i];
	}
	res.totalWritten = pxUartA->Send( pucBuffer, req.len );
	free( pucBuffer );
	if( pxUartA->GetCurrentStatus() != FT_OK )
	{
		return( false );
	}
	else
	{
		return( true );
	}
}

bool ZeabusElec_SendComm2(zeabus_elec_ros_peripheral_bridge::senddata::Request  &req,
        zeabus_elec_ros_peripheral_bridge::senddata::Response &res)
{
	uint8_t* pucBuffer;
	
	pucBuffer = (uint8_t*)malloc( req.len );
	for( int i = 0; i < req.len; i++ )
	{
		pucBuffer[i] = req.buffer[i];
	}
	res.totalWritten = pxUartB->Send( pucBuffer, req.len );
	free( pucBuffer );
	if( pxUartB->GetCurrentStatus() != FT_OK )
	{
		return( false );
	}
	else
	{
		return( true );
	}
}

bool ZeabusElec_ReceiveComm1(zeabus_elec_ros_peripheral_bridge::receivedata::Request  &req,
        zeabus_elec_ros_peripheral_bridge::receivedata::Response &res)
{
	uint8_t* pucBuffer;
	
	pucBuffer = (uint8_t*)malloc( req.maxLen );
	if( pucBuffer == NULL )
	{
		res.totalReceived = 0;
		return( false );
	}
	res.totalReceived = pxUartA->Receive( pucBuffer, req.maxLen );
	for( int i = 0; i < res.totalReceived; i++ )
	{
		req.buffer[i] = pucBuffer[i];
	}
	free( pucBuffer );

	if( pxUartA->GetCurrentStatus() != FT_OK )
	{
		return( false );
	}
	else
	{
		return( true );
	}
}

bool ZeabusElec_ReceiveComm2(zeabus_elec_ros_peripheral_bridge::receivedata::Request  &req,
        zeabus_elec_ros_peripheral_bridge::receivedata::Response &res)
{
	uint8_t* pucBuffer;
	
	pucBuffer = (uint8_t*)malloc( req.maxLen );
	if( pucBuffer == NULL )
	{
		res.totalReceived = 0;
		return( false );
	}
	res.totalReceived = pxUartB->Receive( pucBuffer, req.maxLen );
	for( int i = 0; i < res.totalReceived; i++ )
	{
		req.buffer[i] = pucBuffer[i];
	}
	free( pucBuffer );

	if( pxUartB->GetCurrentStatus() != FT_OK )
	{
		return( false );
	}
	else
	{
		return( true );
	}
}

/* ===================================================
 * main
 * ===================================================
 */
int main( int argc, char** argv )
{
	long iSysCallStat;
	FT_STATUS xFTStatus;
	FT_DEVICE_LIST_INFO_NODE *xDevInfo;
	uint32_t ulTotalDevs, ulNumFoundDev;

	/* Initialize ROS functionalities */	
	ros::init(argc, argv, "Zeabus_Elec_Peripheral_bridge");
 	ros::NodeHandle nh("~");
	
	/* Remove all kernel modules that occupied the FTDI chips */
	iSysCallStat = syscall( SYS_delete_module, "ftdi_sio", O_NONBLOCK | O_TRUNC );
	if( ( iSysCallStat != 0 ) && ( errno != ENOENT ) )
	{
		ROS_FATAL( "Unable to disable system default FTDI driver" );
		return( -1 );
	}

	/*=================================================================================
	  Discover the Power Distributor and also open handles for it.
	  =================================================================================*/

	/* Create and retrieve the list of the FTDI device information. */
	xFTStatus = FT_CreateDeviceInfoList( &ulTotalDevs );
	if ( ( xFTStatus != FT_OK ) || ( ulTotalDevs == 0 ) )
	{
		/* Unable to find any devices */
		ROS_FATAL( "Unable to find any FTDI chips" );
		return( -2 );
	}

	/* We found some FTDI chips, then get the information of the devices */
    xDevInfo = (FT_DEVICE_LIST_INFO_NODE*)malloc( sizeof( FT_DEVICE_LIST_INFO_NODE) * ulTotalDevs );
	xFTStatus = FT_GetDeviceInfoList( xDevInfo, &ulTotalDevs );
	if ( xFTStatus != FT_OK )
	{
		/* Unable to find any devices */
		free( xDevInfo );	/* Free-up the previous allocation */
		ROS_FATAL( "Unable to retrieve any FTDI chip information" );
		return( -3 );
	}
	free( xDevInfo );	/* Free-up the previous allocation */

	/* Attempt to open the Peripheral-Bridge modules described by their serial number */
	/* The module utilizes FT4232H consisting of 4 module instances inside */
	for( int i = 0; i < 4; i++ )
	{
		xFTStatus = FT_OpenEx( (void*)( cstPeripheralSerial[ i ] ), FT_OPEN_BY_SERIAL_NUMBER, &( xHandle[ i ] ) );
		if (xFTStatus != FT_OK)
		{
			/* Fail - unable to open the device with the serial number "PowerDist" */
			ROS_FATAL( "Unable to open a Peripheral Bridge module instance" );
			return( -4 );
		}
	}

	/* All FTDI initialization completed successfully. The xHandle is valid */
	
	/* Create the device manager classes to implement chip functions */
	pxMsspA = new Zeabus_Elec::ftdi_spi_cpol1_cha0_msb_impl( FT_DEVICE_4232H, xHandle[0] );
	if( pxMsspA->GetCurrentStatus() != FT_OK )
	{
		/* Fail - unable to initialize Power Distribution module */
		delete pxMsspA;
		ROS_FATAL( "Unable to initialize Power Distribution module A" );
		return( -5 );
		
	}
	pxMsspA->SetGPIODirection( 0xFFFF );	/* All bits are output */

	pxMsspB = new Zeabus_Elec::ftdi_spi_cpol1_cha0_msb_impl( FT_DEVICE_4232H, xHandle[1] );
	if( pxMsspB->GetCurrentStatus() != FT_OK )
	{
		/* Fail - unable to initialize Power Distribution module */
		delete pxMsspA;
		delete pxMsspB;
		ROS_FATAL( "Unable to initialize Power Distribution module B" );
		return( -6 );
		
	}
	pxMsspB->SetGPIODirection( 0xFFFF );	/* All bits are output */
	
	pxUartA = new Zeabus_Elec::ftdi_uart_impl( FT_DEVICE_4232H, xHandle[2] );
	if( pxUartA->GetCurrentStatus() != FT_OK )
	{
		/* Fail - unable to initialize Power Distribution module */
		delete pxMsspA;
		delete pxMsspB;
		delete pxUartA;
		ROS_FATAL( "Unable to initialize Power Distribution module C" );
		return( -7 );
		
	}

	pxUartB = new Zeabus_Elec::ftdi_uart_impl( FT_DEVICE_4232H, xHandle[3] );
	if( pxUartB->GetCurrentStatus() != FT_OK )
	{
		/* Fail - unable to initialize Power Distribution module */
		delete pxMsspA;
		delete pxMsspB;
		delete pxUartA;
		delete pxUartB;
		ROS_FATAL( "Unable to initialize Power Distribution module D" );
		return( -8 );
		
	}

	/*=================================================================================
	  Now the FTDI chip is opened and hooked. We can continue ROS registration process 
	  =================================================================================*/
	
	/* Register ROS service node for power-distributor switch controller */
	ros::ServiceServer service_zeabus_elec_solenoid_sw = nh.advertiseService("Solenoid_SW", ZeabusElec_SetSolenoid);
	ros::ServiceServer service_zeabus_elec_barometer = nh.advertiseService("Barometer", ZeabusElec_GetPressure);
	ros::ServiceServer service_zeabus_elec_send_comm1 = nh.advertiseService("SendComm1", ZeabusElec_SendComm1);
	ros::ServiceServer service_zeabus_elec_send_comm2 = nh.advertiseService("SendComm2", ZeabusElec_SendComm2);
	ros::ServiceServer service_zeabus_elec_receive_comm1 = nh.advertiseService("ReceiveComm1", ZeabusElec_ReceiveComm1);
	ros::ServiceServer service_zeabus_elec_receive_comm2 = nh.advertiseService("ReceiveComm2", ZeabusElec_ReceiveComm2);

	/* Main-loop. Just a spin-lock */
	ros::Rate rate(100);
	while( ros::ok() )
	{
		rate.sleep();
		ros::spinOnce();
	}
	
	/*=================================================================================
	  At this point of code. ROS has some fatal errors or just normal shutdown. Also us. 
	  =================================================================================*/
	delete pxMsspA;
	delete pxMsspB;
	delete pxUartA;
	delete pxUartB;
	for( int i = 0; i < 4; i++ )
	{
		FT_Close( xHandle[ i ] );
	}
	return( 0 );
}