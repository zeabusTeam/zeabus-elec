#include <ros/ros.h>
#include <std_msgs/String.h>
#include <zeabus_elec_ros_peripheral_bridge/barometer.h>
#include <zeabus_elec_ros_peripheral_bridge/solenoid_sw.h>
#include <zeabus_elec_ros_peripheral_bridge/comm_data.h>
#include <sys/syscall.h>
#include <fcntl.h>
#include <stdlib.h>
#include <ftdi_impl.hpp>

#include <sstream>

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
static ros::Publisher errMsgPublisher;	/* Publisher for error message */
static ros::Publisher barometerPublisher;	/* Publisher for barometer message */
static ros::Publisher comm1RecvPublisher;	/* Publisher for comm1 received-data message */
static ros::Publisher comm2RecvPublisher;	/* Publisher for comm2 received-data message */
static ros::Subscriber solenoidSubscriber; /* Subscriber for solenoid-controlling message */
static ros::Subscriber comm1SendSubscriber; /* Subscriber for comm1 sent-data message */
static ros::Subscriber comm2SendSubscriber; /* Subscriber for comm2 sent-data message */

/* ===================================================
 * ROS service subroutines
 * ===================================================
 */

/* Set the solenoid switches. We have maximum 8 switches. The low-nibble 4 switches 
is controlled by MSSP_A and the other 4 switches is controlled by MSSP_B */
void ZeabusElec_SetSolenoid( const zeabus_elec_ros_peripheral_bridge::solenoid_sw::ConstPtr& msg )
{
	uint8_t swNibble;
	
	FT_STATUS ftStat;
	
	/* Low nibble */
	swNibble = ( msg->switchState ) & 0x0F;
	ftStat = pxMsspA->SetLoGPIOData( swNibble );
	if( ftStat != FT_OK )
	{
		/* Some Error occurred. So, we publish the error message */
		std_msgs::String msg;
		std::stringstream ss;
		ss << "Error in solenoid controller with the code " << ftStat;
		msg.data = ss.str();
		
		errMsgPublisher.publish( msg );
	}

	/* High nibble */
	swNibble = ( msg->switchState ) >> 4;
	ftStat = pxMsspA->SetLoGPIOData( swNibble );
	if( ftStat != FT_OK )
	{
		/* Some Error occurred. So, we publish the error message */
		std_msgs::String msg;
		std::stringstream ss;
		ss << "Error in solenoid controller with the code " << ftStat;
		msg.data = ss.str();
		
		errMsgPublisher.publish( msg );
	}
}

/* Send a stream of uint8 to RS232 port 1 */
void ZeabusElec_SendComm1( const zeabus_elec_ros_peripheral_bridge::comm_data::ConstPtr& msg )
{
	uint8_t* pucBuffer;
	uint32_t ulDataDone;
	
	pucBuffer = (uint8_t*)malloc( msg->len );
	for( int i = 0; i < msg->len; i++ )
	{
		pucBuffer[i] = ( msg->data )[i];
	}
	ulDataDone = pxUartA->Send( pucBuffer, msg->len );
	free( pucBuffer );
	
	if( ( pxUartA->GetCurrentStatus() != FT_OK ) || ( ulDataDone != msg->len ) )
	{
		/* Some Error occurred. So, we publish the error message */
		std_msgs::String eMsg;
		std::stringstream ss;
		ss << "Fail to send " << ( msg->len ) << " bytes to COMM1. Only " << ulDataDone << " are sent and error code is " << pxUartA->GetCurrentStatus();
		eMsg.data = ss.str();
		
		errMsgPublisher.publish( eMsg );
	}
}

/* Send a stream of uint8 to RS232 port 1 */
void ZeabusElec_SendComm2( const zeabus_elec_ros_peripheral_bridge::comm_data::ConstPtr& msg )
{
	uint8_t* pucBuffer;
	uint32_t ulDataDone;
	
	pucBuffer = (uint8_t*)malloc( msg->len );
	for( int i = 0; i < msg->len; i++ )
	{
		pucBuffer[i] = ( msg->data )[i];
	}
	ulDataDone = pxUartA->Send( pucBuffer, msg->len );
	free( pucBuffer );
	
	if( ( pxUartB->GetCurrentStatus() != FT_OK ) || ( ulDataDone != msg->len ) )
	{
		/* Some Error occurred. So, we publish the error message */
		std_msgs::String eMsg;
		std::stringstream ss;
		ss << "Unable to send " << ( msg->len ) << " bytes to COMM2. Only " << ulDataDone << " are sent and error code is " << pxUartA->GetCurrentStatus();
		eMsg.data = ss.str();
		
		errMsgPublisher.publish( eMsg );
	}
}

/* Read barometer value. The barometer is connected by SPI protocol through MSSP_A */
bool ZeabusElec_GetPressure( uint16_t& barometerVal)
{
	uint8_t aucVal[3]; /* Data buffer. The raw data are in 2 bytes as a stream */
	FT_STATUS xFTStatus;
	
	/* To read barometer value, we need to send a 16-bit dummy data first */
	xFTStatus = pxMsspA->Send( aucVal, 2 );
	if( xFTStatus != FT_OK )
	{
		barometerVal = 0;
		return( false );
	}
	
	/* Read the actual 2-byte raw data. The actual data is 10 bits plus some unused bit as:
		   0 0 0 D D D D D  D D D D D x x x
		where the first byte is the MSB. */
	xFTStatus = pxMsspA->Receive( aucVal, 2 );
	if( xFTStatus != FT_OK )
	{
		barometerVal = 0;
		return( false );
	}
	
	/* Extract the data from 2-byte message */
	barometerVal = (uint16_t)( aucVal[ 0 ] ); /* High byte */
	barometerVal <<= 8;
	barometerVal |= (uint16_t)( aucVal[ 1 ] ); /* Low byte */
	barometerVal >>= 3;	/* Suppress 3 tailing zero bits */

	return( true );	/* All success */	
}

/* Receive available data from either COMM1 or COMM2 */
void ZeabusElec_ReceiveComm( Zeabus_Elec::ftdi_impl* commPort, uint8_t commID, ros::Publisher& commPublisher )
{
	uint8_t* pucBuffer;
	uint32_t ulDataDone, ulDataAvailable;

	ulDataAvailable = commPort->CheckWaitingData();
	if( commPort->GetCurrentStatus() != FT_OK )
	{
		/* Cannot check the available data in COMM */
		std_msgs::String msg;
		std::stringstream ss;
		ss << "Fail to check available data in COMM" << commID << " with error code " << commPort->GetCurrentStatus();
		msg.data = ss.str();
		
		errMsgPublisher.publish( msg );
	}
	else
	{
		if( ulDataAvailable > 0 )
		{
			/* We have some data waiting */
			pucBuffer = (uint8_t*)malloc( ulDataAvailable );
			if( pucBuffer == NULL )
			{
				/* Cannot allocate buffer for COMM data */
				std_msgs::String msg;
				std::stringstream ss;
				ss << "Fail to allocate buffer to receive COMM" << commID << " data";
				msg.data = ss.str();
		
				errMsgPublisher.publish( msg );
			}
			else
			{
				/* Reading from the COMM port */
				ulDataDone = commPort->Receive( pucBuffer, ulDataAvailable );
				if( commPort->GetCurrentStatus() != FT_OK )
				{
					/* Reading finished with error */
					std_msgs::String msg;
					std::stringstream ss;
					ss << "Fail to receive data from COMM" << commID << " with error code " << commPort->GetCurrentStatus();
					msg.data = ss.str();
	
					errMsgPublisher.publish( msg );
				}
				else
				{
					if( ulDataAvailable != ulDataDone )
					{
						/* Successfully read but got only part of data */
						std_msgs::String msg;
						std::stringstream ss;
						ss << "Reading from COMM" << commID << " was successful but we get " << ulDataDone 
								<< " bytes instead of " << ulDataAvailable << " bytes.";
						msg.data = ss.str();
		
						errMsgPublisher.publish( msg );
					}

					/* Send the data to the topic channel */
					zeabus_elec_ros_peripheral_bridge::comm_data commMsg;
					for( int i = 0; i < ulDataDone; i++ )
					{
						commMsg.data[i] = pucBuffer[i];
					}
					commMsg.len = ulDataDone;
					commPublisher.publish( commMsg );
				}
				free( pucBuffer );
			}
		}
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
	uint16_t usBarometerVal;
	int comm1BaudRate, comm2BaudRate;

	/* Initialize ROS functionalities */	
	ros::init(argc, argv, "Zeabus_Elec_Peripheral_bridge");
 	ros::NodeHandle nh("Elec");
 	
 	/* Retrieve parameter from launch file */
	nh.param < int > ("comm1baudrate", comm1BaudRate, 115200);
	nh.param < int > ("comm2baudrate", comm1BaudRate, 115200);
	
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
	
	pxUartA = new Zeabus_Elec::ftdi_uart_impl( FT_DEVICE_4232H, xHandle[2], (uint32_t)comm1BaudRate );
	if( pxUartA->GetCurrentStatus() != FT_OK )
	{
		/* Fail - unable to initialize Power Distribution module */
		delete pxMsspA;
		delete pxMsspB;
		delete pxUartA;
		ROS_FATAL( "Unable to initialize Power Distribution module C" );
		return( -7 );
		
	}

	pxUartB = new Zeabus_Elec::ftdi_uart_impl( FT_DEVICE_4232H, xHandle[3], (uint32_t)comm2BaudRate );
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
	
	/* Register ROS publishers for power-distributor switch controller */
	errMsgPublisher = nh.advertise<std_msgs::String>( "Hw_error", 1000 );	/* Publisher for error message */
	barometerPublisher = nh.advertise<zeabus_elec_ros_peripheral_bridge::barometer>( "Barometer", 100 );	/* Publisher for barometer message */
	comm1RecvPublisher = nh.advertise<zeabus_elec_ros_peripheral_bridge::comm_data>( "Comm1/Recv", 100 );	/* Publisher for comm1 received-data message */
	comm2RecvPublisher = nh.advertise<zeabus_elec_ros_peripheral_bridge::comm_data>( "Comm2/Recv", 100 );	/* Publisher for comm2 received-data message */

	/* Register ROS subscribers for power-distributor switch controller */
	solenoidSubscriber = nh.subscribe( "Solenoid_sw", 100, ZeabusElec_SetSolenoid ); /* Subscriber for solenoid-controlling message */
	comm1SendSubscriber = nh.subscribe( "Comm1/Send", 100, ZeabusElec_SendComm1 ); /* Subscriber for comm1 sent-data message */
	comm2SendSubscriber = nh.subscribe( "Comm2/Send", 100, ZeabusElec_SendComm2 ); /* Subscriber for comm2 sent-data message */

	/* Main-loop. Just a spin-lock and wakeup at every 10ms (100Hz) */
	ros::Rate rate(100);
	while( ros::ok() )
	{
		/* Wait for the next cycle */
		rate.sleep();
		ros::spinOnce();
		
		/* Read barometer */
		if( !( ZeabusElec_GetPressure( usBarometerVal ) ) )
		{
			/* Fail to read from barometer */
			std_msgs::String msg;
			std::stringstream ss;
			ss << "Fail to read from barometer with error code " << pxMsspA->GetCurrentStatus();
			msg.data = ss.str();
		
			errMsgPublisher.publish( msg );
		}
		
		/* Read from COMM1 */
		ZeabusElec_ReceiveComm( pxUartA, 1, comm1RecvPublisher );
			
		/* Read from COMM2 */
		ZeabusElec_ReceiveComm( pxUartB, 2, comm2RecvPublisher );
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