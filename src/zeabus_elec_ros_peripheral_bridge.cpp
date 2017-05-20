#include <ros/ros.h>
#include <std_msgs/String.h>
#include <zeabus_elec_ros_peripheral_bridge/barometer.h>
#include <zeabus_elec_ros_peripheral_bridge/solenoid_sw.h>
#include <zeabus_elec_ros_peripheral_bridge/comm_data.h>
#include <ftdi_impl.h>

#include <sstream>
#include <string>
#include <array>
#include <vector>
#include <memory>

/* ===================================================
 * File-scope global variables
 * ===================================================
 */
static const char* stPeripheralSerial = "PeripheralBridge";

static std::shared_ptr<Zeabus_Elec::ftdi_impl> pxMsspA, pxMsspB, pxUartA, pxUartB;
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
	
	int ftStat;
	
	/* Low nibble */
	swNibble = ( msg->switchState ) & 0x0F;
	ftStat = pxMsspA->SetLoGPIOData( swNibble );
	if( ftStat != 0 )
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
	if( ftStat != 0 )
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
	std::vector<uint8_t> pucBuffer;
	uint32_t ulDataDone;
	
	pucBuffer.reserve( msg->len );
	for(int i = 0; i < msg->len; i++ )
	{
		pucBuffer.push_back( ( msg->data )[ i ] );
	}
	ulDataDone = pxUartA->Send( pucBuffer );
	
	if( ( pxUartA->GetCurrentStatus() != 0 ) || ( ulDataDone != msg->len ) )
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
	std::vector<uint8_t> pucBuffer;
	uint32_t ulDataDone;
	
	pucBuffer.reserve( msg->len );
	for(int i = 0; i < msg->len; i++ )
	{
		pucBuffer.push_back( ( msg->data )[ i ] );
	}
	ulDataDone = pxUartB->Send( pucBuffer );
	
	if( ( pxUartB->GetCurrentStatus() != 0 ) || ( ulDataDone != msg->len ) )
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
	std::vector<uint8_t> aucVal(2, 0); /* Data buffer. The raw data are in 2 bytes as a stream */
	int xFTStatus;
	
	/* To read barometer value, we need to send a 16-bit dummy data first */
	xFTStatus = pxMsspA->Send( aucVal );
	if( xFTStatus != 0 )
	{
		barometerVal = 0;
		return( false );
	}
	
	/* Read the actual 2-byte raw data. The actual data is 10 bits plus some unused bit as:
		   0 0 0 D D D D D  D D D D D x x x
		where the first byte is the MSB. */
	xFTStatus = pxMsspA->Receive( aucVal );
	if( xFTStatus != 0 )
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
void ZeabusElec_ReceiveComm( std::shared_ptr<Zeabus_Elec::ftdi_impl> commPort, uint8_t commID, ros::Publisher& commPublisher )
{
	std::vector<uint8_t> pucBuffer;
	uint32_t ulDataDone;

	/* Reading from the COMM port */
	ulDataDone = commPort->Receive( pucBuffer );
	if( commPort->GetCurrentStatus() != 0 )
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
		/* Send the data to the topic channel */
		zeabus_elec_ros_peripheral_bridge::comm_data commMsg;
		for( int i = 0; i < pucBuffer.size(); i++ )
		{
			commMsg.data[i] = pucBuffer[i];
		}
		commMsg.len = ulDataDone;
		commPublisher.publish( commMsg );
	}
}

/* ===================================================
 * main
 * ===================================================
 */
int main( int argc, char** argv )
{
	uint16_t usBarometerVal;
	int comm1BaudRate, comm2BaudRate;

	/* Initialize ROS functionalities */	
	ros::init(argc, argv, "Zeabus_Elec_Peripheral_bridge");
 	ros::NodeHandle nh("/zeabus/elec");
 	
 	/* Retrieve parameter from launch file */
	nh.param < int > ("comm1baudrate", comm1BaudRate, 115200);
	nh.param < int > ("comm2baudrate", comm1BaudRate, 115200);
	
	/*=================================================================================
	  Discover the Power Distributor and also open handles for it.
	  =================================================================================*/

	/* Attempt to open the Peripheral-Bridge modules described by their serial number */
	/* The module utilizes FT4232H consisting of 4 module instances inside */

	/* Create the device manager classes to implement chip functions */
	pxMsspA = std::make_shared<Zeabus_Elec::ftdi_spi_cpol1_cha0_msb_impl>( Zeabus_Elec::ftdi_spi_cpol1_cha0_msb_impl( Zeabus_Elec::FT4232H, stPeripheralSerial, 0 ) );
	if( pxMsspA->GetCurrentStatus() != 0 )
	{
		/* Fail - unable to initialize Power Distribution module */
		ROS_FATAL( "Unable to initialize Power Distribution module A" );
		return( -5 );
		
	}
	pxMsspA->SetGPIODirection( 0xFFFF );	/* All bits are output */

	pxMsspB = std::make_shared<Zeabus_Elec::ftdi_spi_cpol1_cha0_msb_impl>( Zeabus_Elec::ftdi_spi_cpol1_cha0_msb_impl( Zeabus_Elec::FT4232H, stPeripheralSerial, 1 ) );
	if( pxMsspB->GetCurrentStatus() != 0 )
	{
		/* Fail - unable to initialize Power Distribution module */
		ROS_FATAL( "Unable to initialize Power Distribution module B" );
		return( -6 );
		
	}
	pxMsspB->SetGPIODirection( 0xFFFF );	/* All bits are output */
	
	pxUartA = std::make_shared<Zeabus_Elec::ftdi_uart_impl>( Zeabus_Elec::ftdi_uart_impl( Zeabus_Elec::FT4232H, stPeripheralSerial, 2, (uint32_t)comm1BaudRate ) );
	if( pxUartA->GetCurrentStatus() != 0 )
	{
		/* Fail - unable to initialize Power Distribution module */
		ROS_FATAL( "Unable to initialize Power Distribution module C" );
		return( -7 );
		
	}

	pxUartB = std::make_shared<Zeabus_Elec::ftdi_uart_impl>( Zeabus_Elec::ftdi_uart_impl( Zeabus_Elec::FT4232H, stPeripheralSerial, 3, (uint32_t)comm2BaudRate ) );
	if( pxUartB->GetCurrentStatus() != 0 )
	{
		/* Fail - unable to initialize Power Distribution module */
		ROS_FATAL( "Unable to initialize Power Distribution module D" );
		return( -8 );
		
	}

	/*=================================================================================
	  Now the FTDI chip is opened and hooked. We can continue ROS registration process 
	  =================================================================================*/
	
	/* Register ROS publishers for power-distributor switch controller */
	errMsgPublisher = nh.advertise<std_msgs::String>( "hw_error", 1000 );	/* Publisher for error message */
	barometerPublisher = nh.advertise<zeabus_elec_ros_peripheral_bridge::barometer>( "barometer", 100 );	/* Publisher for barometer message */
	comm1RecvPublisher = nh.advertise<zeabus_elec_ros_peripheral_bridge::comm_data>( "comm1/recv", 100 );	/* Publisher for comm1 received-data message */
	comm2RecvPublisher = nh.advertise<zeabus_elec_ros_peripheral_bridge::comm_data>( "comm2/recv", 100 );	/* Publisher for comm2 received-data message */

	/* Register ROS subscribers for power-distributor switch controller */
	solenoidSubscriber = nh.subscribe( "solenoid_sw", 100, ZeabusElec_SetSolenoid ); /* Subscriber for solenoid-controlling message */
	comm1SendSubscriber = nh.subscribe( "comm1/send", 100, ZeabusElec_SendComm1 ); /* Subscriber for comm1 sent-data message */
	comm2SendSubscriber = nh.subscribe( "comm2/send", 100, ZeabusElec_SendComm2 ); /* Subscriber for comm2 sent-data message */

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
	return( 0 );
}