#include <ros/ros.h>
#include <std_msgs/String.h>
#include <zeabus_elec_ros_power_dist/power_dist.h>
#include <sys/syscall.h>
#include <fcntl.h>
#include <stdlib.h>
#include <ftdi_impl.hpp>

#include <sstream>

/* ===================================================
 * File-scope global variables
 * ===================================================
 */
static const char* cstPowerDistSerial = "PowerDist";

static FT_HANDLE	xHandle;		/* Storage of device handle */
static Zeabus_Elec::ftdi_mssp_impl *pxMssp;
static ros::Publisher errMsgPublisher;	/* Publisher for error message */
static ros::Subscriber switchSubscriber; /* Subscriber for switch-controlling message */

/* ===================================================
 * ROS service subroutines
 * ===================================================
 */
void ZeabusElec_SetSwitch( const zeabus_elec_ros_power_dist::power_dist::ConstPtr& msg )
{
	FT_STATUS ftStat;
	
	ftStat = pxMssp->SetHiGPIOData( msg->switchState );
	if( ftStat != FT_OK )
	{
		/* Some Error occurred. So, we publish the error message */
		std_msgs::String msg;
		std::stringstream ss;
		ss << "Error in power-distributor controller with the code " << ftStat;
		msg.data = ss.str();
		
		errMsgPublisher.publish( msg );
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
	ros::init(argc, argv, "Zeabus_Elec_Power_dist");
 	ros::NodeHandle nh("Elec");
	
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

	/* Attempt to open the Power Distributor module described by its serial number */
	xFTStatus = FT_OpenEx( (void*)( cstPowerDistSerial ), FT_OPEN_BY_SERIAL_NUMBER, &( xHandle ) );
	if (xFTStatus != FT_OK)
	{
		/* Fail - unable to open the device with the serial number "PowerDist" */
		ROS_FATAL( "Unable to open Power Distribution device" );
		return( -4 );
	}

	/* All FTDI initialization completed successfully. The xHandle is valid */
	
	/* Create the device manager class to implement chip functions */
	pxMssp = new Zeabus_Elec::ftdi_mssp_impl( FT_DEVICE_232H, xHandle );
	if( pxMssp->GetCurrentStatus() != FT_OK )
	{
		/* Fail - unable to initialize Power Distribution module */
		delete pxMssp;
		ROS_FATAL( "Unable to initialize Power Distribution module" );
		return( -5 );
		
	}
	pxMssp->SetGPIODirection( 0xFFFF );	/* All bits are output */
	
	/*=================================================================================
	  Now the FTDI chip is opened and hooked. We can continue ROS registration process 
	  =================================================================================*/
	
	/* Register ROS publisher to Elec/Hw_error topic */
	errMsgPublisher = nh.advertise<std_msgs::String>( "Hw_error", 100 );
	/* Register ROS subscriber to Elec/Power_dist topic */
	switchSubscriber = nh.subscribe( "Power_switch", 100, ZeabusElec_SetSwitch );

	/* Main-loop. Just a spin-lock */
	ros::spin();
	
	/*=================================================================================
	  At this point of code. ROS has some fatal errors or just normal shutdown. Also us. 
	  =================================================================================*/
	delete pxMssp;
	FT_Close( xHandle );
	return( 0 );
}