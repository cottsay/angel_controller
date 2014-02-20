#include "angel_controller/angel_controller_nodelet.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS( angel_controller, angel_controller_nodelet, angel_controller::angel_controller_nodelet, nodelet::Nodelet )

namespace angel_controller
{

angel_controller_nodelet::angel_controller_nodelet( const bool _autostart ) :
	controller( NULL ),
	autostart( _autostart )
{
}

angel_controller_nodelet::~angel_controller_nodelet( )
{
	delete controller;
}

void angel_controller_nodelet::onInit( )
{
	controller = new angel_controller( getNodeHandle( ), getPrivateNodeHandle( ) );

	if( autostart && !start( ) )
		ROS_ERROR( "Failed to start controller" );
}

bool angel_controller_nodelet::start( )
{
	return ( NULL != controller ) && controller->start( );
}

void angel_controller_nodelet::stop( )
{
	if( NULL != controller )
		controller->stop( );
}

bool angel_controller_nodelet::stat( )
{
	return ( NULL != controller ) && controller->stat( );
}

}
