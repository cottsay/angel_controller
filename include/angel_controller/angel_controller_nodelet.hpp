#ifndef _angel_controller_nodelet_hpp
#define _angel_controller_nodelet_hpp

#include "angel_controller/angel_controller.hpp"

#include <nodelet/nodelet.h>

namespace angel_controller
{
	class angel_controller_nodelet : public nodelet::Nodelet
	{
	public:
		angel_controller_nodelet( const bool _autostart = true );
		~angel_controller_nodelet( );
		bool start( );
		void stop( );
		bool stat( );

	private:
		virtual void onInit( );
		angel_controller *controller;
		const bool autostart;
	};
}

#endif /* _angel_controller_nodelet_hpp */
