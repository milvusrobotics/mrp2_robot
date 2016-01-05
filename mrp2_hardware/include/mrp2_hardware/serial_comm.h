#include <stdio.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>
#include <iostream>
#include <string>
#include <cstring>

#define MAX_LENGTH 128

namespace milvus
{
	//! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
/*	#define DEF_EXCEPTION(name, parent) \
	class name : public parent { \
  		public: \
    	name(const char* msg) : parent(msg) {} \
  	}
  	 
  	//! A standard exception
  	DEF_EXCEPTION(Exception, std::runtime_error);

	//! An exception for use when a timeout is exceeded
  	DEF_EXCEPTION(TimeoutException, Exception);

	#undef DEF_EXCEPTION*/

	/*! \class SerialComm SerialComm.h "inc/SerialComm.h"
	 *  \brief C++ serial port class for ROS.
	 *
	 * This class was based on the serial port code found on the hokuyo_node as suggested by Blaise Gassend on the ros-users mailling list.
	 */
	class SerialComm
	{
		public:
		//! Constructor
		SerialComm();
		//! Destructor
		~SerialComm();

		int open_port(std::string port_name, int baudrate, std::string mode_s);
		int close_port();
		int poll_comport(unsigned char *, int);
		int send_byte(unsigned char);
		int send_buf(unsigned char *, int);
		void cputs(const char *);
		int is_dcd_enabled();
		int is_cts_enabled();
		int is_dsr_enabled();
		void enable_dtr();
		void disable_dtr();
		void enable_rts();
		void disable_rts();

		
		private:
		//! File descriptor
	   	int fd_;
	   	//! Baud rate
		int baud_;
		std::string port_name_;
		struct termios new_port_settings, old_port_settings;
       	int error;
	};

}
