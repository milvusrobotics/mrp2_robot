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
#include <linux/serial.h>

#define MAX_LENGTH 128

namespace milvus
{
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
