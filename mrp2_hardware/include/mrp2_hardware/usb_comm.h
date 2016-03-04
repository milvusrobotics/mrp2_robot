#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string>
#include <libusb-1.0/libusb.h>

#define ACM_CTRL_DTR   0x01
#define ACM_CTRL_RTS   0x02

namespace milvus
{
	class UsbComm
	{
		public:
		//! Constructor
		UsbComm();
		//! Destructor
		~UsbComm();

		int open_device(uint16_t vendor_id, uint16_t product_id, int ep_in_addr, int ep_out_addr);
		int close_device();
		int read_bytes(unsigned char *, int);
		int write_bytes(unsigned char *, int);
		
		private:
		//! Device handle
		void print_array(uint8_t *buf, int length);
		
	   	struct libusb_device_handle *devh;
	   	int ep_in_addr_;
	   	int ep_out_addr_;
	   	//! Baud rate
		int baud_;
		std::string port_name_;
       	int error;
	};

}
