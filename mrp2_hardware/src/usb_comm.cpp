#include "mrp2_hardware/usb_comm.h"

#define MILVUS_EXCEPT(except, msg, ...) \
{ \
    char buf[1000]; \
    snprintf(buf, 1000, msg " (in milvus::UsbComm::%s)" , ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf); \
}

milvus::UsbComm::UsbComm()
{
  devh = NULL;
}

milvus::UsbComm::~UsbComm()
{
}

int milvus::UsbComm::open_device(uint16_t vendor_id, uint16_t product_id, int ep_in_addr, int ep_out_addr)
{
  ep_in_addr_ = ep_in_addr;
  ep_out_addr_ = ep_out_addr;

  int rc;

  rc = libusb_init(NULL);
  if (rc < 0) {
      fprintf(stderr, "Error initializing libusb: %s\n", libusb_error_name(rc));
      exit(1);
  }

  libusb_set_debug(NULL, 3);

  devh = libusb_open_device_with_vid_pid(NULL, vendor_id, product_id);
  if (!devh) {
      fprintf(stderr, "Error finding USB device\n");
      close_device();
  }

  for (int if_num = 0; if_num < 2; if_num++) {
      if (libusb_kernel_driver_active(devh, if_num)) {
          libusb_detach_kernel_driver(devh, if_num);
      }
      rc = libusb_claim_interface(devh, if_num);
      if (rc < 0) {
          fprintf(stderr, "Error claiming interface: %s\n",
                  libusb_error_name(rc));
          close_device();
      }
  }

  rc = libusb_control_transfer(devh, 0x21, 0x22, ACM_CTRL_DTR | ACM_CTRL_RTS,
                                0, NULL, 0, 0);
  if (rc < 0) {
      fprintf(stderr, "Error during control transfer: %s\n",
              libusb_error_name(rc));
  }

  /* - set line encoding: here 9600 8N1
   * 9600 = 0x2580 ~> 0x80, 0x25 in little endian
   */
  unsigned char encoding[] = { 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08 };
  rc = libusb_control_transfer(devh, 0x21, 0x20, 0, 0, encoding,
                              sizeof(encoding), 0);
  if (rc < 0) {
      fprintf(stderr, "Error during control transfer: %s\n",
              libusb_error_name(rc));
  }

  return rc;
}

int milvus::UsbComm::close_device() 
{
  libusb_release_interface(devh, 0);

  if (devh)
    libusb_close(devh);

  libusb_exit(NULL);
  return 0;
}

int milvus::UsbComm::read_bytes(unsigned char *buf, int size)
{
  struct timespec  tv1;
  int actual_length;
  int rc = libusb_bulk_transfer(devh, ep_in_addr_, buf, size, &actual_length,
                                1);
  if (rc == LIBUSB_ERROR_TIMEOUT) {
      //gettimeofday(&tv1, NULL);
      clock_gettime(CLOCK_MONOTONIC, &tv1);
      printf("%ld.%06ld - timeout (%d)\n", tv1.tv_sec, tv1.tv_nsec, actual_length);
      //print_array(buf, actual_length);
  } else if (rc < 0) {
      fprintf(stderr, "Error while waiting for char\n");
      return -1;
  }

  return actual_length;
}


int milvus::UsbComm::write_bytes(unsigned char *buf, int size)
{
  struct timespec  tv1;

  int actual_length;
  if (libusb_bulk_transfer(devh, ep_out_addr_, buf, size,
                           &actual_length, 0) < 0) {
      fprintf(stderr, "Error while sending char\n");
  }
  clock_gettime(CLOCK_MONOTONIC, &tv1);
  printf("%ld.%06ld - write sent \n", tv1.tv_sec, tv1.tv_nsec);
}

void milvus::UsbComm::print_array(uint8_t *buf, int length) 
{
  printf("Array: ");
  int i = 0;
  for (i = 0; i < length; i++)
  {
      printf("%02x ",buf[i] );
  }
  printf("\n");
};

