#include "mrp2_hardware/serial_comm.h"

#define MILVUS_EXCEPT(except, msg, ...) \
{ \
    char buf[1000]; \
    snprintf(buf, 1000, msg " (in milvus::SerialComm::%s)" , ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf); \
}

milvus::SerialComm::SerialComm() : fd_(-1)
{
}

milvus::SerialComm::~SerialComm()
{
}

int milvus::SerialComm::open_port(std::string port_name, int baudrate, std::string mode_s)
{
	int baudr,
      status;

  const char *mode = mode_s.c_str();

  switch(baudrate)
  {
    case      50 : baudr = B50;
                   break;
    case      75 : baudr = B75;
                   break;
    case     110 : baudr = B110;
                   break;
    case     134 : baudr = B134;
                   break;
    case     150 : baudr = B150;
                   break;
    case     200 : baudr = B200;
                   break;
    case     300 : baudr = B300;
                   break;
    case     600 : baudr = B600;
                   break;
    case    1200 : baudr = B1200;
                   break;
    case    1800 : baudr = B1800;
                   break;
    case    2400 : baudr = B2400;
                   break;
    case    4800 : baudr = B4800;
                   break;
    case    9600 : baudr = B9600;
                   break;
    case   19200 : baudr = B19200;
                   break;
    case   38400 : baudr = B38400;
                   break;
    case   57600 : baudr = B57600;
                   break;
    case  115200 : baudr = B115200;
                   break;
    case  230400 : baudr = B230400;
                   break;
    case  460800 : baudr = B460800;
                   break;
    case  500000 : baudr = B500000;
                   break;
    case  576000 : baudr = B576000;
                   break;
    case  921600 : baudr = B921600;
                   break;
    case 1000000 : baudr = B1000000;
                   break;
    case 1152000 : baudr = B1152000;
                   break;
    case 1500000 : baudr = B1500000;
                   break;
    case 2000000 : baudr = B2000000;
                   break;
    case 2500000 : baudr = B2500000;
                   break;
    case 3000000 : baudr = B3000000;
                   break;
    case 3500000 : baudr = B3500000;
                   break;
    case 4000000 : baudr = B4000000;
                   break;
    default      : printf("invalid baudrate\n");
                   return(1);
                   break;
  }

  int cbits=CS8,
      cpar=IGNPAR,
      bstop=0;

  if(strlen(mode) < 3)
  {
    printf("invalid mode:%d \"%s\"\n",(int)strlen(mode), mode);
    return(1);
  }

  switch(mode[0])
  {
    case '8': cbits = CS8;
              break;
    case '7': cbits = CS7;
              break;
    case '6': cbits = CS6;
              break;
    case '5': cbits = CS5;
              break;
    default : printf("invalid number of data-bits '%c'\n", mode[0]);
              return(1);
              break;
  }

  switch(mode[1])
  {
    case 'N':
    case 'n': cpar = IGNPAR;
              break;
    case 'E':
    case 'e': cpar = PARENB;
              break;
    case 'O':
    case 'o': cpar = (PARENB | PARODD);
              break;
    default : printf("invalid parity '%c'\n", mode[1]);
              return(1);
              break;
  }

  switch(mode[2])
  {
    case '1': bstop = 0;
              break;
    case '2': bstop = CSTOPB;
              break;
    default : printf("invalid number of stop bits '%c'\n", mode[2]);
              return(1);
              break;
  }


  fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
  if(fd_==-1)
  {
    perror("unable to open comport ");
    return(1);
  }
  else
  {
    fcntl(fd_, F_SETFL, 0);
  }

  error = tcgetattr(fd_, &old_port_settings);
  if(error==-1)
  {
    close(fd_);
    perror("unable to read portsettings ");
    return(1);
  }
  memset(&new_port_settings, 0, sizeof(new_port_settings));  

  new_port_settings.c_cflag = cbits | cpar | bstop | CLOCAL | CREAD;
  new_port_settings.c_iflag = IGNPAR;
  new_port_settings.c_oflag = 0;
  new_port_settings.c_lflag = 0;
  new_port_settings.c_cc[VMIN] = 0;    
  new_port_settings.c_cc[VTIME] = 0;    

  cfsetispeed(&new_port_settings, baudr);
  cfsetospeed(&new_port_settings, baudr);

  error = tcsetattr(fd_, TCSANOW, &new_port_settings);
  if(error==-1)
  {
    close(fd_);
    perror("unable to adjust portsettings ");
    return(1);
  }

  if(ioctl(fd_, TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
    return(1);
  }

  status |= TIOCM_DTR;   
  status |= TIOCM_RTS; 

  if(ioctl(fd_, TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
    return(1);
  }
  

  return(0);
}

int milvus::SerialComm::close_port() 
{
	int status;

	if(ioctl(fd_, TIOCMGET, &status) == -1)
	{
		perror("unable to get portstatus");
	}

	status &= ~TIOCM_DTR;    /* turn off DTR */
	status &= ~TIOCM_RTS;    /* turn off RTS */

	if(ioctl(fd_, TIOCMSET, &status) == -1)
	{
		perror("unable to set portstatus");
	}

	close(fd_);
}

int milvus::SerialComm::poll_comport(unsigned char *buf, int size)
{
	int n;
	n = read(fd_, buf, size);
	return(n);
}

int milvus::SerialComm::send_byte(unsigned char)
{

}

int milvus::SerialComm::send_buf(unsigned char *buf, int size)
{
	return(write(fd_, buf, size));
}

void milvus::SerialComm::cputs(const char *)
{

}

int milvus::SerialComm::is_dcd_enabled()
{

}

int milvus::SerialComm::is_cts_enabled()
{

}

int milvus::SerialComm::is_dsr_enabled()
{

}

void milvus::SerialComm::enable_dtr()
{

}

void milvus::SerialComm::disable_dtr()
{

}

void milvus::SerialComm::enable_rts()
{

}

void milvus::SerialComm::disable_rts()
{

}

