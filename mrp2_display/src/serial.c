
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdbool.h>

#include <time.h>
#include <sys/time.h>

#include <sched.h>
#include <pthread.h>

static int portFd = -1;
bool is_busy = false;

int portOpen (char *device, int baud)
{
  struct termios options;
  speed_t myBaud;
  int     status;

  switch (baud)
  {
    case     50:  myBaud =     B50 ; break;
    case     75:  myBaud =     B75 ; break;
    case    110:  myBaud =    B110 ; break;
    case    134:  myBaud =    B134 ; break;
    case    150:  myBaud =    B150 ; break;
    case    200:  myBaud =    B200 ; break;
    case    300:  myBaud =    B300 ; break;
    case    600:  myBaud =    B600 ; break;
    case   1200:  myBaud =   B1200 ; break;
    case   1800:  myBaud =   B1800 ; break;
    case   2400:  myBaud =   B2400 ; break;
    case   9600:  myBaud =   B9600 ; break;
    case  19200:  myBaud =  B19200 ; break;
    case  38400:  myBaud =  B38400 ; break;
    case  57600:  myBaud =  B57600 ; break;
    case 115200:  myBaud = B115200 ; break;
    case 230400:  myBaud = B230400 ; break;

    default:
      return -2 ;
  }

  if ((portFd = open (device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
    return -1;

  fcntl (portFd, F_SETFL, O_RDWR);

// Get and modify current options:

  tcgetattr (portFd, &options);

  cfmakeraw   (&options);
  cfsetispeed (&options, myBaud);
  cfsetospeed (&options, myBaud);

  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;

  options.c_cc [VMIN]  =   0;
  options.c_cc [VTIME] = 100;  // Ten seconds (100 deciseconds)

  tcsetattr (portFd, TCSANOW | TCSAFLUSH, &options);

  ioctl (portFd, TIOCMGET, &status);

  status |= TIOCM_DTR;
  status |= TIOCM_RTS;

  ioctl (portFd, TIOCMSET, &status);

  usleep (10000);  // 10mS

  return portFd;
}

void portFlush (void)
{
  tcflush (portFd, TCIOFLUSH);
}

void portClose (void)
{
  close (portFd);
}


int portDataAvail (void)
{
  int result;

  if (ioctl (portFd, FIONREAD, &result) == -1)
    return -1;

  return result;
}

int portGetchar (void)
{
  unsigned char x;

    if (portDataAvail ())
    {
      if (read (portFd, &x, 1) == 1)
        return ((int)x) & 0xFF;
      
      return -1;
    }

  return -1;
}


void portPutchar (int data)
{
  unsigned char c = (unsigned char)data;
  write (portFd, &c, 1);
}

void sendCmd(char cmd, char data1, char data2)
{
  while(is_busy)
    usleep(1000);

  is_busy = true;
  char data[4] = {'$', cmd, data1, data2};
  write(portFd, &data, 4);
  is_busy = false;
}

void lockSerial(bool lock)
{
  is_busy = lock;
}

bool getSerial(void)
{
  return is_busy;
}

