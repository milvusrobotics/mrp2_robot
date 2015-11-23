#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "rs232.h"
#include "time.h"
#include <string>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h" 
#include "std_msgs/Empty.h" 
#include "std_msgs/Bool.h" 

using std::string;

int speeds[2] = {0,0};
char sendArray[20];
bool e_stop = false;
bool dir_left =false;
bool dir_right=true;

int port_nr = 4;
int baudrate = 38400;
char mode[3]={'8', 'N', '1' };

uint8_t tempData[256];
uint8_t tempDataIndex = 0;

bool seekForChar = true;

char startChar = '$';

double Kp,Ki,Kd,Kol;

ros::Time speed_last_time;

ros::Subscriber joy_sub;
ros::Subscriber speeds_sub;
ros::Subscriber estop_sub;
ros::Subscriber resetenc_sub;
ros::Subscriber clear_estop_sub;

ros::Publisher position_pub;
ros::Publisher speed_right_pub;
ros::Publisher speed_left_pub;
ros::Publisher set_speed_left_pub;
ros::Publisher set_speed_right_pub;
ros::Publisher bumpers_pub;

//serial::Serial mrp_serial("/dev/ttyS4", 9600, serial::Timeout::simpleTimeout(1000),serial::bytesize_t::eightbits,serial::parity_t::parity_none,serial::stopbits_t::stopbits_one,serial::flowcontrol_t::flowcontrol_none);

unsigned char checksum(int size)
{
  int ret = 0;
  for(int i=0; i<size; i++)
  {
    ret = ret + sendArray[i];
  }
  if(size > 4)
  {
    return (ret & 255) - 1; 
  }
  return ret & 255 ;
}

unsigned char checksum_check_array(uint8_t *arr, int size)
{
  int ret = 0;
  for(int i=0; i<size; i++)
  {
    ret = ret + arr[i];
  }
  if(size > 4)
  {
    ret = (ret & 0xFF) - 1; 
  }
  else{
    ret = ret & 0xFF; 
  }
  return ret;
}

unsigned char checksum_check(int size)
{
  int ret = 0;
  for(int i=0; i<size; i++)
  {
    ret = ret + sendArray[i];
  }
  if(size > 4)
  {
    ret = (ret & 0xFF) - 1; 
  }
  else{
    ret = ret & 0xFF; 
  }
  return ret;
}

void setSpeeds()
{
  uint8_t send_array[20];
  if(speeds[0] < 0){
    dir_left = 1;
    speeds[0] *= -1;
  }else{
    dir_left = 0;
  }

  if(speeds[1] < 0){
    dir_right = 1;
    speeds[1] *= -1;
  }else{
    dir_right = 0;
  }

  unsigned char s1 = (unsigned char)(speeds[0] >> 16);
  unsigned char s2 = (unsigned char)(speeds[0] >> 8);
  unsigned char s3 = (unsigned char)speeds[0];

  unsigned char s4 = (unsigned char)(speeds[1] >> 16);
  unsigned char s5 = (unsigned char)(speeds[1] >> 8);
  unsigned char s6 = (unsigned char)speeds[1];
  
  //printf("%d,%d,%d\n",s1,s2,s3);  

  

  send_array[0] = '$';
  send_array[1] = e_stop;
  send_array[2] = 'S';
  send_array[3] = 8;
  send_array[4] = checksum_check_array(send_array, 4);
  send_array[5] = dir_left;
  send_array[6] = s1;
  send_array[7] = s2;
  send_array[8] = s3;
  send_array[9] = dir_right;
  send_array[10] = s4;
  send_array[11] = s5;
  send_array[12] = s6;
  send_array[13] = checksum_check_array(send_array, 13);

  //printf("chk:%d\n", sendArray[4]);
  //string out(sendArray, 14);
/*
      for (int i = 0; i < out.length(); i++)
    {
      printf("%02x, ", out[i]);
    }
     printf("\n");
*/
  //ROS_INFO("SPEED OUT, DATA: %d, %d, TS: %f", speeds[0], speeds[1], ros::Time::now().toSec());
  //mrp_serial.write(out);
  int ret =RS232_SendBuf(port_nr,send_array,14);
    //ROS_INFO("Response =%d\n",ret);

}

void clear_estop()
{
  uint8_t send_array[15];
  
  send_array[0] = '$';
  send_array[1] = e_stop;
  send_array[2] = 'E';
  send_array[3] = 1;
  send_array[4] = checksum_check_array(send_array, 4);
  send_array[5] = 1;
  send_array[6] = checksum_check_array(send_array, 6);

  int ret =RS232_SendBuf(port_nr,send_array,7);
}

void clearestopCallback(const std_msgs::Bool::ConstPtr& msg)
{ 
  clear_estop(); 
}

void estopCallback(const std_msgs::Bool::ConstPtr& msg)
{ 
  //e_stop = msg->data; //TODO STM düzeltilecek. 
}

void sendResetEncoders()
{
  uint8_t send_array[10];
  ROS_INFO("reset\n");
  send_array[0] = '$';
  send_array[1] = e_stop;
  send_array[2] = 'R';
  send_array[3] = 0;
  send_array[4] = checksum(4);
  
  /*string out(sendArray, 6);
   for (int i = 0; i < out.length(); i++)
    {
      printf("%02x, ", out[i]);
    }
     printf("\n");*/
  //mrp_serial.write(out);
  int ret =RS232_SendBuf(port_nr,send_array,6);
}
void resetencCallback(const std_msgs::Empty::ConstPtr& msg)
{
  sendResetEncoders();

}

void speedsCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{ 
  int i = 0;
  std_msgs::Int32 speed_msg;

  // speeds[0], speeds[1]
  for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
  {
    speeds[i] = *it;
    i++;
  }
  //printf("l: %d, r: %d\n",speeds[0],speeds[1]);
  speed_last_time = ros::Time::now();

  speed_msg.data = speeds[0];
  set_speed_left_pub.publish(speed_msg);
  speed_msg.data = speeds[1];
  set_speed_right_pub.publish(speed_msg);
  setSpeeds();
  return; 
}

void arrayChopper(uint8_t *buf, int start, int end)
{
    //memcpy(&tempData[0], &buf[start], end);
  int k = 0;
  for (int i = start; i < start+end; i++)
  {
    tempData[k] = buf[i];
    k++;
  }
}
bool checksumMatch(uint8_t *buf, int size)
{

    int checksum = 0;
    int i = 0;
    for(i = 0; i<=size-1; i++) // Sum-up all bytes
    {
        checksum = checksum + buf[i];
    }
    if(size > 4)
    {
      checksum = (checksum & 0xFF) - 1; // Standard MRP Serial checksum procedure
      //ROS_INFO("chksm: %02x",checksum);
    }else{
      checksum = checksum & 0xFF; // Standard MRP Serial checksum procedure
    }
    

    if(checksum == buf[size])    // Return result of the calculation
    {
        return true;
    }    
    return false;
}

int firstValidator(uint8_t *buf)
{
    if(checksumMatch(buf, 4))
    {
        return buf[3];
    }
    return 0;
}

int secondValidator(uint8_t *buf, int data_len)
{
    int total_len = data_len + 6;
    if(checksumMatch(buf, 5 + data_len))
    {
        return 1;
    }
    return 0;
}



int findMessageStart(uint8_t *buf,  int lastIndex)
{
    int start = 0;
    for (start = 0; start < lastIndex ; start++)
    {
        if(buf[start] == startChar)
        {
            break;
        }
    }
    
    lastIndex = lastIndex - start;
            //ROS_INFO("char found at index: %d\n", start);
            //ROS_INFO("kopyalanacak sayı: %d",lastIndex+1);
            //printf("oldDataIndex: %d\n", old_tempDataIndex);
    arrayChopper(buf, start, lastIndex+1);// '$' öncesini çöpe at
    
    return lastIndex;
}


void executeCommand(uint8_t *buf)
{
//ROS_INFO("okkkkkkkkkkkkkkkkkkkkkkkkkkkk");

  std_msgs::Int32MultiArray array;
  std_msgs::Int32 int_msg;
  array.data.clear();

  if(buf[2] == 'S')
  {
      int l_position = buf[6] + (buf[7] << 8) + (buf[8] << 16) + (buf[9] << 24);
      if(buf[5] == 1)
      {
        l_position *= -1;
      }
        

      int r_position = buf[11] + (buf[12] << 8) + (buf[13] << 16) + (buf[14] << 24);
      if(buf[10] == 1)
      {
        r_position *= -1;
      }
        

      array.data.push_back(l_position);
      array.data.push_back(r_position);
      position_pub.publish(array);

      //ROS_INFO("left: %d, right: %d\n",l_position ,r_position);

  }

  if(buf[2] == 'B')
  {
    array.data.push_back(!((buf[5] >> 3) & 0x01));
    array.data.push_back(!((buf[5] >> 2) & 0x01));
    array.data.push_back(!((buf[5] >> 1) & 0x01));
    array.data.push_back(!(buf[5] & 0x01));
    bumpers_pub.publish(array);
    //ROS_INFO("Bumpers: %02x", buf[5]);
  }

  if(buf[2] == 'X')
  {
      int l_speed = buf[6];
      l_speed += (buf[7] << 8);
      if(buf[5] == 1)
      {
        l_speed *= -1;
      }
        

      int r_speed= buf[9];
      r_speed += (buf[10] << 8);
      if(buf[8] == 1)
      {
        r_speed *= -1;
      }
        
      int_msg.data = l_speed;
      speed_left_pub.publish(int_msg);
      int_msg.data = r_speed;
      speed_right_pub.publish(int_msg);

      //ROS_INFO("left: %d, right: %d\n",l_position ,r_position);

  }

}

void printArray(uint8_t *buf, int length)
{
    printf("Array: ");
    int i = 0;
    for (i = 0; i < length; i++)
    {
        printf("%02x ",buf[i] );
    }
    printf("\n");
}


void serialReadProtocol(void)
{
    uint8_t inData[255];
    char inData_c[255];
    int recievedData = 0;
    int startIndex = 0;

    //recievedData = mrp_serial.read(inData,20);

    recievedData = RS232_PollComport(port_nr, inData, 50); // Immediately returns
  //ROS_INFO("recieved: %d\n",recievedData);

    if (recievedData > 0) // Eğer data alınmışsa
    {

        //printf("-------------------------------------------------\n");
       // ROS_INFO("recieved: %d\n",recievedData);
        //printArray(inData, recievedData);
        if(tempDataIndex == 0) // Daha önceden hiç data yoksa
        {
            memcpy(tempData, inData, recievedData); // datayı temp'e al
            tempDataIndex = recievedData;
        }else{ // Daha önceden data var fakat yeterli değil
            //ROS_INFO("recievedData: %d\n", recievedData);
            memcpy(&tempData[tempDataIndex], &inData[0], recievedData); // datayı temp'e ekler
            tempDataIndex += recievedData;
        }

        //ROS_INFO("Total with recieved ");
        //printArray(tempData,tempDataIndex);

        tempDataIndex = findMessageStart(tempData,tempDataIndex);
    }
    //ROS_INFO("tempDataIndex1:  %d", tempDataIndex);
    if(tempDataIndex > 4 && tempData[0] == startChar) // ilk checksum için yeterli data varsa ve ilk karakter doğruysa
    {
        int data_len = firstValidator(tempData);
        //printf("Data length:%d\n",data_len);
        //printArray(tempData,tempDataIndex);
        if(data_len == 0 && tempDataIndex >= 4)
        {
            //ROS_INFO("Data is VALID! 1 checksum\n");
            //executeCommand(tempData);
            tempData[0] = '0'; // ilk mesajı boz
            tempDataIndex = findMessageStart(tempData,tempDataIndex);
        }else if(data_len > 0 && tempDataIndex >= data_len+6)
        {
            if(secondValidator(tempData, data_len))
            {
                //ROS_INFO("Data is VALID! 2 checksum\n");
                executeCommand(tempData);
            }
            tempData[0] = '0'; // ilk mesajı boz
            tempDataIndex = findMessageStart(tempData,tempDataIndex);
            //ROS_INFO("tempDataIndex2:  %d", tempDataIndex);
        }else if(data_len == -1){
            // Eğer checksum doğru değilse
            tempData[0] = '0'; // ilk mesajı boz
            tempDataIndex = findMessageStart(tempData,tempDataIndex);
        }
    }
}


void sendParam(char side, char param, double value)
{
  // value 0.0001 gibi bi değer
  uint8_t send_array[20];
  value *= 10000;

  unsigned int pid = (int)value;

  ROS_INFO("New value set");

  send_array[0] = '$';
  send_array[1] = e_stop;
  send_array[2] = param;
  send_array[3] = 5;
  send_array[4] = checksum_check_array(send_array,4);
  send_array[5] = side;
  send_array[6] = (pid >> 24);
  send_array[7] = (pid >> 16);
  send_array[8] = (pid >> 8);
  send_array[9] =  pid;
  send_array[10] = checksum_check_array(send_array,10);

  //string out(sendArray, 10);
  //printArray(send_array,19);
  //mrp_serial.write(out);
  int ret =RS232_SendBuf(port_nr,send_array,11);
  ret =RS232_SendBuf(port_nr,send_array,11);
  ret =RS232_SendBuf(port_nr,send_array,11);
  ret =RS232_SendBuf(port_nr,send_array,11);
  ret =RS232_SendBuf(port_nr,send_array,11);
  ret =RS232_SendBuf(port_nr,send_array,11);
  ret =RS232_SendBuf(port_nr,send_array,11);
  ret =RS232_SendBuf(port_nr,send_array,11);
  ret =RS232_SendBuf(port_nr,send_array,11);
  ret =RS232_SendBuf(port_nr,send_array,11);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "powerboard_messenger");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);

  speeds_sub   = n.subscribe<std_msgs::Int32MultiArray>("setSpeeds", 1, speedsCallback);
  resetenc_sub = n.subscribe<std_msgs::Empty>("reset_encoders", 1, &resetencCallback);
  estop_sub = n.subscribe<std_msgs::Bool>("estop", 1, &estopCallback);
  clear_estop_sub = n.subscribe<std_msgs::Bool>("clear_estop", 1, &clearestopCallback);

  position_pub = n.advertise<std_msgs::Int32MultiArray>("encoder_positions",100);
  speed_left_pub = n.advertise<std_msgs::Int32>("speeds_left_qpps",100);
  speed_right_pub = n.advertise<std_msgs::Int32>("speeds_right_qpps",100);
  set_speed_left_pub = n.advertise<std_msgs::Int32>("set_speeds_left_qpps", 100);
  set_speed_right_pub = n.advertise<std_msgs::Int32>("set_speeds_right_qpps", 100);
  bumpers_pub = n.advertise<std_msgs::Int32MultiArray>("bumpers", 100);

  
  if(RS232_OpenComport(port_nr,baudrate,mode))
  {
    ROS_ERROR("Can't open comport.\n");
    return -1;
  }
  

  n.getParam("Kp",Kp);
  n.getParam("Ki",Ki);
  n.getParam("Kd",Kd);
  //n.getParam("Kol",Kol);

  ROS_INFO("Parameters : %f,%f,%f\n",Kp,Ki,Kd);
  int i = 0;
  sendResetEncoders();
int x = 0;

  while (ros::ok())
    { 
    //setSpeeds();
    serialReadProtocol();
    ros::spinOnce();
    loop_rate.sleep();

    ros::Time speed_current_time;
    speed_current_time = ros::Time::now();
    if((speed_current_time - speed_last_time).toSec() > 1 && x > 50)
    {
      speeds[0] = 0;
      speeds[1] = 0;
      setSpeeds();
	x = 0;
    }
	x++;

    if(n.hasParam("Kp"))
    {
      double buf;
      n.getParam("Kp",buf);
      if(buf != Kp)
      {
        Kp = buf;
        sendParam('R','P' , Kp);
        sendParam('L','P' , Kp);
      }
    }

    if(n.hasParam("Ki"))
    {
      double buf;
      n.getParam("Ki",buf);
      if(buf != Ki)
      {
        Ki = buf;
        sendParam('R','I' , Ki);
        sendParam('L','I' , Ki);
      }
    }
    if(n.hasParam("Kd"))
    {
      double buf;
      n.getParam("Kd",buf);
      if(buf != Kd)
      {
        Kd = buf;
        sendParam('R','D' , Kd);
        sendParam('L','D' , Kd);
      }
    }
  }
}
