#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <sstream>
#include <serial.h>

#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "mrp2_display/gpio.h"

ros::Subscriber bumpers_sub;
ros::Subscriber battery_volt_sub;
ros::Subscriber estop_btn_sub;
ros::Subscriber estop_sub;

ros::Publisher inputs_pub;

double last_volt = 0.0;
bool last_fr = false;
bool last_fl = false;
bool last_rr = false;
bool last_rl = false;

bool last_estop_btn = false;
bool last_estop     = false;

// Set-reset GPIO output pins
bool gpio(mrp2_display::gpio::Request &req, mrp2_display::gpio::Response &res){

	sendCmd(GPIO_SET, (char)req.pin_number, (char)req.pin_state);

	res.status = true;
	return true;
}

// Plays beep sound
void beep(void)
{
	sendCmd(BEEP, 0, 0);
}

// Send request to read input pins and publish data
void analog_read()
{
	portFlush();
	sendCmd(READ_ANALOG, 0, 0);
	lockSerial(true);

	usleep(100000);

	if(portDataAvail() == 10){

		std_msgs::Int32MultiArray array;
		int i, an;
		int indata[10];

		array.data.clear();

		for(i = 0; i < 10; i++){
			indata[i] = portGetchar();
		}

		an = indata[0] << 8;
		an += indata[1];
		array.data.push_back(an);

		an = indata[2] << 8;
		an += indata[3];
		array.data.push_back(an);

		an = indata[4] << 8;
		an += indata[5];
		array.data.push_back(an);

		an = indata[6];
		array.data.push_back(an);

		an = indata[7] << 8;
		an += indata[8];
		array.data.push_back(an);

		an = indata[9];
		array.data.push_back(an);

		inputs_pub.publish(array);
	}
	lockSerial(false);
}

// Changes e-stop button led on the screen
void estopBtnCallback(const std_msgs::Bool::ConstPtr& stat)
{
	if(last_estop_btn != stat->data){
		last_estop_btn = stat->data;
		sendCmd(ESTOP_BTN, last_estop_btn, 0);
	}
}

// Shows estop string on screen
void estopCallback(const std_msgs::Bool::ConstPtr& stat)
{
	if(last_estop == false){
		if(last_estop != stat->data){
			last_estop = stat->data;
			sendCmd(ESTOP, 0, 0);
		}
	}else{
		if(last_estop != stat->data){
			last_estop = stat->data;
		}
	}
	
}

// Sets battery voltage
void batteryVoltCallback(const std_msgs::Float32::ConstPtr& volt)
{
	if(last_volt != volt->data){
		last_volt = volt->data;
		uint16_t send = (uint16_t)(last_volt * 10 );
		if(last_estop == false)
			sendCmd(BATTERY, (uint8_t)(send >> 8), (uint8_t)send);
	}
}

// Changes status of bumper indicators 
void bumpersCallback(const std_msgs::Int32MultiArray::ConstPtr& bumpers)
{
	uint8_t fl = bumpers->data[3];
	uint8_t fr = bumpers->data[2];
	uint8_t rl = bumpers->data[1];
	uint8_t rr = bumpers->data[0];

	if(last_fl != fl){
		last_fl = fl;
		sendCmd(BUMPER_FL, fl, 0);
		usleep(100000);
		
		if(fl == 1){
			beep();
		}		
	}

	if(last_fr != fr){
		last_fr = fr;
		sendCmd(BUMPER_FR, fr, 0);
		usleep(100000);

		if(fr == 1){
		beep();
		}	
	}

	if(last_rl != rl){
		last_rl = rl;
		sendCmd(BUMPER_RL, rl, 0);
		usleep(100000);

		if(rl == 1){
		beep();
		}	
	}

	if(last_rr != rr){
		last_rr = rr;
		sendCmd(BUMPER_RR, rr, 0);
		usleep(100000);

		if(rr == 1){
		beep();
		}
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "mrp2_display");

	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	std::string port;
  	ros::param::param<std::string>("~port", port, "/dev/ttyS1");

	// Serial port setup
	printf("Starting display at port: %s \n", port.c_str());
	if (portOpen(const_cast<char*>(port.c_str()), 57600) < 0)
	{
		fprintf (stderr, "rgb: Can't initialise Display: %s\n", strerror (errno)) ;
		return 1 ;
	}

  	bumpers_sub = n.subscribe<std_msgs::Int32MultiArray>("bumpers", 1, &bumpersCallback);
  	battery_volt_sub = n.subscribe<std_msgs::Float32>("/batt_volt", 1, &batteryVoltCallback);
  	estop_btn_sub = n.subscribe<std_msgs::Bool>("/estop_btn", 1, &estopBtnCallback);
  	estop_sub = n.subscribe<std_msgs::Bool>("/estop", 1, &estopCallback);

  	inputs_pub = n.advertise<std_msgs::Int32MultiArray>("panel_inputs", 1);
  	ros::ServiceServer service = n.advertiseService("/panel_outputs/gpio", gpio);

  	int i = 0;

	while (ros::ok())
  	{
  		i++;
  		if(i == 3)
  		{
  			analog_read();
  			i = 0;
  		}
  		
		ros::spinOnce();
    	loop_rate.sleep();	
	}
	return 0 ;
}

