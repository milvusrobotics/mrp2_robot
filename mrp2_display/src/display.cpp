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
#include "mrp2_display/gpio.h"

ros::Subscriber bumpers_sub;
ros::Subscriber battery_soc_sub;

ros::Publisher inputs_pub;

int last_soc = 0;
bool last_fr = false;
bool last_fl = false;
bool last_rr = false;
bool last_rl = false;

bool gpio(mrp2_display::gpio::Request &req, mrp2_display::gpio::Response &res){

	portPutchar(103);
	portPutchar(req.pin_number);
	portPutchar(req.pin_state);

	res.status = true;
	return true;
}

void beep(void)
{
	portPutchar(104);
	portPutchar(0);
	portPutchar(0);
}

void analog_read()
{
	portFlush();
	portPutchar(102);
	portPutchar(0);
	portPutchar(0);	

	usleep(100000);

	if(portDataAvail() == 10){

		std_msgs::Int32MultiArray array;
		int i, an;
		int indata[10];

		array.data.clear();

		
		for(i = 0; i < 10; i++){
			indata[i] = portGetchar();
			usleep(1000);
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
}


void batterySOCCallback(const std_msgs::Int32::ConstPtr& soc)
{
	if(last_soc != soc->data){
		last_soc = soc->data;
	 	portPutchar(101);
		portPutchar(last_soc);
		portPutchar(0);	
	}
}

void bumpersCallback(const std_msgs::Int32MultiArray::ConstPtr& bumpers)
{
	int fl = bumpers->data[3];
	int fr = bumpers->data[2];
	int rl = bumpers->data[1];
	int rr = bumpers->data[0];

	if(last_fl != fl){
		last_fl = fl;
		portPutchar(97);
		portPutchar(fl);
		portPutchar(0);
		usleep(10000);
		
		if(fl == 1){
			beep();
		}		
	}

	if(last_fr != fr){
		last_fr = fr;
		portPutchar(98);
		portPutchar(fr);
		portPutchar(0);
		usleep(10000);

		if(fr == 1){
		beep();
		}	
	}

	if(last_rl != rl){
		last_rl = rl;
		portPutchar(99);
		portPutchar(rl);
		portPutchar(0);
		usleep(10000);

		if(rl == 1){
		beep();
		}	
	}

	if(last_rr != rr){
		last_rr = rr;
		portPutchar(100);
		portPutchar(rr);
		portPutchar(0);
		usleep(10000);

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

	printf ("\n\n\n\n") ;
	printf ("MRP2 Display Node\n") ;
	printf ("=============================\n") ;

	std::string port;
  	ros::param::param<std::string>("~port", port, "/dev/ttyS0");

	// Serial port setup
	printf("Starting display at port: %s \n", port.c_str());
	if (portOpen(const_cast<char*>(port.c_str()), 57600) < 0)
	{
		fprintf (stderr, "rgb: Can't initialise Display: %s\n", strerror (errno)) ;
		return 1 ;
	}

  	bumpers_sub = n.subscribe<std_msgs::Int32MultiArray>("bumpers", 1, &bumpersCallback);
  	battery_soc_sub = n.subscribe<std_msgs::Int32>("/hw_monitor/batt_soc", 1, &batterySOCCallback);

  	inputs_pub = n.advertise<std_msgs::Int32MultiArray>("panel_inputs", 5);
  	ros::ServiceServer service = n.advertiseService("/panel_outputs/gpio", gpio);

	while (ros::ok())
  	{
		ros::spinOnce();
    	loop_rate.sleep();
    	analog_read();
	}
	return 0 ;
}

