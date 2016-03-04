/*
 * display.cpp:
 *
 * Milvus Technology, July 2015
 ***********************************************************************
 */
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <sstream>
#include <geniePi.h>

#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "mrp2_display/gpio.h"

ros::Subscriber bumpers_sub;
ros::Subscriber battery_voltage_sub;
ros::Subscriber battery_soc_sub;

ros::Publisher inputs_pub;

/*
 * resetDisplay:
 *********************************************************************************
 */

bool add(mrp2_display::gpio::Request &req, mrp2_display::gpio::Response &res){

	genieWriteObj(GENIE_OBJ_PINOUTPUT, req.pin_number, req.pin_state);
	res.status = true;
	return true;
}

static void resetDisplay (void)
{
  // setLEDs (0, 0, 0) ;

  // genieWriteObj (GENIE_OBJ_SLIDER, 0, 0) ;
  // genieWriteObj (GENIE_OBJ_SLIDER, 1, 0) ;
  // genieWriteObj (GENIE_OBJ_SLIDER, 2, 0) ;

  // genieWriteObj (GENIE_OBJ_GAUGE,         0, 0) ;
  // genieWriteObj (GENIE_OBJ_ANGULAR_METER, 0, 0) ;
  // genieWriteObj (GENIE_OBJ_COOL_GAUGE,    0, 0) ;
}

/*
 * handleGenieEvent:
 *	Take a reply off the display and call the appropriate handler for it.
 *********************************************************************************
 */

void handleGenieEvent(struct genieReplyStruct *reply)
{
  if (reply->cmd != GENIE_REPORT_EVENT)
  {
    printf ("Invalid event from the display: 0x%02X\r\n", reply->cmd) ;
    return ;
  }

  if (reply->object == GENIE_OBJ_WINBUTTON)
    resetDisplay();
  else
    printf ("Unhandled Event: object: %2d, index: %d data: %d [%02X %02X %04X]\r\n",
      reply->object, reply->index, reply->data, reply->object, reply->index, reply->data) ;
}


void batteryVoltageCallback(const std_msgs::Int32::ConstPtr& voltage)
{
	char buf[32];
	std_msgs::String msg;

	double d = (double)(voltage->data)/1000;
	std::stringstream ss;
	ss << d;
	ss.precision(2);
	msg.data = ss.str();

 	sprintf(buf, "%s ", msg.data.c_str()) ;

 	genieWriteStr(0, buf);
}

void batterySOCCallback(const std_msgs::Int32::ConstPtr& soc)
{
	char buf[32];
	std_msgs::String msg;

	std::stringstream ss;
	ss << soc->data;
	msg.data = ss.str();


 	sprintf(buf, "%s", msg.data.c_str()) ;

 	genieWriteStr(1, buf);	
}

void bumpersCallback(const std_msgs::Int32MultiArray::ConstPtr& bumpers)
{

	int fl = bumpers->data[3];
	int fr = bumpers->data[2];
	int rl = bumpers->data[1];
	int rr = bumpers->data[0];
/*
	if (fl)
	{
		genieWriteObj(GENIE_OBJ_USER_LED, 0, 1);
	} 
	else 
	{
		genieWriteObj(GENIE_OBJ_USER_LED, 0, 0);
	}

	if (fr)
	{
		genieWriteObj(GENIE_OBJ_USER_LED, 1, 1);
	} 
	else 
	{
		genieWriteObj(GENIE_OBJ_USER_LED, 1, 0);
	}

	if (rl)
	{
		genieWriteObj(GENIE_OBJ_USER_LED, 2, 1);
	} 
	else 
	{
		genieWriteObj(GENIE_OBJ_USER_LED, 2, 0);
	}

	if (rr)
	{
		genieWriteObj(GENIE_OBJ_USER_LED, 3, 1);
	} 
	else 
	{
		genieWriteObj(GENIE_OBJ_USER_LED, 3, 0);
	}

	if (fl || fr || rl || rr)
	{
		genieWriteObj(GENIE_OBJ_SOUND, 1, 100);
		genieWriteObj(GENIE_OBJ_SOUND, 0, 1);
	} 
*/
}

void pollInputs(){
	std_msgs::Int32MultiArray array;

	array.data.clear();

	for(int i = 0; i<8; i++)
		array.data.push_back (genieReadObj(GENIE_OBJ_PININPUT, i));

	inputs_pub.publish(array);
}


/*
 *********************************************************************************
 * main:
 *********************************************************************************
 */

int main(int argc, char **argv)
{
	struct genieReplyStruct reply;

	ros::init(argc, argv, "mrp2_display");

	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	printf ("\n\n\n\n") ;
	printf ("MRP2 Display Node\n") ;
	printf ("=============================\n") ;
	std::string port;
  	ros::param::param<std::string>("~port", port, "/dev/ttyS0");
	// Genie display setup
	printf("Starting display at port: %s \n", port.c_str());
	if (genieSetup(const_cast<char*>(port.c_str()), 115200) < 0)
	{
		fprintf (stderr, "rgb: Can't initialise Genie Display: %s\n", strerror (errno)) ;
		return 1 ;
	}

	// Make sure we're displaying form 0 (the first one!)
	// and set the slider, LEDs, Gauges, etc. to zero.
	// genieWriteObj(GENIE_OBJ_FORM, 0, 0);
	// resetDisplay();

  	//bumpers_sub = n.subscribe<std_msgs::Int32MultiArray>("bumpers", 1, &bumpersCallback);
  	battery_voltage_sub = n.subscribe<std_msgs::Int32>("/hw_monitor/batt_volt", 1, &batteryVoltageCallback);
  	battery_soc_sub = n.subscribe<std_msgs::Int32>("/hw_monitor/batt_soc", 1, &batterySOCCallback);

  	inputs_pub = n.advertise<std_msgs::Int32MultiArray>("panel_inputs", 10);

  	genieSetPin(PIN_AN, 0);
  	genieSetPin(PIN_AN, 1);

	while (ros::ok())
  	{
  		
		if (genieReplyAvail())
		{
			genieGetReply(&reply) ;
			handleGenieEvent(&reply) ;
		}

		pollInputs();
		
		ros::spinOnce();

    	loop_rate.sleep();
	}
	return 0 ;
}

