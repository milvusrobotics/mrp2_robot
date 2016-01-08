// ROS
#include <ros/ros.h>

// ros_control
#include <controller_manager/controller_manager.h>

#include "mrp2_hardware/mrp2_hardware.h"


int main(int argc, char **argv)
{

	ros::init(argc, argv, "mrp2_hardware");
	ros::NodeHandle nh;

	MRP2HW robot;

	controller_manager::ControllerManager cm(&robot, nh);

	ros::Rate rate(1.0 / robot.getPeriod().toSec());
	ros::AsyncSpinner spinner(1);
	spinner.start();

	while(ros::ok())
	{
		robot.read();
		
		if(robot.estop_release)
      	{
        	cm.update(robot.getTime(), robot.getPeriod(),true);
        	ROS_WARN_STREAM("E-STOP is relased.");
        	robot.estop_release = false;
      	}else{
      		cm.update(robot.getTime(), robot.getPeriod());
      	}

		robot.write();
		rate.sleep();
	}
	spinner.stop();

	return 0;
}
