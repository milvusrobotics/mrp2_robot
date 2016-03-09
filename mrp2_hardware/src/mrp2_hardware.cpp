// ROS
#include <ros/ros.h>

// ros_control
#include <controller_manager/controller_manager.h>

#include "mrp2_hardware/mrp2_hardware.h"

typedef boost::chrono::steady_clock time_source;

static const double BILLION = 1000000000.0;

int main(int argc, char **argv)
{

	// Settings
	double cycle_time_error_threshold_ = 0.02;

	// Timing
	ros::Timer non_realtime_loop_;
	ros::Duration elapsed_time_;
	double loop_hz_ = 50;
	struct timespec last_time_;
	struct timespec current_time_;
	struct timespec read_time_, write_time_, update_time_;
	ros::Duration elapsed_time_read_, elapsed_time_write_, elapsed_time_update_;

	ros::init(argc, argv, "mrp2_hardware");
	ros::NodeHandle nh;

	MRP2HW robot;
 
	

	// Start timer
	ros::Duration desired_update_freq_ = ros::Duration(1 / loop_hz_);

	controller_manager::ControllerManager cm(&robot, nh);

	ros::Rate rate(loop_hz_);
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Get current time for use with first update
	clock_gettime(CLOCK_MONOTONIC, &last_time_);
	
	while(ros::ok())
	{

		clock_gettime(CLOCK_MONOTONIC, &current_time_);
		elapsed_time_ =
		  ros::Duration(current_time_.tv_sec - last_time_.tv_sec + (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);

		elapsed_time_read_ = ros::Duration(read_time_.tv_sec - last_time_.tv_sec + (read_time_.tv_nsec - last_time_.tv_nsec) / BILLION);
		elapsed_time_write_ = ros::Duration(write_time_.tv_sec - last_time_.tv_sec + (write_time_.tv_nsec - last_time_.tv_nsec) / BILLION);
		elapsed_time_update_ = ros::Duration(update_time_.tv_sec - last_time_.tv_sec + (update_time_.tv_nsec - last_time_.tv_nsec) / BILLION);

		last_time_ = current_time_;
		const double cycle_time_error = (elapsed_time_ - desired_update_freq_).toSec();
		if (cycle_time_error > cycle_time_error_threshold_)
		{
			ROS_WARN_STREAM("Cycle time exceeded error threshold by: "
		                                 << cycle_time_error << ", cycle time: " << elapsed_time_
		                                 << ", threshold: " << cycle_time_error_threshold_ << ", read time: " << elapsed_time_read_
		                                 << ", update time: " << elapsed_time_update_
		                                 << ", write time: " << elapsed_time_write_);
		}

		robot.read();
		clock_gettime(CLOCK_MONOTONIC, &read_time_);
		if(robot.estop_release)
      	{
        	cm.update(ros::Time::now(), elapsed_time_, true);
        	ROS_WARN_STREAM("E-stop is released.");
        	robot.estop_release = false;
      	}else{
      		cm.update(ros::Time::now(), elapsed_time_);
			//ROS_WARN("robot time: %f", robot.getTime().toSec());
      	}
      	clock_gettime(CLOCK_MONOTONIC, &update_time_);
		robot.write();
		clock_gettime(CLOCK_MONOTONIC, &write_time_);
		rate.sleep();
	}
	spinner.stop();

	return 0;
}
