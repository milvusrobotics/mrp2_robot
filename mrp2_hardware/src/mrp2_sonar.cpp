 #include <ros/ros.h>
 #include <sensor_msgs/Range.h>
 #include <mrp2_hardware/mrp2_serial.h>

ros::Publisher sonar1_pub;
ros::Publisher sonar2_pub;
ros::Publisher sonar3_pub;
ros::Publisher sonar4_pub;
ros::Publisher sonar5_pub;
ros::Publisher sonar6_pub;
ros::Publisher sonar7_pub;

MRP2_Serial *sonar_serial;

int use_sonar, sonar_baud;
std::string sonar_port;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "mrp2_sonar");
	ros::NodeHandle nh_;

	ros::param::param<std::string>("~sonar_port", sonar_port, "/dev/mrp2_ftdi_MRP2SNR001");
    ros::param::param<int32_t>("~sonar_baud", sonar_baud, 115200);

	sonar1_pub = nh_.advertise<sensor_msgs::Range>("sonar_1", 100);
    sonar2_pub = nh_.advertise<sensor_msgs::Range>("sonar_2", 100);
    sonar3_pub = nh_.advertise<sensor_msgs::Range>("sonar_3", 100);
    sonar4_pub = nh_.advertise<sensor_msgs::Range>("sonar_4", 100);
    sonar5_pub = nh_.advertise<sensor_msgs::Range>("sonar_5", 100);
    sonar6_pub = nh_.advertise<sensor_msgs::Range>("sonar_6", 100);
    sonar7_pub = nh_.advertise<sensor_msgs::Range>("sonar_7", 100);

	sonar_serial = new MRP2_Serial(sonar_port, sonar_baud, "8N1", false);
	sonar_serial->set_read_timeout(0.05);

    ros::Rate loop_rate(20);

    while(ros::ok())
    {
		std::vector<int> sonar_vals;

    	sonar_vals.reserve(20);
    	sonar_vals.clear();

   		sonar_vals = sonar_serial->get_sonars(true);
		
		sensor_msgs::Range sonar;

        sonar.radiation_type = sensor_msgs::Range::ULTRASOUND;
        sonar.header.stamp = ros::Time::now();
        sonar.field_of_view = 0.6108652;
        sonar.max_range = 4;
        sonar.min_range = 0;
        
        sonar.header.frame_id = "base_sonar_1_link";
        sonar.range = ((double)sonar_vals[0] / 100);
        sonar1_pub.publish(sonar);

        sonar.header.frame_id = "base_sonar_2_link";
        sonar.range = ((double)sonar_vals[1] / 100);
        sonar2_pub.publish(sonar);

        sonar.header.frame_id = "base_sonar_3_link";
        sonar.range = ((double)sonar_vals[2] / 100);
        sonar3_pub.publish(sonar);

        sonar.header.frame_id = "base_sonar_4_link";
        sonar.range = ((double)sonar_vals[3] / 100);
        sonar4_pub.publish(sonar);

        sonar.header.frame_id = "base_sonar_5_link";
        sonar.range = ((double)sonar_vals[4] / 100);
        sonar5_pub.publish(sonar);

        sonar.header.frame_id = "base_sonar_6_link";
        sonar.range = ((double)sonar_vals[5] / 100);
        sonar6_pub.publish(sonar);

        sonar.header.frame_id = "base_sonar_7_link";
        sonar.range = ((double)sonar_vals[6] / 100);
        sonar7_pub.publish(sonar);

    	ros::spinOnce();
    	loop_rate.sleep();
    }
}