// ROS
 #include <ros/ros.h>
 #include <std_msgs/Float64.h>
 #include <std_msgs/Float32.h>
 #include <std_srvs/Empty.h>
 #include <std_msgs/Empty.h>
 #include <std_msgs/Int32.h>
 #include <std_msgs/MultiArrayLayout.h>
 #include <std_msgs/MultiArrayDimension.h>
 #include <std_msgs/Int32MultiArray.h>
 #include <std_msgs/Float32MultiArray.h>
 #include <std_msgs/Bool.h>

 #include <nav_msgs/Odometry.h>
 #include <sensor_msgs/Range.h>

 // ros_control
 #include <controller_manager/controller_manager.h>
 #include <hardware_interface/joint_command_interface.h>
 #include <hardware_interface/joint_state_interface.h>
 #include <hardware_interface/imu_sensor_interface.h>
 #include <hardware_interface/robot_hw.h>
 #include <realtime_tools/realtime_buffer.h>

 // NaN
 #include <limits>

 // ostringstream
 #include <sstream>

 #include <mrp2_hardware/mrp2_serial.h>

 #include <dynamic_reconfigure/server.h>
 #include <mrp2_hardware/ParametersConfig.h>

 class MRP2HW : public hardware_interface::RobotHW
 {
 public:
   MRP2HW()
   : running_(true)
   , start_srv_(nh_.advertiseService("start", &MRP2HW::start_callback, this))
   , stop_srv_(nh_.advertiseService("stop", &MRP2HW::stop_callback, this))
   {

      pos_[0] = 0.0; pos_[1] = 0.0;
      vel_[0] = 0.0; vel_[1] = 0.0;
      eff_[0] = 0.0; eff_[1] = 0.0;
      cmd_[0] = 0.0; cmd_[1] = 0.0;

      hardware_interface::JointStateHandle state_handle_1("wheel_right_joint", &pos_[0], &vel_[0], &eff_[0]);
      jnt_state_interface_.registerHandle(state_handle_1);

      hardware_interface::JointStateHandle state_handle_2("wheel_left_joint", &pos_[1], &vel_[1], &eff_[1]);
      jnt_state_interface_.registerHandle(state_handle_2);

      registerInterface(&jnt_state_interface_);

      hardware_interface::JointHandle vel_handle_1(jnt_state_interface_.getHandle("wheel_right_joint"), &cmd_[0]);
      jnt_vel_interface_.registerHandle(vel_handle_1);

      hardware_interface::JointHandle vel_handle_2(jnt_state_interface_.getHandle("wheel_left_joint"), &cmd_[1]);
      jnt_vel_interface_.registerHandle(vel_handle_2);

      registerInterface(&jnt_vel_interface_);

      /*hardware_interface::ImuSensorHandle::Data data;
      data.name = "base_imu";
      data.frame_id = "base_imu_link";
      hardware_interface::ImuSensorHandle imu_handle(data);
      imu_sens_interface_.registerHandle(imu_handle);*/

      sym_tuning = false;
      publish_pos = false;
      publish_ref = false;
      publish_feed = false;

      //registerInterface(&imu_sens_interface_);
      //server = new dynamic_reconfigure::Server<mrp2_hardware::ParametersConfig>(dynamic_reconfigure_mutex_, nh_);
      f = boost::bind(&MRP2HW::callback, this, _1, _2);
      server.setCallback(f);

      pos_reset_sub      = nh_.subscribe<std_msgs::Empty>("positions_reset", 1, &MRP2HW::positions_reset_callback, this);
      estop_clear_sub    = nh_.subscribe<std_msgs::Empty>("estop_clear", 1, &MRP2HW::estop_clear_callback, this);

      sep_cmd_vel_pub    = nh_.advertise<std_msgs::Float32MultiArray>("motor_controller/sep_cmd_vel", 100);

      bumpers_pub        = nh_.advertise<std_msgs::Int32MultiArray>("bumpers", 100);
      estop_pub          = nh_.advertise<std_msgs::Bool>("estop", 100);
      estop_btn_pub      = nh_.advertise<std_msgs::Bool>("estop_btn", 100);
      motor_stall_l_pub  = nh_.advertise<std_msgs::Bool>("hw_monitor/diagnostics/motor_stall_l", 100);
      motor_stall_r_pub  = nh_.advertise<std_msgs::Bool>("hw_monitor/diagnostics/motor_stall_r", 100);
      batt_low_pub       = nh_.advertise<std_msgs::Bool>("hw_monitor/diagnostics/batt_low", 100);
      batt_high_pub      = nh_.advertise<std_msgs::Bool>("hw_monitor/diagnostics/batt_high", 100);
      controller_pub     = nh_.advertise<std_msgs::Bool>("hw_monitor/diagnostics/controller", 100);
      aux_lights_pub     = nh_.advertise<std_msgs::Bool>("hw_monitor/diagnostics/aux_lights", 100);


      robot_serial = new MRP2_Serial("/dev/mrp2_powerboard", 921600, "8N1");
      //robot_serial = new MRP2_Serial(0x0483, 0x5740, 0x81, 0x01);
      robot_serial->update();

      bumper_states.resize(4);
      bumper_states = robot_serial->get_bumpers();

      // Init first parameter values
      std::vector<int> imax ;


      /*if (imax.size() < 1) {
        ROS_ERROR("MRP2_Hardware: Connection error to Powerboard. Aborting...");
        exit(0);
      }*/


      init_config.BUMPER_ESTOP = robot_serial->get_bumper_estop(true);

      //robot_serial->set_bumper_estop(0);

      //boost::recursive_mutex::scoped_lock dyn_reconf_lock(dynamic_reconfigure_mutex_);
      server.updateConfig(init_config);
      //dyn_reconf_lock.unlock();

      estop_state = robot_serial->get_estop(true);
      estop_release = false;
      robot_serial->reset_positions();
      robot_serial->clear_diag(0); // TODO How to implement diag_t ?

      current_time = ros::Time::now();
      last_time = ros::Time::now();
      pos_left = 0;
      pos_right = 0;

   }

   ros::Time getTime() const {return ros::Time::now();}
   ros::Duration getPeriod() const {return ros::Duration(0.01);}

   bool estop_release;

   void read()
   {
      std_msgs::Int32MultiArray array32;
      std_msgs::Bool b;
      std_msgs::Int32 i;
      std_msgs::Float32 f;

      current_time = ros::Time::now();

      bumper_states = robot_serial->get_bumpers(true);


      long last_pos_l=0, now_pos_l=0;
      long last_pos_r=0, now_pos_r=0;

      //speeds = robot_serial->get_speeds(true);

      //now_pos_l = robot_serial->get_position_l(true);
      //now_pos_r = robot_serial->get_position_r(true);

      //double pos_l = (now_pos_l/(21600.0))*M_PI*2; // 2652: 11pulse x 4quadrature x 51gearratio
      //double pos_r = (now_pos_r/(21600.0))*M_PI*2;

      //pos_[0] = pos_r;
      //pos_[1] = pos_l;

      double dt = (current_time - last_time).toSec();
      last_time = current_time;

      double qpps_l, qpps_r;
      qpps_l = robot_serial->get_speed_l(true);
      qpps_r = robot_serial->get_speed_r(true);
      //qpps_l = speeds[0];
      //qpps_r = speeds[1];
      double speed_l = (qpps_l/(21600.0))*2*M_PI; // 21600: 360pulse x 4quadrature x 15gearratio
      double speed_r = (qpps_r/(21600.0))*2*M_PI; // 

      vel_[0] = speed_r;
      vel_[1] = speed_l;

      pos_left += speed_l*dt;
      pos_right += speed_r*dt;

      //ROS_INFO("sp_l: %f, sp_r: %f, pos_l: %f, pos_r: %f, n_pos_l: %ld, n_pos_r: %ld\n", speed_l, speed_r, pos_left*2244/(2*M_PI), pos_right*2244/(2*M_PI), now_pos_l, now_pos_r);

      pos_[0] = pos_right;
      pos_[1] = pos_left;

      array32.data.clear();
      array32.data.push_back(bumper_states[2]);
      array32.data.push_back(bumper_states[3]);
      array32.data.push_back(bumper_states[0]);
      array32.data.push_back(bumper_states[1]);
      bumpers_pub.publish(array32);

      estop_state = robot_serial->get_estop(false);
      if(estop_state == true)
      {
        estop_state = robot_serial->get_estop(true);
        if(estop_state == false)
          estop_release = true;
      }else{
        estop_state = robot_serial->get_estop(true);
      }

      b.data = estop_state;
      estop_pub.publish(b);

      bool estop_button = robot_serial->get_estop_button(true);
      b.data = estop_button;
      estop_btn_pub.publish(b);

      /*robot_serial->update_diag();
      b.data = robot_serial->get_diag(DIAG_MOTOR_STALL_L);
      motor_stall_l_pub.publish(b);
      b.data = robot_serial->get_diag(DIAG_MOTOR_STALL_R);
      motor_stall_r_pub.publish(b);
      b.data = robot_serial->get_diag(DIAG_BATT_LOW);
      batt_low_pub.publish(b);
      b.data = robot_serial->get_diag(DIAG_BATT_HIGH);
      batt_high_pub.publish(b);
      b.data = robot_serial->get_diag(DIAG_MOTOR_DRVR_ERR);
      controller_pub.publish(b);
      b.data = robot_serial->get_diag(DIAG_AUX_LIGHTS_ERR);
      aux_lights_pub.publish(b);
      i.data = robot_serial->get_batt_volt(true);
      batt_volt_pub.publish(i);
      i.data = robot_serial->get_batt_current(true);
      batt_current_pub.publish(i);
      i.data = robot_serial->get_batt_soc(true);
      batt_soc_pub.publish(i);*/

   }

   void write()
   {

      std_msgs::Float32MultiArray spd_array;
      spd_array.data.clear();
      
      /*if(publish_feed)
      {
        speeds = robot_serial->get_speeds();
        speed.data = speeds[0];
        feedback_l_pub.publish(speed);
        speed.data = speeds[1];
        feedback_r_pub.publish(speed);
      }

      if(publish_ref)
      {
        speed.data = right_vel;
        ref_r_pub.publish(speed);
        speed.data = left_vel;
        ref_l_pub.publish(speed);
      }*/

      if(estop_state)
      {
        spd_array.data.push_back(0);
        spd_array.data.push_back(0);
      }else{
        spd_array.data.push_back(cmd_[0]*60*15/(2*M_PI)); // 15: Motor Gear Ratio
        spd_array.data.push_back(cmd_[1]*60*15/(2*M_PI)); // Unit is RPM, cmd_[0] unit is rad/s
      }

      sep_cmd_vel_pub.publish(spd_array);

   }

   bool start_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
   {
     running_ = true;
     return true;
   }

   bool stop_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
   {
     running_ = false;
     return true;
   }

   void positions_reset_callback(const std_msgs::Empty::ConstPtr& msg)
   {
      ROS_WARN("Positions reset.");
      robot_serial->reset_positions();
   }

   void estop_clear_callback(const std_msgs::Empty::ConstPtr& msg)
   {
      ROS_WARN("Estop is cleared.");
      robot_serial->set_estop(0);
   }

   void callback(mrp2_hardware::ParametersConfig &config, uint32_t level)
   {
      switch (level)
      {
        case MRP2_Serial::setBUMPER_ESTOP:
          robot_serial->set_bumper_estop(config.BUMPER_ESTOP);
          break;
      }

      //ROS_INFO("got: %F and level: %d",config.P_L, level);
   }


 private:
    hardware_interface::JointStateInterface    jnt_state_interface_;
    hardware_interface::VelocityJointInterface jnt_vel_interface_;
    hardware_interface::ImuSensorInterface     imu_sens_interface_;
    double cmd_[2];
    double pos_[2];
    double vel_[2];
    double eff_[2];
    bool running_;

    bool sym_tuning, publish_pos, publish_ref, publish_feed;
    bool estop_state;

    double pos_left, pos_right;


    MRP2_Serial *robot_serial;
    std::vector<int> bumper_states, speeds, diags;

    ros::NodeHandle nh_;
    ros::ServiceServer start_srv_;
    ros::ServiceServer stop_srv_;

    ros::Publisher bumpers_pub;

    ros::Publisher estop_pub;
    ros::Publisher estop_btn_pub;
    ros::Publisher motor_stall_l_pub;
    ros::Publisher motor_stall_r_pub;
    ros::Publisher batt_low_pub;
    ros::Publisher batt_high_pub;
    ros::Publisher controller_pub;
    ros::Publisher aux_lights_pub;

    ros::Publisher sep_cmd_vel_pub;

    ros::Subscriber pos_reset_sub;
    ros::Subscriber estop_clear_sub;

    ros::Time current_time, last_time;

    dynamic_reconfigure::Server<mrp2_hardware::ParametersConfig> server;
    dynamic_reconfigure::Server<mrp2_hardware::ParametersConfig>::CallbackType f;
    boost::recursive_mutex dynamic_reconfigure_mutex_;
    boost::mutex connect_mutex_;

    mrp2_hardware::ParametersConfig init_config;

    double _ftod(float fValue)
    {
        char cz_dummy[30];
        sprintf(cz_dummy,"%9.5f",fValue);
        double dValue = strtod(cz_dummy,NULL);
        return dValue;
    }

 };
