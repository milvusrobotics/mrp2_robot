#ifndef MRP2_SERIAL_H
#define MRP2_SERIAL_H

#include "serial_comm.h"
#include "usb_comm.h"
#include "time.h"
#include <sys/time.h>
#include <string>
#include <stdint.h>
#include <unistd.h>
#include <vector>

using std::string;

typedef enum {
		DIAG_CLEAR_CMD = 0,
		DIAG_MOTOR_STALL_L = 1,
		DIAG_MOTOR_STALL_R = 2,
		DIAG_BATT_LOW = 3,
		DIAG_BATT_HIGH = 4,
		DIAG_MOTOR_DRVR_ERR = 5,
		DIAG_AUX_LIGHTS_ERR = 6
	}diag_t;

class MRP2_Serial
{
	public:
		MRP2_Serial(std::string port_name, uint32_t baudrate = 38400, std::string mode = "8N1", bool simple = true);
		MRP2_Serial(uint16_t vendor_id, uint16_t product_id, int ep_in_addr, int ep_out_addr, bool simple = true);
		virtual ~MRP2_Serial ();
		void set_speeds(int32_t left_speed, int32_t right_speed);
		void set_speed_l(int32_t left_speed);
		void set_speed_r(int32_t right_speed);
		void set_param_pid(char side, char param, float value);
		void set_param_imax(char side, uint32_t value);
		void set_maxspeed_fwd(uint32_t value);
		void set_maxspeed_rev(uint32_t value);
		void set_max_accel(uint32_t value);
		//BATT SET FUNCTIONS WILL BE HERE
		void set_estop(bool value);
		void clear_diag(int diag);
		std::vector<int> get_speeds(bool update=false);
		int get_speed_l(bool update=false);
		int get_speed_r(bool update=false);
		float get_param_pid(char side, char param, bool update=false);
		std::vector<int> get_param_imax(char side, bool update=false);
		int get_maxspeed_fwd(bool update=false);
		int get_maxspeed_rev(bool update=false);
		int get_maxaccel(bool update=false);
		//BATT GET FUNCTIONS WILL BE HERE
		int get_batt_volt(bool update=false);
		int get_batt_current(bool update=false);
		int get_batt_soc(bool update=false);
		std::vector<int> get_positions(bool update=false);
		int get_position_l(bool update=false);
		int get_position_r(bool update=false);
		std::vector<int> get_bumpers(bool update=false);
		void reset_positions();
		void reset_position_l();
		void reset_position_r();
		bool get_estop(bool update=false);
		bool get_diag(int diag);
		void update_diag();
		int get_batt_cell_capacity(bool update=false);
		void set_bumper_estop(bool value);
		bool get_bumper_estop(bool update=false);
		bool get_estop_button(bool update=false);
		std::vector<int> get_sonars(bool update=false);
		bool is_available();
		void set_read_timeout(double timeout);
		double get_read_timeout(void);
				
		void update();

		typedef enum {
		setSPEEDS   = 1,
		setSPEED_L = 2,
		setSPEED_R = 3,
		setPARAM_KP_L = 4,
		setPARAM_KP_R = 5,
		setPARAM_KI_L = 6,
		setPARAM_KI_R = 7,
		setPARAM_KD_L = 8,
		setPARAM_KD_R = 9,
		setPARAM_IMAX_L = 10,
		setPARAM_IMAX_R = 11,
		setMAXSPEED_FWD = 12,
		setMAXSPEED_REV = 13,
		setMAXACCEL = 14,
		setBATT_CELL_V = 15,
		setBATT_PARALLEL_COUNT = 16,
		setBATT_SERIES_COUNT = 17,
		setBATT_CELL_NOMINAL_V = 18,
		setESTOP = 19,
		clearDIAG = 20,
		getSPEEDS = 21,
		getSPEED_L = 22,
		getSPEED_R = 23,
		getPARAM_KP_L = 24,
		getPARAM_KP_R = 25,
		getPARAM_KI_L = 26,
		getPARAM_KI_R = 27,
		getPARAM_KD_L = 28,
		getPARAM_KD_R = 29,
		getPARAM_IMAX_L = 30,
		getPARAM_IMAX_R = 31,
		getMAXSPEED_FWD = 32,
		getMAXSPEED_REV = 33,
		getMAXACCEL = 34,
		getBATT_CELL_V = 35,
		getBATT_PARALLEL_COUNT = 37,
		getBATT_SERIES_COUNT = 38,
		getBATT_CELL_NOMINAL_V = 39,
		getBATT_VOLT = 40,
		getBATT_CURRENT = 41,
		getBATT_SOC = 42,
		getPOSITIONS = 43,
		getPOSITION_L = 44,
		getPOSITION_R = 45,
		getBUMPERS = 46,
		resetPOSITIONS = 47,
		resetPOSITION_L = 48,
		resetPOSITION_R = 49,
		getESTOP = 50,
		getDIAG = 51,
		ACK = 52,
		getBATT_CELL_CAPACITY=53,
		setBUMPER_ESTOP=54,
		getBUMPER_ESTOP=55,
		getESTOP_BTN=56,
		getSONARS=83
	}serial_t;
		
	private:
		void array_chopper(uint8_t *buf, int start, int end);
		unsigned char checksum(int size);
		bool checksum_match(uint8_t *buf, int size);
		unsigned char checksum_check_array(uint8_t *arr, int size);
		int first_validator(uint8_t *buf);
		int second_validator(uint8_t *buf, int data_len);
		int find_message_start(uint8_t *buf,  int lastIndex);
		int execute_command(uint8_t *buf);
		void print_array(uint8_t *buf, int length);
		int send_and_get_reply(uint8_t _command, uint8_t *send_array, int send_size, bool is_ack);
		int read_serial(uint8_t _command_to_read);
		int process(uint8_t *inData, int recievedData, uint8_t _command_to_read);
		int process_simple (uint8_t *inData, int recievedData, uint8_t _command_to_read);
		bool _get_ack(serial_t command);

		int _speed_l, _speed_r, _imax_l, _imax_r, _maxspeed_fwd, _maxspeed_rev, _maxaccel, _batt_volt, _batt_current, _batt_soc, _batt_cell_capacity, _bumper_estop, _estop_btn;
		bool _estop, _diag_motor_stall_l, _diag_motor_stall_r, _diag_batt_low, _diag_batt_high, _diag_motor_drvr_err, _diag_aux_lights_err;
		double _Kp_l, _Ki_l, _Kd_l, _Kp_r, _Ki_r, _Kd_r;
		int _position_l, _position_r;
		std::vector<int> _bumpers, _positions, _speeds, _imax, _sonars;

		int speeds[2];
		char sendArray[20];
		bool e_stop;
		bool dir_left;
		bool dir_right;

		int _port_nr;
		int _baudrate;
		std::string _port_name, _mode;
		//char _mode[3];

		uint8_t tempData[10000];
		uint8_t tempDataIndex;

		bool seekForChar;
		char startChar;
		uint8_t _ack_data;

		double Kp,Ki,Kd,Kol;
		double read_timeout_;

		milvus::SerialComm serial_port;
		milvus::UsbComm usb_port;

		uint16_t vendor_id_, product_id_; 
		int ep_in_addr_, ep_out_addr_;

		bool use_usb_;

		bool line_ok_;

		bool simple_;
};

#endif
