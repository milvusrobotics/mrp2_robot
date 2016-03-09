#include "mrp2_hardware/mrp2_serial.h"

static const double MILLION = 1000000.0;
static const double BILLION = 1000000000.0;

MRP2_Serial::MRP2_Serial (std::string port_name, uint32_t baudrate, std::string mode, bool simple)
 : _port_name(port_name), _baudrate(baudrate), _mode(mode), simple_(simple)
{
  std::fill_n(speeds, 2, 0);
  e_stop = false;
  dir_left =false;
  dir_right=true;
  tempDataIndex = 0;
  seekForChar = true;
  line_ok_ = true;
  startChar = '$';
  serial_port.open_port(_port_name, _baudrate, _mode);
  use_usb_ = false;
  read_timeout_ = 0.02;

}

MRP2_Serial::MRP2_Serial (uint16_t vendor_id, uint16_t product_id, int ep_in_addr, int ep_out_addr, bool simple)
 : vendor_id_(vendor_id), product_id_(product_id), ep_in_addr_(ep_in_addr), ep_out_addr_(ep_out_addr), simple_(simple)
{
  std::fill_n(speeds, 2, 0);
  e_stop = false;
  dir_left =false;
  dir_right=true;
  tempDataIndex = 0;
  seekForChar = true;
  line_ok_ = true;
  startChar = '$';
  usb_port.open_device(vendor_id_, product_id_, ep_in_addr_, ep_out_addr_);
  use_usb_ = true;
  read_timeout_ = 0.02;
}


MRP2_Serial::~MRP2_Serial ()
{
  
}

void
MRP2_Serial::update ()
{
}

void
MRP2_Serial::set_speeds(int32_t left_speed, int32_t right_speed)
{
  uint8_t send_array[20];

  left_speed *= -1;
  right_speed *= -1;

  unsigned char s1 = (unsigned char)(left_speed >> 24);
  unsigned char s2 = (unsigned char)(left_speed >> 16);
  unsigned char s3 = (unsigned char)(left_speed >> 8);
  unsigned char s4 = (unsigned char)left_speed;

  unsigned char s5 = (unsigned char)(right_speed >> 24);
  unsigned char s6 = (unsigned char)(right_speed >> 16);
  unsigned char s7 = (unsigned char)(right_speed >> 8);
  unsigned char s8 = (unsigned char)right_speed;
  
  send_array[0] = '$';
  send_array[1] = setSPEEDS;
  send_array[2] = 8;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = s1;
  send_array[5] = s2;
  send_array[6] = s3;
  send_array[7] = s4;
  send_array[8] = s5;
  send_array[9] = s6;
  send_array[10] = s7;
  send_array[11] = s8;
  send_array[12] = checksum_check_array(send_array, 12);

  send_and_get_reply(setSPEEDS, send_array, 13, true);

}

void 
MRP2_Serial::set_speed_l(int32_t left_speed)
{
  uint8_t send_array[20];

  left_speed *= -1;

  unsigned char s1 = (unsigned char)(left_speed >> 24);
  unsigned char s2 = (unsigned char)(left_speed >> 16);
  unsigned char s3 = (unsigned char)(left_speed >> 8);
  unsigned char s4 = (unsigned char)left_speed;
  
  send_array[0] = '$';
  send_array[1] = setSPEED_L;
  send_array[2] = 4;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = s1;
  send_array[5] = s2;
  send_array[6] = s3;
  send_array[7] = s4;
  send_array[8] = checksum_check_array(send_array, 8);

  send_and_get_reply(setSPEED_L, send_array, 9, true);

}

void 
MRP2_Serial::set_speed_r(int32_t right_speed)
{
  uint8_t send_array[20];

  right_speed *= -1;

  unsigned char s5 = (unsigned char)(right_speed >> 24);
  unsigned char s6 = (unsigned char)(right_speed >> 16);
  unsigned char s7 = (unsigned char)(right_speed >> 8);
  unsigned char s8 = (unsigned char)right_speed;
  
  send_array[0] = '$';
  send_array[1] = setSPEED_R;
  send_array[2] = 4;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = s5;
  send_array[5] = s6;
  send_array[6] = s7;
  send_array[7] = s8;
  send_array[8] = checksum_check_array(send_array, 8);

  send_and_get_reply(setSPEED_R, send_array, 9, true);

}

void
MRP2_Serial::set_param_pid(char side, char param, float value)
{

  //printf("Float value: %f\n", value);
  uint8_t send_array[20];
  
  union {
    float f;
    unsigned char bytes[4];
  } val;

  val.f = value;


  unsigned char fl_array[4];
  memcpy(fl_array, &val, 4);


  if(side == 'L')
  {
    switch(param)
    {
      case 'P':
        send_array[1] = setPARAM_KP_L;
        break;
      case 'I':
        send_array[1] = setPARAM_KI_L;
        break;
      case 'D':
        send_array[1] = setPARAM_KD_L;
        break;
      default:
        break;
    }
  }else if(side == 'R')
  {
    switch(param)
    {
      case 'P':
        send_array[1] = setPARAM_KP_R;
        break;
      case 'I':
        send_array[1] = setPARAM_KI_R;
        break;
      case 'D':
        send_array[1] = setPARAM_KD_R;
        break;
      default:
        break;
    }
  }

  //printf("Side: %c, param: %c:\n", side, param);
  //print_array(fl_array, 4);

  send_array[0] = '$';
  send_array[2] = 4;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = fl_array[3];
  send_array[5] = fl_array[2];
  send_array[6] = fl_array[1];
  send_array[7] = fl_array[0];
  send_array[8] = checksum_check_array(send_array, 8);

  send_and_get_reply(send_array[1], send_array, 9, true);
}

void
MRP2_Serial::set_param_imax(char side, uint32_t value)
{
  uint8_t send_array[20];
  
  if(side == 'L'){
    send_array[1] = setPARAM_IMAX_L;
  }else if(side == 'R'){
    send_array[1] = setPARAM_IMAX_R;
  }

  send_array[0] = '$';  
  send_array[2] = 4;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = (unsigned char)(value >> 24);
  send_array[5] = (unsigned char)(value >> 16);
  send_array[6] = (unsigned char)(value >> 8);
  send_array[7] = (unsigned char)value;
  send_array[8] = checksum_check_array(send_array, 8);

  send_and_get_reply(send_array[1], send_array, 9, true);

}

void
MRP2_Serial::set_maxspeed_fwd(uint32_t value)
{
  uint8_t send_array[20];

  send_array[0] = '$';
  send_array[1] = setMAXSPEED_FWD;  
  send_array[2] = 4;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = (unsigned char)(value >> 24);
  send_array[5] = (unsigned char)(value >> 16);
  send_array[6] = (unsigned char)(value >> 8);
  send_array[7] = (unsigned char)value;
  send_array[8] = checksum_check_array(send_array, 8);

  send_and_get_reply(setMAXSPEED_FWD, send_array, 9, true);
}

void
MRP2_Serial::set_maxspeed_rev(uint32_t value)
{
  uint8_t send_array[20];

  send_array[0] = '$';
  send_array[1] = setMAXSPEED_REV;  
  send_array[2] = 4;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = (unsigned char)(value >> 24);
  send_array[5] = (unsigned char)(value >> 16);
  send_array[6] = (unsigned char)(value >> 8);
  send_array[7] = (unsigned char)value;
  send_array[8] = checksum_check_array(send_array, 8);

  send_and_get_reply(setMAXSPEED_REV, send_array, 9, true);
}

void
MRP2_Serial::set_max_accel(uint32_t value)
{
  uint8_t send_array[20];

  send_array[0] = '$';
  send_array[1] = setMAXACCEL;  
  send_array[2] = 4;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = (unsigned char)(value >> 24);
  send_array[5] = (unsigned char)(value >> 16);
  send_array[6] = (unsigned char)(value >> 8);
  send_array[7] = (unsigned char)value;
  send_array[8] = checksum_check_array(send_array, 8);


  send_and_get_reply(setMAXACCEL, send_array, 9, true);
}

void
MRP2_Serial::set_estop(bool value)
{
  uint8_t send_array[20];

  send_array[0] = '$';
  send_array[1] = setESTOP;  
  send_array[2] = 1;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = (unsigned char)value;
  send_array[5] = checksum_check_array(send_array, 6);

  send_and_get_reply(setESTOP, send_array, 6, true);
}

void
MRP2_Serial::clear_diag(int diag)
{
  uint8_t send_array[20];
  send_array[0] = '$';
  send_array[1] = clearDIAG;
  send_array[2] = 1;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = diag; 
  send_array[5] = checksum_check_array(send_array, 5);

  send_and_get_reply(clearDIAG, send_array, 6, true);
}

std::vector<int> 
MRP2_Serial::get_speeds(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getSPEEDS;
    send_and_get_reply(getSPEEDS, send_array, 2, false);
  }
  return _speeds;
}

int 
MRP2_Serial::get_speed_l(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getSPEED_L;
    send_and_get_reply(getSPEED_L, send_array, 2, false);
  }
  return _speed_l;
}

int 
MRP2_Serial::get_speed_r(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getSPEED_R;
    send_and_get_reply(getSPEED_R, send_array, 2, false);
  }
  return _speed_r;
}

float
MRP2_Serial::get_param_pid(char side, char param, bool update)
{
  uint8_t send_array[2];

  send_array[0] = '$';

  if(side == 'L')
  {
    if(param == 'P'){
      if(update){
        send_array[1] = getPARAM_KP_L;
        send_and_get_reply(send_array[1], send_array, 2, false);
      }
      return _Kp_l;
    }else if(param == 'I'){
      if(update){
        send_array[1] = getPARAM_KI_L;
        send_and_get_reply(send_array[1], send_array, 2, false);
      }
      return _Ki_l;
    }else if(param == 'D'){
      if(update){
        send_array[1] = getPARAM_KD_L;
        send_and_get_reply(send_array[1], send_array, 2, false);
      }
      return _Kd_l;
    }
  }
  else if(side == 'R'){
    if(param == 'P'){
      if(update){
        send_array[1] = getPARAM_KP_R;
        send_and_get_reply(send_array[1], send_array, 2, false);
      }
      return _Kp_r;
    }else if(param == 'I'){
      if(update){
        send_array[1] = getPARAM_KI_R;
        send_and_get_reply(send_array[1], send_array, 2, false);
      }
      return _Ki_r;
    }else if(param == 'D'){
      if(update){
        send_array[1] = getPARAM_KD_R;
        send_and_get_reply(send_array[1], send_array, 2, false);
      }
      return _Kd_r;
    }
  }

}

std::vector<int>
MRP2_Serial::get_param_imax(char side, bool update)
{
  if (update) {
    uint8_t send_array[2];

    send_array[0] = '$';

    if(side == 'L')
    {
      send_array[1] = getPARAM_IMAX_L;
    }else if(side =='R')
    {
      send_array[1] = getPARAM_IMAX_R;
    }
    
    send_and_get_reply(send_array[1], send_array, 2, false);
  }
  return _imax;
}

int
MRP2_Serial::get_maxspeed_fwd(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getMAXSPEED_FWD;
    send_and_get_reply(getMAXSPEED_FWD, send_array, 2, false);
  }
  return _maxspeed_fwd;
}

int
MRP2_Serial::get_maxspeed_rev(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getMAXSPEED_REV;
    send_and_get_reply(getMAXSPEED_REV, send_array, 2, false);
  }
  return _maxspeed_rev;
}

int
MRP2_Serial::get_maxaccel(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getMAXACCEL;
    send_and_get_reply(getMAXACCEL, send_array, 2, false);
  }
  return _maxaccel;
}

int
MRP2_Serial::get_batt_volt(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getBATT_VOLT;
    send_and_get_reply(getBATT_VOLT, send_array, 2, false);
  }
  return _batt_volt;
}

int
MRP2_Serial::get_batt_current(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getBATT_CURRENT;
    send_and_get_reply(getBATT_CURRENT, send_array, 2, false);
  }
  return _batt_current;
}

int
MRP2_Serial::get_batt_soc(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getBATT_SOC;
    send_and_get_reply(getBATT_SOC, send_array, 2, false);
  }
  return _batt_soc;
}

std::vector<int> 
MRP2_Serial::get_positions(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getPOSITIONS;
    send_and_get_reply(getPOSITIONS, send_array, 2, false);
  }
  return _positions;
}

int 
MRP2_Serial::get_position_l(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getPOSITION_L;
    send_and_get_reply(getPOSITION_L, send_array, 2, false);
  }
  return _position_l;
}

int 
MRP2_Serial::get_position_r(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getPOSITION_R;
    send_and_get_reply(getPOSITION_R, send_array, 2, false);
  }
  return _position_r;
}

std::vector<int> 
MRP2_Serial::get_bumpers(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getBUMPERS;
    send_and_get_reply(getBUMPERS, send_array, 2, false);
  }
  return _bumpers;
}

void 
MRP2_Serial::reset_positions()
{
  uint8_t send_array[10];
  
  send_array[0] = '$';
  send_array[1] = resetPOSITIONS;
  send_array[2] = 0;
  send_array[3] = checksum(4);

  send_and_get_reply(resetPOSITIONS, send_array, 4, true);
}

void 
MRP2_Serial::reset_position_l()
{
  uint8_t send_array[10];
  
  send_array[0] = '$';
  send_array[1] = resetPOSITION_L;
  send_array[2] = 0;
  send_array[3] = checksum(4);

  send_and_get_reply(resetPOSITION_L, send_array, 4, true);
}

void 
MRP2_Serial::reset_position_r()
{
  uint8_t send_array[10];
  
  send_array[0] = '$';
  send_array[1] = resetPOSITION_R;
  send_array[2] = 0;
  send_array[3] = checksum(4);

  send_and_get_reply(resetPOSITION_R, send_array, 4, true);
}

bool
MRP2_Serial::get_estop(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getESTOP;
    send_and_get_reply(getESTOP, send_array, 2, false);
  }
  return _estop;
}

void 
MRP2_Serial::update_diag()
{
  uint8_t send_array[2];
  send_array[0] = '$';
  send_array[1] = getDIAG;
  send_and_get_reply(getSPEED_R, send_array, 2, false);
}

bool
MRP2_Serial::get_diag(int diag)
{
  switch(diag){
      case DIAG_MOTOR_STALL_L:
        return _diag_motor_stall_l;
    break;
      case DIAG_MOTOR_STALL_R:
        return _diag_motor_stall_r;
    break;
      case DIAG_BATT_LOW:
        return _diag_batt_low;
    break;
      case DIAG_BATT_HIGH:
        return _diag_batt_high;
    break;
      case DIAG_MOTOR_DRVR_ERR:
        return _diag_motor_drvr_err;
    break;
      case DIAG_AUX_LIGHTS_ERR:
        return _diag_aux_lights_err;
    break;
  }
}

int 
MRP2_Serial::get_batt_cell_capacity(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getBATT_CELL_CAPACITY;
    send_and_get_reply(getBATT_CELL_CAPACITY, send_array, 2, false);
  }
  return _batt_cell_capacity;
}

void
MRP2_Serial::set_bumper_estop(bool value)
{
  uint8_t send_array[20];

  send_array[0] = '$';
  send_array[1] = setBUMPER_ESTOP;  
  send_array[2] = 1;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = (unsigned char)value;
  send_array[5] = checksum_check_array(send_array, 6);

  send_and_get_reply(setBUMPER_ESTOP, send_array, 6, true);
}

bool 
MRP2_Serial::get_bumper_estop(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getBUMPER_ESTOP;
    send_and_get_reply(getBUMPER_ESTOP, send_array, 2, false);
  }
  return _bumper_estop;
}
    
bool
MRP2_Serial::get_estop_button(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getESTOP_BTN;
    send_and_get_reply(getESTOP_BTN, send_array, 2, false);
  }
  return _estop_btn;
}

std::vector<int> 
MRP2_Serial::get_sonars(bool update)
{
  if (update) {
    uint8_t send_array[2];
    send_array[0] = '$';
    send_array[1] = getSONARS;
    send_and_get_reply(getSONARS, send_array, 2, false);
  }
  return _sonars;
}

int
MRP2_Serial::send_and_get_reply(uint8_t _command, uint8_t *send_array, int send_size, bool is_ack)
{
  struct timeval  tv1, tv2;
  struct timespec ctv1, ctv2;

  double _time_diff = 0;
  int _ret_val = 0;

  gettimeofday(&tv1, NULL);
  
  //printf("%ld.%06ld - SAGR called: %d\n", tv1.tv_sec, tv1.tv_usec, _command);

  while (!line_ok_)
  {
    usleep(1);
  }

  line_ok_ = false;

  gettimeofday(&tv1, NULL);

  //printf("%ld.%06ld - Serial send called: %d\n", tv1.tv_sec, tv1.tv_usec, _command);

  if(use_usb_)
    int ret = usb_port.write_bytes(send_array,send_size);
  else
    int ret = serial_port.send_buf(send_array,send_size);

  gettimeofday(&tv1, NULL);
  gettimeofday(&tv2, NULL);

  clock_gettime(CLOCK_MONOTONIC, &ctv1);
  clock_gettime(CLOCK_MONOTONIC, &ctv2);

  //printf("%ld.%06ld - Command sent: %d\n", ctv1.tv_sec, ctv1.tv_nsec, _command);

  if (is_ack)
  {
    _ack_data = 0;
    while (_ack_data != _command && _time_diff < read_timeout_)
    {
      _ret_val = read_serial(ACK);
      //gettimeofday(&tv2, NULL);
      //_time_diff = (double) (tv2.tv_usec - tv1.tv_usec) / MILLION + (double) (tv2.tv_sec - tv1.tv_sec);
      clock_gettime(CLOCK_MONOTONIC, &ctv2);
      _time_diff = ctv2.tv_sec - ctv1.tv_sec + (ctv2.tv_nsec - ctv1.tv_nsec) / BILLION;
      usleep(1);
    }
    line_ok_ = true;
    /*if(_ack_data != _command && _time_diff >= _time_out)
      printf("ACK Timeout!, Command: %d\n", _command);
    else if (_ack_data == _command && _time_diff < _time_out)
      printf("%ld.%06ld - Command reply received: %d\n", ctv2.tv_sec, ctv2.tv_nsec, _command);*/
  }
  else
  {
    while (_ret_val == 0 && _time_diff < read_timeout_)
    {

        _ret_val = read_serial(_command);
        //gettimeofday(&tv2, NULL);
        //_time_diff = (double) (tv2.tv_usec - tv1.tv_usec) / MILLION + (double) (tv2.tv_sec - tv1.tv_sec);
        clock_gettime(CLOCK_MONOTONIC, &ctv2);
        _time_diff = ctv2.tv_sec - ctv1.tv_sec + (ctv2.tv_nsec - ctv1.tv_nsec) / BILLION;
        usleep(1);
    }
    line_ok_ = true;
    /*if(_ret_val == 0 && _time_diff >= _time_out)
      printf("Read Timeout!, Command: %d\n", _command);
    else if (_ret_val != 0 && _time_diff < _time_out)
      printf("%ld.%06ld - Command reply received: %d\n", ctv2.tv_sec, ctv2.tv_nsec, _command);*/

  }
  
  
  return _ret_val;
}

bool
MRP2_Serial::is_available ()
{
  return line_ok_;
}

int
MRP2_Serial::read_serial (uint8_t _command_to_read)
{
  uint8_t inData[24] = {0};
  int recievedData = 0;

  if(use_usb_)
    recievedData = usb_port.read_bytes(inData,24);
  else
    recievedData = serial_port.poll_comport(inData, 24);

  /*if(_command_to_read == getSONARS && recievedData > 0)
    print_array(inData, recievedData);*/

  if (recievedData == 0)
    return 0;

  if (simple_)
    return process_simple(inData, recievedData, _command_to_read);
  else
    return process(inData, recievedData, _command_to_read);
}

int 
MRP2_Serial::process_simple (uint8_t *inData, int recievedData, uint8_t _command_to_read)
{
  int ret_val_ = 0;

  uint8_t* ret_data_ = new uint8_t[recievedData];
  memcpy( ret_data_, inData, recievedData * sizeof(uint8_t) );

  int data_len = first_validator(ret_data_);
  if(second_validator(ret_data_, data_len) != -1)
  {
      ret_val_ = execute_command(ret_data_);
  }
  return ret_val_;
}

int 
MRP2_Serial::process (uint8_t *inData, int recievedData, uint8_t _command_to_read)
{
  int startIndex = 0;
  int proc_length = 0;
  int while_cnt = 0;
  int _ret_val = 0;

  if (recievedData > 0) 
  {

      if(tempDataIndex == 0)
      {
          memcpy(tempData, inData, recievedData); 
          tempDataIndex = recievedData;
      }else{ 
          memcpy(&tempData[tempDataIndex], &inData[0], recievedData); 
          tempDataIndex += recievedData;
      }


      tempDataIndex = find_message_start(tempData,tempDataIndex);
  }

  if (tempDataIndex > 3 && tempData[0] == startChar) 
  {
      
      int data_len = first_validator(tempData);


      if (tempDataIndex < data_len+5)
      {
        return _ret_val;
      }


      if (data_len != -1)
      {
        if (tempData[1] != _command_to_read)
        {
          tempData[0] = '0';
          return _ret_val;          
        }
      }

      if(data_len == 0 && tempDataIndex >= 3)
      {
          _ret_val = execute_command(tempData);
          tempData[0] = '0'; 
          tempDataIndex = find_message_start(tempData,tempDataIndex);
      }else if(data_len > 0 && tempDataIndex >= data_len+5)
      {
          if(second_validator(tempData, data_len) != -1)
          {

              _ret_val = execute_command(tempData);

          }
          tempData[0] = '0'; 
          tempDataIndex = find_message_start(tempData,tempDataIndex);

      }else if(data_len == -1){

          tempData[0] = '0'; 
          tempDataIndex = find_message_start(tempData,tempDataIndex);
      }

  }

  return _ret_val;

}

void 
MRP2_Serial::array_chopper(uint8_t *buf, int start, int end) {
  int k = 0;
  for (int i = start; i < start+end; i++)
  {
    tempData[k] = buf[i];
    k++;
  }
};

unsigned char 
MRP2_Serial::checksum(int size)
{
  uint8_t ret = 0;
  for(int i=0; i<size; i++)
  {
    ret = ret + sendArray[i];
  }
  if(size > 3)
  {
    return (ret & 255) - 1; 
  }
  return ret & 255 ;
}

unsigned char 
MRP2_Serial::checksum_check_array(uint8_t *arr, int size)
{
  uint8_t ret = 0;
  for(int i=0; i<size; i++)
  {
    ret = ret + arr[i];
  }
  if(size > 3)
  {
    ret = (ret & 0xFF) - 1; 
  }
  else{
    ret = ret & 0xFF; 
  }
  return ret;
}

bool 
MRP2_Serial::checksum_match(uint8_t *buf, int size) {
  uint8_t checksum = 0;
  int i = 0;
  for(i = 0; i<size; i++) 
  {
      checksum = checksum + buf[i];
  }
  if(size > 3)
  {
    checksum = (checksum & 0xFF) - 1; 
    
  }else{
    checksum = checksum & 0xFF; 
  }

  if(checksum == buf[size])    
  {
      return true;
  }    
  return false;
};

int 
MRP2_Serial::first_validator(uint8_t *buf) {
  if(checksum_match(buf, 3))
    {
        return buf[2];
    }
    return -1;
};

int 
MRP2_Serial::second_validator(uint8_t *buf, int data_len) {
  int total_len = data_len + 5;
  if(checksum_match(buf, 4 + data_len))
  {
      return 1;
  }
  return -1;
};

int 
MRP2_Serial::find_message_start(uint8_t *buf,  int lastIndex) {
  int start = 0;
  for (start = 0; start < lastIndex ; start++)
  {
      if(buf[start] == startChar)
      {
          break;
      }
  }
  
  lastIndex = lastIndex - start;

  array_chopper(buf, start, lastIndex+1);
  
  return lastIndex;
};

int 
MRP2_Serial::execute_command(uint8_t *buf) {

  union{
    float f;
    uint8_t bytes[4];
  }val;

  if(buf[1] == getBUMPERS)
  {
    _bumpers.clear();
    _bumpers.push_back((buf[4] >> 3) & 0x01); // front left
    _bumpers.push_back((buf[4] >> 2) & 0x01); // front right
    _bumpers.push_back((buf[4] >> 1) & 0x01); // rear left
    _bumpers.push_back(buf[4] & 0x01);        // rear right

  }

  if(buf[1] == 52)
  {
    _ack_data = buf[4];
  }

  if(buf[1] == getSPEEDS)
  {
      int l_speed = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24); 
      int r_speed= buf[8] + (buf[9] << 8) + (buf[10] << 16) + (buf[11] << 24); 
      
      _speed_l = l_speed*-1;
      _speed_r = r_speed*-1;
      _speeds.clear();
      _speeds.push_back(_speed_l);
      _speeds.push_back(_speed_r);

  }

  if(buf[1] == getSPEED_L)
  {
      int l_speed = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24); 
      
      _speed_l = l_speed*-1;
      _speeds.clear();
      _speeds.push_back(_speed_l);
      _speeds.push_back(_speed_r);

  }

  if(buf[1] == getSPEED_R)
  {
      int r_speed = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24); 
      
      _speed_r = r_speed*-1;
      _speeds.clear();
      _speeds.push_back(_speed_l);
      _speeds.push_back(_speed_r);

  }

  if(buf[1] == getPARAM_KP_L)
  {
    //printf("KP_L: 4:%d, 5:%d, 6:%d, 7:%d\n", buf[4], buf[5], buf[6], buf[7]);

    val.bytes[0] = buf[4];
    val.bytes[1] = buf[5];
    val.bytes[2] = buf[6];
    val.bytes[3] = buf[7];

    _Kp_l = val.f;
    //printf("kp_l:%f\n", _Kp_l);
  }

  if(buf[1] == getPARAM_KI_L)
  {
    val.bytes[0] = buf[4];
    val.bytes[1] = buf[5];
    val.bytes[2] = buf[6];
    val.bytes[3] = buf[7];
    _Ki_l = val.f;
     //printf("ki_l:%f\n", _Ki_l);
  }

  if(buf[1] == getPARAM_KD_L)
  {
    val.bytes[0] = buf[4];
    val.bytes[1] = buf[5];
    val.bytes[2] = buf[6];
    val.bytes[3] = buf[7];
    _Kd_l = val.f;
     //printf("kd_l:%f\n", _Kd_l);
  }

  if(buf[1] == getPARAM_KP_R)
  {
    val.bytes[0] = buf[4];
    val.bytes[1] = buf[5];
    val.bytes[2] = buf[6];
    val.bytes[3] = buf[7];
    _Kp_r = val.f;
     //printf("kp_r:%f\n", _Kp_r);
  }

  if(buf[1] == getPARAM_KI_R)
  {
    val.bytes[0] = buf[4];
    val.bytes[1] = buf[5];
    val.bytes[2] = buf[6];
    val.bytes[3] = buf[7];
    _Ki_r = val.f;
    //printf("ki_r:%f\n", _Ki_r);
  }

  if(buf[1] == getPARAM_KD_R)
  {
    val.bytes[0] = buf[4];
    val.bytes[1] = buf[5];
    val.bytes[2] = buf[6];
    val.bytes[3] = buf[7];
    _Kd_r = val.f;
    //printf("kd_r:%f\n", _Kd_r);
  }

  if(buf[1] == getPARAM_IMAX_L)
  {
    _imax_l = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24);
    _imax.clear();
    _imax.push_back(_imax_l);
    _imax.push_back(_imax_r);
  }

  if(buf[1] == getPARAM_IMAX_R)
  {
    _imax_r = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24);
    _imax.clear();
    _imax.push_back(_imax_l);
    _imax.push_back(_imax_r);
  }

  if(buf[1] == getMAXSPEED_FWD)
  {
    _maxspeed_fwd = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24);
  }

  if(buf[1] == getMAXSPEED_REV)
  {
    _maxspeed_rev = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24);
  }

  if(buf[1] == getMAXACCEL)
  {
    _maxaccel = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24);
  }

  if(buf[1] == getBATT_VOLT)
  {
    _batt_volt = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24);
  }

  if(buf[1] == getBATT_CURRENT)
  {
    _batt_current = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24);
  }

  if(buf[1] == getBATT_SOC)
  {
    _batt_soc = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24);
  }

  if(buf[1] == getPOSITION_L)
  {

      int l_position = buf[5] + (buf[6] << 8) + (buf[7] << 16) + (buf[8] << 24);
      if(buf[4] == 1)
      {
        l_position *= -1;
      }
        
      _position_l = l_position;
      _positions.clear();
      _positions.push_back(_position_l);
      _positions.push_back(_position_r);

  }

  if(buf[1] == getPOSITION_R)
  {

      int r_position = buf[5] + (buf[6] << 8) + (buf[7] << 16) + (buf[8] << 24);
      if(buf[4] == 1)
      {
        r_position *= -1;
      }
        
      _position_r = r_position;
      _positions.clear();
      _positions.push_back(_position_l);
      _positions.push_back(_position_r);

  }

  if(buf[1] == getESTOP)
  {
    _estop = buf[4];
  }

  if(buf[1] == getDIAG)
  {
    _diag_motor_stall_l = buf[5];
    _diag_motor_stall_r = buf[6];
    _diag_batt_low = buf[7];
    _diag_batt_high = buf[8];
    _diag_motor_drvr_err = buf[9];
    _diag_aux_lights_err = buf[10];
  }

  if(buf[1] == getBATT_CELL_CAPACITY)
  {
    _batt_cell_capacity = buf[5] + (buf[6] << 8) + (buf[7] << 16) + (buf[8] << 24);
  }

  if(buf[1] == getBUMPER_ESTOP)
  {
    _bumper_estop = buf[4];
  }

  if(buf[1] == getESTOP_BTN)
  {
    _estop_btn = buf[4];
  }

  if(buf[1] == getSONARS)
  {
    _sonars.clear();
    _sonars.push_back(buf[4] + (buf[5] << 8));
    _sonars.push_back(buf[6] + (buf[7] << 8));
    _sonars.push_back(buf[8] + (buf[9] << 8));
    _sonars.push_back(buf[10] + (buf[11] << 8));
    _sonars.push_back(buf[12] + (buf[13] << 8));
    _sonars.push_back(buf[14] + (buf[15] << 8));
    _sonars.push_back(buf[16] + (buf[17] << 8));
  }

  return buf[1];

};

void 
MRP2_Serial::print_array(uint8_t *buf, int length) {
  printf("Array: ");
  int i = 0;
  for (i = 0; i < length; i++)
  {
      //printf("%02x ",buf[i] );
    printf("%d ",buf[i] );
  }
  printf("\n");
};

void 
MRP2_Serial::set_read_timeout(double timeout){
  read_timeout_ = timeout;
}

double 
MRP2_Serial::get_read_timeout(void){
  return read_timeout_;
}
