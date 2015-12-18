#include "mrp2_hardware/mrp2_serial.h"

MRP2_Serial::MRP2_Serial (int port_nr, uint32_t baudrate)
 : _baudrate(baudrate), _port_nr(port_nr)
{
  std::fill_n(speeds, 2, 0);
  e_stop = false;
  dir_left =false;
  dir_right=true;
  tempDataIndex = 0;
  seekForChar = true;
  startChar = '$';
  _mode[0] = '8'; _mode[1] = 'N'; _mode[2] = '1';
  
  if(RS232_OpenComport(_port_nr,_baudrate,_mode))
  {
    printf("Can't open comport.\n");
  }
  /*else
    printf("Port %d opened with baud %d.\n", _port_nr, _baudrate);*/

}

MRP2_Serial::~MRP2_Serial ()
{
  
}

void
MRP2_Serial::update ()
{
    //read_serial();  
    //printf("Finished\n");
}

void
MRP2_Serial::set_speeds(int32_t left_speed, int32_t right_speed)
{
  uint8_t send_array[20];
  if(left_speed < 0){
    dir_left = 1;
    left_speed *= -1;
  }else{
    dir_left = 0;
  }

  if(right_speed < 0){
    dir_right = 1;
    right_speed *= -1;
  }else{
    dir_right = 0;
  }

  unsigned char s1 = (unsigned char)(left_speed >> 16);
  unsigned char s2 = (unsigned char)(left_speed >> 8);
  unsigned char s3 = (unsigned char)left_speed;

  unsigned char s4 = (unsigned char)(right_speed >> 16);
  unsigned char s5 = (unsigned char)(right_speed >> 8);
  unsigned char s6 = (unsigned char)right_speed;
  
  send_array[0] = '$';
  send_array[1] = setSPEEDS;
  send_array[2] = 8;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = dir_left;
  send_array[5] = s1;
  send_array[6] = s2;
  send_array[7] = s3;
  send_array[8] = dir_right;
  send_array[9] = s4;
  send_array[10] = s5;
  send_array[11] = s6;
  send_array[12] = checksum_check_array(send_array, 12);

  send_and_get_reply(setSPEEDS, send_array, 13, true);

}

void 
MRP2_Serial::set_speed_l(int32_t left_speed)
{
  uint8_t send_array[20];
  if(left_speed < 0){
    dir_left = 1;
    left_speed *= -1;
  }else{
    dir_left = 0;
  }

  unsigned char s1 = (unsigned char)(left_speed >> 16);
  unsigned char s2 = (unsigned char)(left_speed >> 8);
  unsigned char s3 = (unsigned char)left_speed;
  
  send_array[0] = '$';
  send_array[1] = setSPEED_L;
  send_array[2] = 4;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = dir_left;
  send_array[5] = s1;
  send_array[6] = s2;
  send_array[7] = s3;
  send_array[8] = checksum_check_array(send_array, 8);

  send_and_get_reply(setSPEED_L, send_array, 9, true);

}

void 
MRP2_Serial::set_speed_r(int32_t right_speed)
{
  uint8_t send_array[20];
  

  if(right_speed < 0){
    dir_right = 1;
    right_speed *= -1;
  }else{
    dir_right = 0;
  }

  unsigned char s4 = (unsigned char)(right_speed >> 16);
  unsigned char s5 = (unsigned char)(right_speed >> 8);
  unsigned char s6 = (unsigned char)right_speed;
  
  send_array[0] = '$';
  send_array[1] = setSPEED_R;
  send_array[2] = 4;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = dir_right;
  send_array[5] = s4;
  send_array[6] = s5;
  send_array[7] = s6;
  send_array[8] = checksum_check_array(send_array, 8);

  send_and_get_reply(setSPEED_R, send_array, 9, true);

}

void
MRP2_Serial::set_param_pid(char side, char param, double value)
{
  uint32_t data = (uint32_t)(value*10000);
  uint8_t send_array[20];
  
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

  send_array[0] = '$';
  send_array[2] = 4;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = (unsigned char)(data >> 24);
  send_array[5] = (unsigned char)(data >> 16);
  send_array[6] = (unsigned char)(data >> 8);
  send_array[7] = (unsigned char)data;
  send_array[8] = checksum_check_array(send_array, 8);

  send_and_get_reply(send_array[1], send_array, 9, true);
}

void
MRP2_Serial::set_param_imax(char side, uint16_t value)
{
  uint8_t send_array[20];
  
  if(side == 'L'){
    send_array[1] = setPARAM_IMAX_L;
  }else if(side == 'R'){
    send_array[1] = setPARAM_IMAX_R;
  }

  send_array[0] = '$';  
  send_array[2] = 2;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = (unsigned char)(value >> 8);
  send_array[5] = (unsigned char)value;
  send_array[6] = checksum_check_array(send_array, 6);

  send_and_get_reply(send_array[1], send_array, 7, true);

}

void
MRP2_Serial::set_maxspeed_fwd(uint16_t value)
{
  uint8_t send_array[20];

  send_array[0] = '$';
  send_array[1] = setMAXSPEED_FWD;  
  send_array[2] = 2;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = (unsigned char)(value >> 8);
  send_array[5] = (unsigned char)value;
  send_array[6] = checksum_check_array(send_array, 6);

  send_and_get_reply(setMAXSPEED_FWD, send_array, 7, true);
}

void
MRP2_Serial::set_maxspeed_rev(uint16_t value)
{
  uint8_t send_array[20];

  send_array[0] = '$';
  send_array[1] = setMAXSPEED_REV;  
  send_array[2] = 2;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = (unsigned char)(value >> 8);
  send_array[5] = (unsigned char)value;
  send_array[6] = checksum_check_array(send_array, 6);

  send_and_get_reply(setMAXSPEED_REV, send_array, 7, true);
}

void
MRP2_Serial::set_max_accel(uint16_t value)
{
  uint8_t send_array[20];

  send_array[0] = '$';
  send_array[1] = setMAXACCEL;  
  send_array[2] = 2;
  send_array[3] = checksum_check_array(send_array, 3);
  send_array[4] = (unsigned char)(value >> 8);
  send_array[5] = (unsigned char)value;
  send_array[6] = checksum_check_array(send_array, 6);

  send_and_get_reply(setMAXACCEL, send_array, 7, true);
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

double
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
    //send_array[0] = '$';
    //send_array[1] = 'S';
    send_and_get_reply(getSONARS, send_array, 2, false);
  }
  return _sonars;
}

int
MRP2_Serial::send_and_get_reply(uint8_t _command, uint8_t *send_array, int send_size, bool is_ack)
{
  struct timeval  tv1, tv2;

  time_t time_1 = time(0);
  time_t time_2 = time(0);
  double _time_diff = 0;
  int _ret_val = 0;
  double _time_out = 0.1;

  gettimeofday(&tv1, NULL);
  gettimeofday(&tv2, NULL);

  //printf("\nSent and get reply called at %f...\n", (double)(tv1.tv_sec + (double)(tv1.tv_usec/1000000.0)) );
  int ret =RS232_SendBuf(_port_nr,send_array,send_size);
  
  if (is_ack)
  {
    _ack_data = 0;
    while (_ack_data != _command && _time_diff < _time_out)
    {
      _ret_val = read_serial(ACK);
      gettimeofday(&tv2, NULL);
      _time_diff = (double) (tv2.tv_usec - tv1.tv_usec) / 1000000 + (double) (tv2.tv_sec - tv1.tv_sec);
    }
    //printf("ACK is %d for command %d\n", _ack_data, _command);
  }
  else
  {
    while (_ret_val == 0 && _time_diff < _time_out)
    {
        _ret_val = read_serial(_command);
        gettimeofday(&tv2, NULL);
        _time_diff = (double) (tv2.tv_usec - tv1.tv_usec) / 1000000 + (double) (tv2.tv_sec - tv1.tv_sec);
    }
    //printf("Reply is %d for command %d\n", _ret_val, _command);
  }

  /*if (_time_diff >= _time_out)
    printf("Timeout Occured!\n");*/
  
  //gettimeofday(&tv2, NULL);
  //printf("\nSent and get reply returned at %f...\n", (double)(tv2.tv_sec + (double)(tv2.tv_usec/1000000.0)) );
  return _ret_val;
}

int
MRP2_Serial::read_serial (uint8_t _command_to_read)
{
  uint8_t inData[50] = {0};
  int recievedData = 0;
  //sleep(1);
  
  

  //for (int i = 0; i < 10; ++i)
  //{
    
    recievedData = RS232_PollComport(_port_nr, inData, 50);
    //usleep(10);
    //printf("\nCalling process For Command %d\n", _command_to_read);
    return process(inData, recievedData, _command_to_read);
    //printf("while count: %d\n", process(inData, recievedData));
  
  //}
}

int 
MRP2_Serial::process (uint8_t *inData, int recievedData, uint8_t _command_to_read)
{
  //uint8_t inData[255] = {0};
  //int recievedData = sizeof(inData)/sizeof(inData[0]);
  int startIndex = 0;
  int proc_length = 0;
  int while_cnt = 0;
  int _ret_val = 0;
  //recievedData = mrp_serial.read(inData,20);
  //usleep(50);
  //recievedData = RS232_PollComport(_port_nr, inData, 255); // Immediately returns
//ROS_INFO("recieved: %d\n",recievedData);
  //printf("Port %d polled with baud %d, received %d bytes.\n", _port_nr, _baudrate, recievedData);
  

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

      tempDataIndex = find_message_start(tempData,tempDataIndex);
  }
  //ROS_INFO("tempDataIndex1:  %d", tempDataIndex);
  if (tempDataIndex > 3 && tempData[0] == startChar) // ilk checksum için yeterli data varsa ve ilk karakter doğruysa
  {
      //print_array(tempData, 50);
      int data_len = first_validator(tempData);
      //printf("TempdataIndex :%d, data_len: %d\n",tempDataIndex, data_len);

      if (tempDataIndex < data_len+5)
      {
        return _ret_val;
      }

      if (data_len != -1)
      {
        if (tempData[1] != _command_to_read)
        {
          //printf("\nData (i.e, %d) is valid but not the one we are waiting (i.e: %d). Trashing and Continuing...\n",tempData[1], _command_to_read);
          tempData[0] = '0';
          return _ret_val;
          /*if (tempData[4] != _command_to_read)
          {
            printf("\nThis is not the ACK we wait for. It is for %d. Trashing and Continuing...\n", tempData[4]);
            tempData[0] = '0';
            continue;
          }*/
          
        }
      }

      //printArray(tempData,tempDataIndex);
      if(data_len == 0 && tempDataIndex >= 3)
      {
          //ROS_INFO("Data is VALID! 1 checksum\n");
          _ret_val = execute_command(tempData);
          tempData[0] = '0'; // ilk mesajı boz
          tempDataIndex = find_message_start(tempData,tempDataIndex);
      }else if(data_len > 0 && tempDataIndex >= data_len+5)
      {
          if(second_validator(tempData, data_len) != -1)
          {
              //ROS_INFO("Data is VALID! 2 checksum\n");
              _ret_val = execute_command(tempData);
              //proc_length = sizeof(tempData)/sizeof(tempData[0]);
          }
          tempData[0] = '0'; // ilk mesajı boz
          tempDataIndex = find_message_start(tempData,tempDataIndex);
          //ROS_INFO("tempDataIndex2:  %d", tempDataIndex);
      }else if(data_len == -1){
          // Eğer checksum doğru değilse
          tempData[0] = '0'; // ilk mesajı boz
          tempDataIndex = find_message_start(tempData,tempDataIndex);
      }
      //while_cnt++;
  }
  return _ret_val;

}

void 
MRP2_Serial::array_chopper(uint8_t *buf, int start, int end) {
  //memcpy(&tempData[0], &buf[start], end);
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
  int ret = 0;
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
  int ret = 0;
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
  int checksum = 0;
  int i = 0;
  for(i = 0; i<size; i++) // Sum-up all bytes
  {
      checksum = checksum + buf[i];
  }
  if(size > 3)
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
          //ROS_INFO("char found at index: %d\n", start);
          //ROS_INFO("kopyalanacak sayı: %d",lastIndex+1);
          //printf("oldDataIndex: %d\n", old_tempDataIndex);
  array_chopper(buf, start, lastIndex+1);// '$' öncesini çöpe at
  
  return lastIndex;
};

int 
MRP2_Serial::execute_command(uint8_t *buf) {
  //std_msgs::Int32MultiArray array;
  //std_msgs::Int32 int_msg;
  //array.data.clear();
  //printf("\nReturn Command is %d\n", buf[1]);
  //print_array(buf,10);

  if(buf[1] == getBUMPERS)
  {
    _bumpers.clear();
    _bumpers.push_back((buf[4] >> 3) & 0x01);
    _bumpers.push_back((buf[4] >> 2) & 0x01);
    _bumpers.push_back((buf[4] >> 1) & 0x01);
    _bumpers.push_back(buf[4] & 0x01);

  }

  if(buf[1] == 52)
  {
    _ack_data = buf[4];
  }

  if(buf[1] == getSPEEDS)
  {
      int l_speed = buf[5];
      l_speed += (buf[6] << 8);
      if(buf[4] == 1)
      {
        l_speed *= -1;
      }
        

      int r_speed= buf[8];
      r_speed += (buf[9] << 8);
      if(buf[7] == 1)
      {
        r_speed *= -1;
      }
      
      _speed_l = l_speed;
      _speed_r = r_speed;
      _speeds.clear();
      _speeds.push_back(_speed_l);
      _speeds.push_back(_speed_r);

  }

  if(buf[1] == getSPEED_L)
  {
      int l_speed = buf[5];
      l_speed += (buf[6] << 8);
      if(buf[4] == 1)
      {
        l_speed *= -1;
      }
      
      _speed_l = l_speed;
      _speeds.clear();
      _speeds.push_back(_speed_l);
      _speeds.push_back(_speed_r);

  }

  if(buf[1] == getSPEED_R)
  {
      int r_speed = buf[5];
      r_speed += (buf[6] << 8);
      if(buf[4] == 1)
      {
        r_speed *= -1;
      }
      
      _speed_r = r_speed;
      _speeds.clear();
      _speeds.push_back(_speed_l);
      _speeds.push_back(_speed_r);

  }

  if(buf[1] == getPARAM_KP_L)
  {
    int i = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24);
    _Kp_l = (double)i/10000;
  }

  if(buf[1] == getPARAM_KI_L)
  {
    int i = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24);
    _Ki_l = (double)i/10000;
  }

  if(buf[1] == getPARAM_KD_L)
  {
    int i = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24);
    _Kd_l = (double)i/10000;
  }

  if(buf[1] == getPARAM_KP_R)
  {
    int i = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24);
    _Kp_r = (double)i/10000;
  }

  if(buf[1] == getPARAM_KI_R)
  {
    int i = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24);
    _Ki_r = (double)i/10000;
  }

  if(buf[1] == getPARAM_KD_R)
  {
    int i = buf[4] + (buf[5] << 8) + (buf[6] << 16) + (buf[7] << 24);
    _Kd_r = (double)i/10000;
  }

  if(buf[1] == getPARAM_IMAX_L)
  {
    _imax_l = buf[4] + (buf[5] << 8);
    _imax.clear();
    _imax.push_back(_imax_l);
    _imax.push_back(_imax_r);
  }

  if(buf[1] == getPARAM_IMAX_R)
  {
    _imax_r = buf[4] + (buf[5] << 8);
    _imax.clear();
    _imax.push_back(_imax_l);
    _imax.push_back(_imax_r);
  }

  if(buf[1] == getMAXSPEED_FWD)
  {
    _maxspeed_fwd = buf[4];
    _maxspeed_fwd += (buf[5] << 8);
  }

  if(buf[1] == getMAXSPEED_REV)
  {
    _maxspeed_rev = buf[4];
    _maxspeed_rev += (buf[5] << 8);
  }

  if(buf[1] == getMAXACCEL)
  {
    _maxaccel = buf[4];
    _maxaccel += (buf[5] << 8);
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

  return buf[1];

};

void 
MRP2_Serial::print_array(uint8_t *buf, int length) {
  printf("Array: ");
  int i = 0;
  for (i = 0; i < length; i++)
  {
      printf("%02x ",buf[i] );
  }
  printf("\n");
};
