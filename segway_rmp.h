//#include "rmp_frame.h"
//extern "C" {
//#include "usbcan.h"
//}

#ifndef SEGWAY_APOX_H
#define SEGWAY_APOX_H

#include <stdint.h>

class DualCANIO;
class CanPacket;
class SegwayRMP
{
public:
  SegwayRMP();
  ~SegwayRMP();

  int CanBusRead();
  void send_vel_cmd(float x_vel, float yaw_rate);
  bool poll(float timeout_secs = 1.0);
  inline void get_last_odometry(float &x, float &y, float &yaw, uint32_t &left, uint32_t &right) const
  { x = integrated_x; y = integrated_y; yaw = integrated_yaw; left = last_left; right = last_right; }
protected:
	//dgc_usbcan_p can;
	//LightweightSerial *serial_port;
  DualCANIO *canio;
  bool odom_init_complete;
  float integrated_x, integrated_y, integrated_yaw;
  uint8_t incoming_message[100];
  enum { START_DLE, START_STX, IN_MESSAGE, IN_MESSAGE_DLE } parser_state;
  uint8_t incoming_write_pos;
  uint32_t last_left, last_right;


  // Hack'd
  void MakeStatusCommand(CanPacket* pkt, uint16_t cmd, uint16_t val);
  int CanBusSetup();
  void MakeVelocityCommand(CanPacket* pkt, uint16_t trans, uint16_t rot);
  bool send_canio_packet(CanPacket& pkt);

//  bool send_rmp_config_command(uint8_t command);
//  bool send_rmp_packet(uint8_t *packet, uint32_t size);
//  bool send_can_message(uint32_t can_id, uint8_t *message, uint32_t size);
  bool handle_incoming_byte(uint8_t b);
  bool decode_incoming_message();
  int rmp_diff(uint32_t from, uint32_t to);
};

#endif

