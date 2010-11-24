#include <limits.h>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include "segway_rmp.h"
#include "canio.h"
#include "canio_kvaser.h"
#include "rmp_frame.h"

using std::string;

static const float MAX_X_VEL = 1.2;
static const float MAX_YAW_RATE = 0.4;
/*static const int RMP_MAX_TRANS_VEL_COUNT = 1176;
static const int RMP_MAX_ROT_VEL_COUNT = 1024;*/
static const uint8_t USB_DLE = 0x10;
static const uint8_t USB_STX = 0x02;
static const uint8_t USB_ETX = 0x03;

SegwayRMP::SegwayRMP()
: odom_init_complete(false), 
  integrated_x(0), integrated_y(0), integrated_yaw(0),
  parser_state(START_DLE), incoming_write_pos(0)
{
  /*
  can = dgc_usbcan_initialize(device);
  if (!can)
    throw std::runtime_error(string("couldn't initialize canbus on ") + 
                             string(device));
  */


// Old can
/*
  serial_port = new LightweightSerial(device, 500000);
  if (!serial_port->is_ok())
    throw std::runtime_error(string("couldn't initialize canbus on ") + 
                             string(device));
  send_rmp_config_command(0x31); // set baud to 500k
  send_rmp_config_command('R'); // start RUN mode
  send_rmp_config_command(0x22); // set NORMAL mode
*/
  // New Canio
  if(CanBusSetup() < 0)
  {
    fprintf(stderr, "Can Bus Failed to Initialize!\n");
  }
  else
  {
    fprintf(stderr, "Can Bus Initialized Successfully!\n");
  }
}

SegwayRMP::~SegwayRMP()
{
  //dgc_usbcan_close(&can);
  //send_rmp_config_command(0x23); // set SLEEP mode
  delete canio;
}


/**********************************************
 * START HACK
 **********************************************/
int
SegwayRMP::CanBusSetup()
{
  this->canio = new CANIOKvaser;

  // start the CAN at 500 kpbs
  if(this->canio->Init(BAUD_500K) < 0)  
  {
    fprintf(stderr, "FAILED TO INIT CAN BUS!\n");
    return(-1);
  }
  CanPacket pkt;
  MakeStatusCommand(&pkt, (uint16_t)RMP_CAN_CMD_RST_INT,
				(uint16_t)RMP_CAN_RST_ALL);
  if(send_canio_packet(pkt) < 0)
  {
    fprintf(stderr, "Failed to send Can Init Packet!\n");
    return(-1);
  }
  
  return 0;
}



/* Creates a status CAN packet from the given arguments
 */
void
SegwayRMP::MakeStatusCommand(CanPacket* pkt, uint16_t cmd, uint16_t val)
{
	int16_t trans,rot;

	pkt->id = RMP_CAN_ID_COMMAND;
	pkt->PutSlot(2, cmd);

	// it was noted in the windows demo code that they
	// copied the 8-bit value into both bytes like this
	pkt->PutByte(6, val);
	pkt->PutByte(7, val);

	trans = 0;

	if(trans > RMP_MAX_TRANS_VEL_COUNT)
		trans = RMP_MAX_TRANS_VEL_COUNT;
	else if(trans < -RMP_MAX_TRANS_VEL_COUNT)
		trans = -RMP_MAX_TRANS_VEL_COUNT;

	rot = 0;

	if(rot > RMP_MAX_ROT_VEL_COUNT)
		rot = RMP_MAX_ROT_VEL_COUNT;
	else if(rot < -RMP_MAX_ROT_VEL_COUNT)
		rot = -RMP_MAX_ROT_VEL_COUNT;

	// put in the last speed commands as well
	pkt->PutSlot(0,(uint16_t)trans);
	pkt->PutSlot(1,(uint16_t)rot);

	if(cmd)
	{
		fprintf(stderr, "SEGWAYIO: STATUS: cmd: %02x val: %02x pkt: %s\n",
				cmd, val, pkt->toString());
	}
}




/* takes a player command (in host byte order) and turns it into CAN packets
 * for the RMP
 */
void
SegwayRMP::MakeVelocityCommand(CanPacket* pkt,
		uint16_t trans,
		uint16_t rot)
{
	pkt->id = RMP_CAN_ID_COMMAND;
	pkt->PutSlot(2, (uint16_t)RMP_CAN_CMD_NONE);
	pkt->PutSlot(0, (uint16_t)trans);
	pkt->PutSlot(1, (uint16_t)rot);
}

bool SegwayRMP::send_canio_packet(CanPacket& pkt)
{
	  return(canio->WritePacket(pkt) >= 0);
}

int SegwayRMP::CanBusRead()
{
	CanPacket pkt;
	int channel;
	int ret;
	rmp_frame_t data_frame[2];

	//static struct timeval last;
	//struct timeval curr;

	data_frame[0].ready = 0;
	data_frame[1].ready = 0;

	// read one cycle of data from each channel
	for(channel = 0; channel < DUALCAN_NR_CHANNELS; channel++)
	{
		ret=0;
		// read until we've gotten all 5 packets for this cycle on this channel
		while((ret = canio->ReadPacket(&pkt, channel)) >= 0)
		{
			// then we got a new packet...
			//printf("SEGWAYIO: pkt: %s\n", pkt.toString());

		        /*	
                        data_frame[channel].AddPacket(pkt);

			// if data has been filled in, then let's make it the latest data
			// available to player...
			if(data_frame[channel].IsReady())
			{
				// Only bother doing the data conversions for one channel.
				// It appears that the data on channel 0 is garbarge, so we'll read
				// from channel 1.
				if(channel == 1)
				{
					//UpdateData(&data_frame[channel]);
				}

				data_frame[channel].ready = 0;
				break;
			}
                        */
		}

		if (ret < 0)
		{
			printf("error (%d) reading packet on channel %d", ret, channel);
		}
	}

	return(0);
}

/**********************************************
 * End HACK
 **********************************************/

/*
bool SegwayRMP::send_rmp_packet(uint8_t *packet, uint32_t packet_size)
{
  unsigned char outgoing_buffer[100], checksum = 0, size = 0;
  outgoing_buffer[size++] = USB_DLE;
  outgoing_buffer[size++] = USB_STX;
  for (uint32_t i = 0; i < packet_size; i++)
  {
    checksum ^= packet[i];
    if (packet[i] == USB_DLE) // escape sequence
      outgoing_buffer[size++] = USB_DLE;
    outgoing_buffer[size++] = packet[i];
  }
  if (checksum == USB_DLE)
    outgoing_buffer[size++] = USB_DLE;
  outgoing_buffer[size++] = checksum;
  outgoing_buffer[size++] = USB_DLE;
  outgoing_buffer[size++] = USB_ETX;
  return (size == serial_port->write_block(outgoing_buffer, size));
}

bool SegwayRMP::send_rmp_config_command(uint8_t command)
{
  uint8_t pkt[5];
  pkt[0] = 0;
  pkt[1] = command | 0x80;
  return send_rmp_packet(pkt, 2);
}
*/

void SegwayRMP::send_vel_cmd(float x_vel, float yaw_rate)
//void build_vel_pkt(uint8_t *send_data)
{
  static uint8_t send_data[20];
  static int last_raw_yaw_rate = 0, last_raw_x_vel = 0; // for ramping
  static const int max_x_stepsize = 8, max_yaw_stepsize = 4;
  

  // ensure that when backing up, the velocity is enough to not stall the robot
  if (x_vel < -0.01 && x_vel > -0.25)
    x_vel = -0.25;
/*
  if (yaw_rate > 0.01 && yaw_rate < 0.2)
    yaw_rate = 0.2;
  else if (yaw_rate < -0.01 && yaw_rate > -0.2)
    yaw_rate = -0.2;
*/
  //g_last_cmd_vel.lock();
  // clamp inputs
  //double x_vel = g_last_cmd_vel.vel.vx;
  //double yaw_rate = g_last_cmd_vel.ang_vel.vz;

	if (x_vel > MAX_X_VEL)
		x_vel = MAX_X_VEL;
	else if (x_vel < -MAX_X_VEL)
		x_vel = -MAX_X_VEL;
	if (yaw_rate > MAX_YAW_RATE)
		yaw_rate = MAX_YAW_RATE;
	else if (yaw_rate < -MAX_YAW_RATE)
		yaw_rate = -MAX_YAW_RATE;
  //g_last_cmd_vel.unlock();

  // (calibrated for 22psi on the tires, with dual casters front/rear)
  int16_t raw_x_vel = (int16_t)rint(x_vel * 332.0f); 
	int16_t raw_yaw_rate = (int16_t)rint(yaw_rate * 415.0f);

	// threshold us again, just because I'm paranoid...
	if (raw_x_vel > RMP_MAX_TRANS_VEL_COUNT)
		raw_x_vel = RMP_MAX_TRANS_VEL_COUNT;
	else if (raw_x_vel < -RMP_MAX_TRANS_VEL_COUNT)
		raw_x_vel = -RMP_MAX_TRANS_VEL_COUNT;

	if (raw_yaw_rate > RMP_MAX_ROT_VEL_COUNT)
		raw_yaw_rate = RMP_MAX_ROT_VEL_COUNT;
	else if (raw_yaw_rate < -RMP_MAX_ROT_VEL_COUNT)
		raw_yaw_rate = -RMP_MAX_ROT_VEL_COUNT;

	// now have it ramp to the target velocity (so it doesn't jerk around)
	if (raw_x_vel > last_raw_x_vel + max_x_stepsize)
		raw_x_vel = last_raw_x_vel + max_x_stepsize;
	else if (raw_x_vel < last_raw_x_vel - max_x_stepsize)
		raw_x_vel = last_raw_x_vel - max_x_stepsize;
	last_raw_x_vel = raw_x_vel;

	if (raw_yaw_rate > last_raw_yaw_rate + max_yaw_stepsize)
		raw_yaw_rate = last_raw_yaw_rate + max_yaw_stepsize;
	else if (raw_yaw_rate < last_raw_yaw_rate - max_yaw_stepsize)
		raw_yaw_rate = last_raw_yaw_rate - max_yaw_stepsize;
	last_raw_yaw_rate = raw_yaw_rate;

  // now the bit-wrangling into the packet format
	unsigned short u_raw_x_vel = (unsigned short)raw_x_vel;
	unsigned short u_raw_yaw_rate = (unsigned short)raw_yaw_rate;
	
	
	// Hack'd
	CanPacket pkt;
	MakeVelocityCommand(&pkt, (uint16_t)u_raw_x_vel, (uint16_t)u_raw_yaw_rate);
	
	send_canio_packet(pkt);
	
	/*
	send_data[0] = (u_raw_x_vel >> 8) & 0xff;
	send_data[1] = u_raw_x_vel & 0xff;
	send_data[2] = (u_raw_yaw_rate >> 8) & 0xff;
	send_data[3] = u_raw_yaw_rate & 0xff;
  send_data[4] = 0;
  send_data[5] = 0;
  send_data[6] = 0;
  send_data[7] = 0;
  send_can_message(0x0413, send_data, 8);
  */
  //dgc_usbcan_send_can_message(can, 0x0413, send_data, 8); // can ID "COMMAND"
}

/*
bool SegwayRMP::send_can_message(uint32_t can_id, 
                                  uint8_t *message, uint32_t message_size)
{
  if (message_size != 8)
    return false;
  uint8_t pkt[100], size = 0;
  pkt[size++] = 0x80;
  pkt[size++] = (uint8_t)((can_id >> 24) & 0x1f);
  pkt[size++] = (uint8_t)((can_id >> 16) & 0xff);
  pkt[size++] = (uint8_t)((can_id >>  8) & 0xff);
  pkt[size++] = (uint8_t)((can_id      ) & 0xff);
  pkt[size++] = 0;
  pkt[size++] = 0;
  pkt[size++] = 0;
  pkt[size++] = message_size;
  //printf("sending %d byte message\n", message_size);
  for (uint32_t i = 0; i < message_size; i++)
    pkt[size++] = message[i];
  return send_rmp_packet(pkt, size);
}
*/

int SegwayRMP::rmp_diff(uint32_t from, uint32_t to)
{
	int diff1, diff2;
	diff1 = to - from;
	if (to > from)
		diff2 = -(from + UINT_MAX - to);
	else
		diff2 = UINT_MAX - from + to;
	if (abs(diff1) < abs(diff2))
		return diff1;
	else
		return diff2;
}

bool SegwayRMP::decode_incoming_message()
{
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < incoming_write_pos - 1; i++)
    checksum ^= incoming_message[i];
  if (checksum != incoming_message[incoming_write_pos - 1])
    return false; // failed checksum
  uint32_t can_id = ((uint32_t)incoming_message[1]      ) |
                    ((uint32_t)incoming_message[2] <<  8) |
                    ((uint32_t)incoming_message[3] << 16) |
                    ((uint32_t)incoming_message[4] << 24);
  //printf("im writepos = %d\n", incoming_write_pos);
  if (can_id == 0x0403)
  {
    const uint8_t * const message = incoming_message + 7; // payload start
    uint32_t msg_left = ((uint32_t)message[3] << 16) |
                        ((uint32_t)message[2] << 24) |
                        ((uint32_t)message[1] <<  0) |
                        ((uint32_t)message[0] <<  8);
    uint32_t msg_right = ((uint32_t)message[7] << 16) |
                         ((uint32_t)message[6] << 24) |
                         ((uint32_t)message[5] <<  0) |
                         ((uint32_t)message[4] <<  8);

    /*
    uint32_t msg_foreaft = ((uint32_t)message[3] << 16) |
                           ((uint32_t)message[2] << 24) |
                           ((uint32_t)message[1] <<  0) |
                           ((uint32_t)message[0] <<  8);
    uint32_t msg_yaw = ((uint32_t)message[7] << 16) |
                       ((uint32_t)message[6] << 24) |
                       ((uint32_t)message[5] <<  0) |
                       ((uint32_t)message[4] <<  8);
    */
    if (!odom_init_complete)
    {
      odom_init_complete = true;
      last_left = msg_left;
      last_right = msg_right;
    }

    int delta_left  = rmp_diff(last_left, msg_left);
    int delta_right = rmp_diff(last_right, msg_right) * 1.01;//25;
    double delta_lin = (delta_right + delta_left) / 2.0 / 40181.0;  //(double)delta_lin_raw / 40181;
    double delta_ang = (delta_right - delta_left) / 117031.0 * 2.0 * M_PI * 0.945;

    integrated_yaw += delta_ang;
    integrated_x += delta_lin * cos(integrated_yaw);
    integrated_y += delta_lin * sin(integrated_yaw);
/*
    printf("%d %d %.3f %.3f %.3f %.3f %.3f\n", 
           delta_left, delta_right, delta_lin, delta_ang,
           integrated_x, integrated_y, integrated_yaw);
*/
    last_left = msg_left;
    last_right = msg_right;
 
    //printf("%u %u\n", msg_foreaft, msg_yaw);
    //printf("%.3f %.3f %.3f\n", integrated_x, integrated_y, integrated_yaw);
    return true;
  }
  return false;
}

bool SegwayRMP::handle_incoming_byte(uint8_t b)
{
  switch (parser_state)
  {
    case START_DLE:
      if (b == USB_DLE)
        parser_state = START_STX;
      break;
    case START_STX:
      if (b == USB_STX)
        parser_state = IN_MESSAGE;
      incoming_write_pos = 0;
      break;
    case IN_MESSAGE:
      if (b == USB_DLE)
        parser_state = IN_MESSAGE_DLE;
      else
        if (incoming_write_pos < sizeof(incoming_message) - 1)
          incoming_message[incoming_write_pos++] = b;
      break;
    case IN_MESSAGE_DLE:
      if (b == USB_ETX)
      {
        parser_state = START_DLE;
        return decode_incoming_message(); // don't clutter the state machine here...
      }
      else
      {
        if (incoming_write_pos < sizeof(incoming_message) - 1)
          incoming_message[incoming_write_pos++] = b;
        parser_state = IN_MESSAGE;
      }
      break;
  }
  return false;
}

bool SegwayRMP::poll(float timeout_secs)
{
  bool found_odom_data = false;
  unsigned char incoming_data[100];
  int nread = 0;
  /*
  while ((nread = serial_port->read_block(incoming_data, sizeof(incoming_data))) > 0)
    for (int i = 0; i < nread; i++)
      found_odom_data |= handle_incoming_byte(incoming_data[i]);
   */
  return found_odom_data;
}

        /*
        int delta_lin_raw = rmp_diff(last_foreaft, rmp.foreaft);
        int delta_yaw_raw = rmp_diff(last_yaw, rmp.yaw);
        double delta_lin = (double)delta_lin_raw / RMP_COUNT_PER_M * 0.82;
        double delta_ang = (double)delta_yaw_raw / RMP_COUNT_PER_REV * 2 * M_PI * 0.921;// * 0.93;

        odom_x += delta_lin * cos(odom_yaw);
        odom_y += delta_lin * sin(odom_yaw);
        odom_yaw += delta_ang;

        static int odom_count = 0;
        //if (odom_count++ % 2 == 0) // send it at 50 hz
        {
            odom.pose.position.x  = odom_x;
            odom.pose.position.y  = odom_y;
            odom.pose.position.z  = 0;
            tf::Quaternion rot(odom_yaw, 0,0);
            tf::QuaternionTFToMsg(rot, odom.pose.orientation);
            odom.header.stamp = ros::Time::now();
            odom.header.stamp -= ros::Duration(0.4);
            odom_pub.publish(odom);
            tf.sendTransform(tf::Stamped<tf::Transform>(
              tf::Transform(rot, 
                tf::Point(odom.pose.position.x,
                          odom.pose.position.y, 0.0)),
              odom.header.stamp, "base_footprint", "odom"));
          }
        }
        last_foreaft = rmp.foreaft;
        last_yaw = rmp.yaw;
      }
}
*/
#if 0
int main(int argc, char **argv)
{
  ros::init(argc, argv, "segway_rmp");
  ros::NodeHandle nh;
  std::string port("/dev/ttyUSB2");
  nh.getParam("~port", port);
  ROS_DEBUG("opening canbus on %s\n", port.c_str());
	dgc_usbcan_p can = dgc_usbcan_initialize(port.c_str());
	if (!can)
	{
		ROS_FATAL("ahh couldn't open the can controller\n");
		ROS_BREAK();
	}
  tf::TransformBroadcaster tf;
	unsigned char message[100];
	int message_length;
	unsigned can_id;
  double last_send_time = ros::Time::now().toSec();
  robot_msgs::PoseStamped odom;
  odom.header.frame_id = "odom";
  ros::Publisher odom_pub = nh.advertise<robot_msgs::PoseStamped>("odom", 1);
  ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 10, cmd_vel_cb);
  double odom_x = 0, odom_y = 0, odom_yaw = 0;
  bool odom_init_complete = false;
  int last_foreaft = 0, last_yaw = 0;
  rmp_frame_t rmp;
  memset(&rmp, 0, sizeof(rmp));
  bool req_timeout = true;

  while(ros::ok())
  {
    ros::spinOnce();
    if (ros::Time::now().toSec() - last_send_time > 0.02)
    {
      double time_since_last_cmd = ros::Time::now().toSec() - g_last_req_time;
      if (time_since_last_cmd > 0.15)
        req_timeout = true;
      else
        req_timeout = false;

      if (!req_timeout)
      {
        uint8_t send_data[20];
        build_vel_pkt(send_data);
      }
      last_send_time = ros::Time::now().toSec();
    }

    }
		//else
	  usleep(500);
  }
  dgc_usbcan_close(&can);
  return 0;
}

#endif
