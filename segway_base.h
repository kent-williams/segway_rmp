
#ifndef SEGWAY_BASE_H
#define SEGWAY_BASE_H

#include <stdint.h>


class SegwayBase
{
public:
  SegwayRMP(const char *device);
  virtual ~SegwayRMP();

  virtual void send_vel_cmd(float x_vel, float yaw_rate);
  virtual bool poll(float timeout_secs = 1.0);
}
