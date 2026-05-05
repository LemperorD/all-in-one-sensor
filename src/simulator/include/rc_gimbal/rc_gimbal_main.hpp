#ifndef RC_GIMBAL_MAIN_HPP
#define RC_GIMBAL_MAIN_HPP

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <linux/joystick.h>
#include <thread>

namespace rc_gimbal
{

typedef struct js_map  
{  
    int     time;  
    int     a;  
    int     b;  
    int     x;  
    int     y;  
    int     lb;  
    int     rb;  
    int     start;  
    int     back;  
    int     home;  
    int     lo;  
    int     ro;  

    int     lx;  
    int     ly;  
    int     rx;  
    int     ry;  
    int     lt;  
    int     rt;  
    int     xx;  
    int     yy;  

} js_map_t;

class RcGimbalMain
{
public: // constructor and destructor
  explicit RcGimbalMain(const char *file_name);
  ~RcGimbalMain();

public: // 公共接口
  int get_fd() const { return js_fd_; }

private: // 打开和关闭设备的接口
  int js_open(const char *file_name);
  int js_map_read(int js_fd, js_map_t *map);

private: // 轮询线程
  void timerThread();
  void timerCallback();
  void tryReconnect();

private:
  char *file_name_;
  int js_fd_ = -1;
  std::thread timer_thread_;
  std::chrono::steady_clock::time_point last_received_time_;
  std::chrono::steady_clock::time_point last_reconnect_time_;

};

} // namespace rc_gimbal

#endif // RC_GIMBAL_MAIN_HPP