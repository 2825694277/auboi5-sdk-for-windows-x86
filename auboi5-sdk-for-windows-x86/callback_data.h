#pragma once
#include "rsdef.h"
#include "information_sever.h"
#include "main_sever.h"
void callback_RealTime_RoadPoint(const aubo_robot_namespace::wayPoint_S* wayPoint, void* arg);

//登陆机械臂
bool try_login(RSHD& rshd, const char* addr, int port);

//退出登陆
bool try_logout(RSHD rshd);

//启动机械臂(必须连接真实机械臂）
bool try_robotStartup(RSHD rshd);

void ToEulerAngles(const Ori q, Rpy& angles);

void set_callback(RSHD rshd, websocket_server* server, bool open_callback);