#pragma once
#include "rsdef.h"
#include "information_sever.h"
void callback_RealTime_RoadPoint(const aubo_robot_namespace::wayPoint_S* wayPoint, void* arg);

void callback_RealTime_Joint_status(const aubo_robot_namespace::JointStatus* jointStatus, int size, void* arg);

void callback_RealTime_End_speed(double speed, void* arg);

void callback_Robot_event(const aubo_robot_namespace::RobotEventInfo* eventInfo, void* arg);

void set_all_callback(RSHD rshd, websocket_server* server);

//登陆机械臂
bool try_login(RSHD& rshd, const char* addr, int port);

//退出登陆
bool try_logout(RSHD rshd);

//启动机械臂(必须连接真实机械臂）
bool try_robotStartup(RSHD rshd);

class recive_data
{
public:
	recive_data(RSHD rshd, websocket_server* server);
	~recive_data();

	void start_recive_data();
	void stop_recive_data();

	std::atomic<bool> Running;
	std::atomic<bool> isRunning;
	std::thread recive_data_thread;
	RSHD Rshd;
	websocket_server* Server;

};