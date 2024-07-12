#pragma once
#include "rsdef.h"
#include "information_sever.h"
#include "main_sever.h"
void callback_RealTime_RoadPoint(const aubo_robot_namespace::wayPoint_S* wayPoint, void* arg);

//��½��е��
bool try_login(RSHD& rshd, const char* addr, int port);

//�˳���½
bool try_logout(RSHD rshd);

//������е��(����������ʵ��е�ۣ�
bool try_robotStartup(RSHD rshd);

void ToEulerAngles(const Ori q, Rpy& angles);

void set_callback(RSHD rshd, websocket_server* server, bool open_callback);