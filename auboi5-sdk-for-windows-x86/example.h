#pragma once

#include <string>
#include "rsdef.h"

using namespace std;

//��ӡ·����Ϣ
void printRoadPoint(const aubo_robot_namespace::wayPoint_S  *wayPoint);

void callback_RealTimeRoadPoint(const aubo_robot_namespace::wayPoint_S  *wayPoint, void *arg);
//
//��½��е��
bool example_login(RSHD &rshd, const char * addr, int port);

//�˳���½
bool example_logout(RSHD rshd);

//������е��(����������ʵ��е�ۣ�
bool example_robotStartup(RSHD rshd);
//
//int rs_move_rotate(RSHD rshd, const CoordCalibrate* user_coord, const Move_Rotate_Axis* rotate_axis, double rotate_angle, bool isblock = true);



//��е���ᶯ����
bool example_moveJ(RSHD rshd);

//��е�۱��ֵ�ǰ��ֱ̬���˶�����
bool example_moveL(RSHD rshd);

//��е�۹켣�˶�����
void example_moveP(RSHD rshd);

//��е����������
void example_ik_fk(RSHD rshd);

//��е�ۿ��ƹ�IO����(����������ʵ��е�ۣ�
void example_boardIO(RSHD rshd);

//��е�۹��߶�IO����(����������ʵ��е�ۣ�
void example_ToolIO(RSHD rshd);

//�����̬����
void demo_relativeOri(RSHD rshd);

//ʵʱ·����Ϣ�ص���������
bool example_callbackRobotRoadPoint(RSHD rshd);

//����ģʽ�µ��ᶯ�ӿ�
void example_follow_mode_movej(RSHD rshd);

void example_get_diagnosis(RSHD rshd);

int  example_forceFun(RSHD rshd);


int  example_forceFun_1(RSHD rshd);
//�رջ�е�ۣ�����������ʵ��е�ۣ�
bool example_robotShutdown(RSHD rshd);
void example_move_joint_to(RSHD rshd);
