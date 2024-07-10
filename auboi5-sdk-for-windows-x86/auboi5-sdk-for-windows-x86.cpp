// auboi5-sdk-for-windows-x86.cpp : �������̨Ӧ�ó������ڵ㡣
//

//#include "stdafx.h"
#include "auboi5-sdk-for-windows-x86.h"
#include "example.h"
#include <windows.h>
#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define ROBOT_ADDR "192.168.177.179"
#define ROBOT_PORT 8899



//��е�ۿ��������ľ��
RSHD g_rshd = -1;
bool main_open = false;
#ifdef main_open
int main(int argc, char* argv[])
{
	int nRetCode = 0;
	double forceLimit = 0; 
	double distLimit = 0;

	//��¼������
	if (example_login(g_rshd, ROBOT_ADDR, ROBOT_PORT))
	{
		//������е��(����������ʵ��е�ۣ�
		example_robotStartup(g_rshd);
		//example_forceFun_1(g_rshd);


		//�����̬����
		//demo_relativeOri(g_rshd);

		//��������
		//rs_project_startup(g_rshd);

		//��е���ᶯ����
		//example_moveJ(g_rshd);

		//��е�۱��ֵ�ǰ��ֱ̬���˶�����
		//example_moveL(g_rshd);
		for (int a = 0; a < 1; a = a )
		{
			 example_callbackRobotRoadPoint(g_rshd);
			//example_move_joint_to(g_rshd);
			Sleep(0.05);
		}
		//��е�۹켣�˶�����
		//example_moveP(g_rshd);

		//��е����������
		//example_ik_fk(g_rshd);

		//��е�ۿ��ƹ�IO����(����������ʵ��е�ۣ�
		//example_boardIO(g_rshd);

		//��е�۹��߶�IO����(����������ʵ��е�ۣ�
		//example_ToolIO(g_rshd);

		//ʵʱ·����Ϣ�ص���������
		//example_callbackRobotRoadPoint(g_rshd);

		//��ʱ2���룬�۲�ص�����
		//Sleep(2000);

		//�رջ�е�ۣ�����������ʵ��е�ۣ�
		//example_robotShutdown(g_rshd);

		//����ģʽ�µ��ᶯ����
		//example_follow_mode_movej(g_rshd);

		//example_get_diagnosis(g_rshd);

		//����ֹͣ 
		//rs_project_stop(g_rshd);

		//�˳���¼
		example_logout(g_rshd);
	}

	//����ʼ���ӿڿ�
	rs_uninitialize();

	std::cout<<"please enter to exit"<<std::endl;
	char c = getchar();

	return 0;
}
#endif