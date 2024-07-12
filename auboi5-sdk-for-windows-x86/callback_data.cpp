#include "callback_data.h"
#include "nlohmann/json.hpp"
#include "information_sever.h"
#include <thread>
#include <atomic>
#include <condition_variable>
#include <chrono>
/********************************************************************
	function:	try_login
	purpose :	��½��е��
	param   :	rshd ��������ľ��
				addr ��е�۷�������ַ
				port ��е�۷������˿�
	return  :	true �ɹ� false ʧ��
*********************************************************************/
bool try_login(RSHD& rshd, const char* addr, int port)
{
	bool result = false;

	rshd = RS_FAILED;

	//��ʼ���ӿڿ�
	if (rs_initialize() == RS_SUCC)
	{
		//����������
		if (rs_create_context(&rshd) == RS_SUCC)
		{
			//��½��е�۷�����
			if (rs_login(rshd, addr, port) == RS_SUCC)
			{
				result = true;
				//��½�ɹ�
				std::cout << "login succ" << std::endl;
			}
			else
			{
				//��½ʧ��
				std::cerr << "login failed" << std::endl;
			}
		}
		else
		{
			//����������ʧ��
			std::cerr << "rs_create_context error" << std::endl;
		}
	}
	else
	{
		//��ʼ���ӿڿ�ʧ��
		std::cerr << "rs_initialize error" << std::endl;
	}

	return result;
}

/********************************************************************
	function:	try_logout
	purpose :	�˳���½
	param   :	rshd �����ľ��

	return  :	true �ɹ� false ʧ��
*********************************************************************/
bool try_logout(RSHD rshd)
{
	return rs_logout(rshd) == RS_SUCC ? true : false;
}

/********************************************************************
	function:	try_robotStartup
	purpose :	������е��(����������ʵ��е�ۣ�
	param   :	rshd �����ľ��

	return  :	true �ɹ� false ʧ��
*********************************************************************/
bool try_robotStartup(RSHD rshd)
{
	bool result = false;

	//���ߵĶ���ѧ�������˶�ѧ����
	ToolDynamicsParam tool_dynamics = { 0 };
	//��е����ײ�ȼ�
	uint8 colli_class = 6;
	//��е�������Ƿ��ȡ��̬��Ĭ�Ͽ�����
	bool read_pos = true;
	//��е�۾�̬��ײ��⣨Ĭ�Ͽ�����
	bool static_colli_detect = true;
	//��е�������ٶȣ�ϵͳ�Զ����ƣ�Ĭ��Ϊ30000)
	int board_maxacc = 30000;
	//��е�۷�������״̬
	ROBOT_SERVICE_STATE state = ROBOT_SERVICE_READY;

	if (rs_robot_startup(rshd, &tool_dynamics, colli_class, read_pos, static_colli_detect, board_maxacc, &state)
		== RS_SUCC)
	{
		result = true;
		std::cout << "call robot startup succ, robot state:" << state << std::endl;
	}
	else
	{
		std::cerr << "robot startup failed" << std::endl;
	}

	return result;
}

//·����Ϣ����
//typedef struct
//{
//	cartesianPos_U cartPos;            // ��е�۵�λ����Ϣ(x,y,z)
//
//	Ori            orientation;        // ��е����̬��Ϣ,��Ԫ��(w,x,y,z)
//
//	double         jointpos[ARM_DOF];  // ��е�۹ؽڽ���Ϣ
//
//}wayPoint_S;
//������Ϣ���ͣ�
//{
//"joint_angle": [10,20,30,40,50,60] , // 6���ؽڽǶȣ���λΪ��
//"position" : [1,2,3] ,  // ��е��ĩ��λ�ã���λΪ��
//"rpy" : [1,2,3]   // ��е����̬����λΪ��
//}

void callback_RealTime_RoadPoint(const aubo_robot_namespace::wayPoint_S* wayPoint, void* arg)
{
	
	nlohmann::json data;

	// λ����Ϣ
	data["position"][0] = wayPoint->cartPos.position.x;
	data["position"][1] = wayPoint->cartPos.position.y;
	data["position"][2] = wayPoint->cartPos.position.z;

	// ��̬��Ϣ
	Ori in_ori; Rpy tag_rpy;
	in_ori.w = wayPoint->orientation.w;
	in_ori.x = wayPoint->orientation.x;
	in_ori.y = wayPoint->orientation.y;
	in_ori.z = wayPoint->orientation.z;

	ToEulerAngles(in_ori,tag_rpy);

	data["rpy"][0] = tag_rpy.rx / M_PI * 180;
	data["rpy"][1] = tag_rpy.ry / M_PI * 180;
	data["rpy"][2] = tag_rpy.rz / M_PI * 180;

	// �ؽڽ���Ϣ
	for (int i = 0; i < ARM_DOF; ++i) {
		data["joint_angle"].push_back(wayPoint->jointpos[i]);
		//std::cout << "joint_angle" << i << " = " << wayPoint->jointpos[i];
	}

	// ��Ϣ���ͣ�Ĭ�������½��������ӵĿͻ��˷�����Ϣ
	websocket_server* server_handle = static_cast<websocket_server*>(arg);
	if (!server_handle->handle_set.empty()){
		server_handle->send_message(data.dump());
	}
}

void set_callback(RSHD rshd, websocket_server *server, bool open_callback) {
	//����·����Ϣ����
	if (RS_SUCC == rs_enable_push_realtime_roadpoint(rshd, open_callback))
	{
		if (RS_SUCC != rs_setcallback_realtime_roadpoint(rshd, open_callback ? callback_RealTime_RoadPoint:nullptr , server))
		{
			std::cerr << "����ʵʱ·����Ϣ���ͻص����� failed" << std::endl;
		}
		else {
			std::cout << "����ʵʱ·����Ϣ���ͻص����� succ" << std::endl;
		}
	}
	else
		std::cerr << "��������ʵʱ·����Ϣ���� failed!" << std::endl;
}


void ToEulerAngles(const Ori q,Rpy &angles) {


	// roll (x-axis rotation)
	double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
	angles.rx = std::atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = 2 * (q.w * q.y - q.z * q.x);
	if (std::abs(sinp) >= 1)
		angles.ry = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		angles.ry = std::asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
	angles.rz = std::atan2(siny_cosp, cosy_cosp);

}

