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
void callback_RealTime_RoadPoint(const aubo_robot_namespace::wayPoint_S* wayPoint, void* arg)
{
	
	nlohmann::json data;
	data["package_type"] = "wayPoint_S";

	// λ����Ϣ
	data["content"]["position"]["x"] = wayPoint->cartPos.position.x;
	data["content"]["position"]["y"] = wayPoint->cartPos.position.y;
	data["content"]["position"]["z"] = wayPoint->cartPos.position.z;

	// ��̬��Ϣ
	data["content"]["orientation"]["w"] = wayPoint->orientation.w;
	data["content"]["orientation"]["x"] = wayPoint->orientation.x;
	data["content"]["orientation"]["y"] = wayPoint->orientation.y;
	data["content"]["orientation"]["z"] = wayPoint->orientation.z;
	// �ؽڽ���Ϣ
	for (int i = 0; i < ARM_DOF; ++i) {
		data["content"]["joint_positions"][i] = wayPoint->jointpos[i];
	}

	// ��Ϣ���ͣ�Ĭ�������½��������ӵĿͻ��˷�����Ϣ
	websocket_server* server_handle = static_cast<websocket_server*>(arg);
	if (!server_handle->handle_set.empty()){
		server_handle->send_message(server_handle->handle_set.back(), data.dump());
	}
}

//�ؽ���Ϣ����
//typedef struct PACKED
//{
//	int    jointCurrentI;       // �ؽڵ���    Current of driver
//	int    jointSpeedMoto;      // �ؽ��ٶ�    Speed of driver
//	float  jointPosJ;           // �ؽڽ�      Current position in radian
//	float  jointCurVol;         // �ؽڵ�ѹ    Rated voltage of motor. Unit: mV
//	float  jointCurTemp;        // ��ǰ�¶�    Current temprature of joint
//	int    jointTagCurrentI;    // ���Ŀ����� Target current of motor
//	float  jointTagSpeedMoto;   // ���Ŀ���ٶ� Target speed of motor
//	float  jointTagPosJ;        // Ŀ��ؽڽǡ� Target position of joint in radian
//	uint16 jointErrorNum;       // �ؽڴ�����   Joint error of joint num
//
//}JointStatus;
void callback_RealTime_Joint_status(const aubo_robot_namespace::JointStatus* jointStatus, int size, void* arg)
{
	nlohmann::json data;
	data["package_type"] = "JointStatus";
	data["content"]["size"] = size;
	for (int i = 0; i < size; ++i) {
		nlohmann::json jointData;

		jointData["joint_current"] = jointStatus[i].jointCurrentI;
		jointData["joint_speed"] = jointStatus[i].jointSpeedMoto;
		jointData["joint_position"] = jointStatus[i].jointPosJ;
		jointData["joint_voltage"] = jointStatus[i].jointCurVol;
		jointData["joint_temperature"] = jointStatus[i].jointCurTemp;
		jointData["target_current"] = jointStatus[i].jointTagCurrentI;
		jointData["target_speed"] = jointStatus[i].jointTagSpeedMoto;
		jointData["target_position"] = jointStatus[i].jointTagPosJ;
		jointData["joint_error_code"] = jointStatus[i].jointErrorNum;

		data["content"]["joints"].push_back(jointData);
	}

	// ��Ϣ���ͣ�Ĭ�������½��������ӵĿͻ��˷�����Ϣ
	websocket_server* server_handle = static_cast<websocket_server*>(arg);
	if (!server_handle->handle_set.empty()) {
		server_handle->send_message(server_handle->handle_set.back(), data.dump());
	}
}

//ĩ���ٶ���Ϣ����
void callback_RealTime_End_speed(double speed, void* arg)
{
	nlohmann::json data;
	data["package_type"] = "JointStatus";
	data["content"]["speed"] = speed;

	// ��Ϣ���ͣ�Ĭ�������½��������ӵĿͻ��˷�����Ϣ
	websocket_server* server_handle = static_cast<websocket_server*>(arg);
	if (!server_handle->handle_set.empty()) {
		server_handle->send_message(server_handle->handle_set.back(), data.dump());
	}
}

//��е���¼���Ϣ����
//typedef struct {
//	RobotEventType  eventType;       //�¼����ͺ�
//	int             eventCode;       //
//	std::string     eventContent;    //�¼�����
//}RobotEventInfo;
void callback_Robot_event(const aubo_robot_namespace::RobotEventInfo* eventInfo, void* arg)
{
	nlohmann::json data;
	data["package_type"] = "JointStatus";
	
	data["content"]["RobotEventType"] = eventInfo->eventType;
	data["content"]["int"] = eventInfo->eventCode;
	data["content"]["std::string"] = eventInfo->eventContent;

	// ��Ϣ���ͣ�Ĭ�������½��������ӵĿͻ��˷�����Ϣ
	websocket_server* server_handle = static_cast<websocket_server*>(arg);
	if (!server_handle->handle_set.empty()) {
		server_handle->send_message(server_handle->handle_set.back(), data.dump());
	}

}

void set_all_callback(RSHD rshd, websocket_server *server) {
	int failed_times = 4;

	//����·����Ϣ����
	if (RS_SUCC == rs_enable_push_realtime_roadpoint(rshd, true))
	{
		if (!rs_setcallback_realtime_roadpoint(rshd, callback_RealTime_RoadPoint, server))
		{
			std::cerr << "����ʵʱ·����Ϣ���ͻص����� failed" << std::endl;
		}
		else {
			failed_times--;
		}
	}
	else
		std::cerr << "��������ʵʱ·����Ϣ���� failed!" << std::endl;


	//���ùؽ���Ϣ����
	if (RS_SUCC == rs_enable_push_realtime_joint_status(rshd, server))
	{
		if (!rs_setcallback_realtime_joint_status(rshd, callback_RealTime_Joint_status, server))
		{

			std::cerr << "����ʵʱ�ؽ���Ϣ���ͻص����� failed" << std::endl;
		}
		else {
			failed_times--;
		}
	}
	else
		std::cerr << "��������ʵʱ�ؽ���Ϣ���� failed!" << std::endl;

	//����ĩ���ٶ���Ϣ����
	if (RS_SUCC == rs_enable_push_realtime_end_speed(rshd, true))
	{
		if (!rs_setcallback_realtime_end_speed(rshd, callback_RealTime_End_speed, server))
		{
			std::cerr << "����ʵʱĩ���ٶ���Ϣ���ͻص����� failed" << std::endl;
		}
		else {
			failed_times--;
		}
	}
	else
		std::cerr << "��������ʵʱĩ���ٶ���Ϣ���� failed!" << std::endl;


	//���û�е���¼���Ϣ����
	if (!rs_setcallback_robot_event(rshd, callback_Robot_event, server))
	{
		std::cerr << "���û�е���¼���Ϣ���ͻص����� failed" << std::endl;
	}
	else {
		failed_times--;
	}

	if (!failed_times){
		std::cout << "������Ϣ�¼��Ѱ󶨺����ɹ�" << std::endl;
	}
}

recive_data::recive_data(RSHD rshd, websocket_server* server):Running(false), Rshd(rshd), Server(server),isRunning(false)
{

}

recive_data::~recive_data()
{
	Running = false;
	if (this->recive_data_thread.joinable())
	{
		this->recive_data_thread.join();
	}
}

void recive_data::start_recive_data()
{
	if (this->isRunning)
	{
		std::cout << "�Ѿ���ʼ������Ϣ�����ظ�����" << std::endl;
		return;
	}
	this->Running = true;
	recive_data_thread = std::thread([this] {
		this->isRunning = true;
		while (this->Running)
		{
			set_all_callback(this->Rshd, this->Server);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		this->isRunning = false;
	});
	std::cout << "��ʼ������Ϣ" << std::endl;
}

void recive_data::stop_recive_data()
{
	this->Running = false;
	if (this->recive_data_thread.joinable())
	{
		this->recive_data_thread.join();
		std::cout << "��ֹͣ������Ϣ" << std::endl;
	}
}
