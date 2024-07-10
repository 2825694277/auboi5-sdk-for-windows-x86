#include "callback_data.h"
#include "nlohmann/json.hpp"
#include "information_sever.h"
#include <thread>
#include <atomic>
#include <condition_variable>
#include <chrono>
/********************************************************************
	function:	try_login
	purpose :	登陆机械臂
	param   :	rshd 输出上下文句柄
				addr 机械臂服务器地址
				port 机械臂服务器端口
	return  :	true 成功 false 失败
*********************************************************************/
bool try_login(RSHD& rshd, const char* addr, int port)
{
	bool result = false;

	rshd = RS_FAILED;

	//初始化接口库
	if (rs_initialize() == RS_SUCC)
	{
		//创建上下文
		if (rs_create_context(&rshd) == RS_SUCC)
		{
			//登陆机械臂服务器
			if (rs_login(rshd, addr, port) == RS_SUCC)
			{
				result = true;
				//登陆成功
				std::cout << "login succ" << std::endl;
			}
			else
			{
				//登陆失败
				std::cerr << "login failed" << std::endl;
			}
		}
		else
		{
			//创建上下文失败
			std::cerr << "rs_create_context error" << std::endl;
		}
	}
	else
	{
		//初始化接口库失败
		std::cerr << "rs_initialize error" << std::endl;
	}

	return result;
}

/********************************************************************
	function:	try_logout
	purpose :	退出登陆
	param   :	rshd 上下文句柄

	return  :	true 成功 false 失败
*********************************************************************/
bool try_logout(RSHD rshd)
{
	return rs_logout(rshd) == RS_SUCC ? true : false;
}

/********************************************************************
	function:	try_robotStartup
	purpose :	启动机械臂(必须连接真实机械臂）
	param   :	rshd 上下文句柄

	return  :	true 成功 false 失败
*********************************************************************/
bool try_robotStartup(RSHD rshd)
{
	bool result = false;

	//工具的动力学参数和运动学参数
	ToolDynamicsParam tool_dynamics = { 0 };
	//机械臂碰撞等级
	uint8 colli_class = 6;
	//机械臂启动是否读取姿态（默认开启）
	bool read_pos = true;
	//机械臂静态碰撞检测（默认开启）
	bool static_colli_detect = true;
	//机械臂最大加速度（系统自动控制，默认为30000)
	int board_maxacc = 30000;
	//机械臂服务启动状态
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

//路点信息推送
//typedef struct
//{
//	cartesianPos_U cartPos;            // 机械臂的位置信息(x,y,z)
//
//	Ori            orientation;        // 机械臂姿态信息,四元素(w,x,y,z)
//
//	double         jointpos[ARM_DOF];  // 机械臂关节角信息
//
//}wayPoint_S;
void callback_RealTime_RoadPoint(const aubo_robot_namespace::wayPoint_S* wayPoint, void* arg)
{
	
	nlohmann::json data;
	data["package_type"] = "wayPoint_S";

	// 位置信息
	data["content"]["position"]["x"] = wayPoint->cartPos.position.x;
	data["content"]["position"]["y"] = wayPoint->cartPos.position.y;
	data["content"]["position"]["z"] = wayPoint->cartPos.position.z;

	// 姿态信息
	data["content"]["orientation"]["w"] = wayPoint->orientation.w;
	data["content"]["orientation"]["x"] = wayPoint->orientation.x;
	data["content"]["orientation"]["y"] = wayPoint->orientation.y;
	data["content"]["orientation"]["z"] = wayPoint->orientation.z;
	// 关节角信息
	for (int i = 0; i < ARM_DOF; ++i) {
		data["content"]["joint_positions"][i] = wayPoint->jointpos[i];
	}

	// 信息发送，默认向最新建立起连接的客户端发送消息
	websocket_server* server_handle = static_cast<websocket_server*>(arg);
	if (!server_handle->handle_set.empty()){
		server_handle->send_message(server_handle->handle_set.back(), data.dump());
	}
}

//关节信息推送
//typedef struct PACKED
//{
//	int    jointCurrentI;       // 关节电流    Current of driver
//	int    jointSpeedMoto;      // 关节速度    Speed of driver
//	float  jointPosJ;           // 关节角      Current position in radian
//	float  jointCurVol;         // 关节电压    Rated voltage of motor. Unit: mV
//	float  jointCurTemp;        // 当前温度    Current temprature of joint
//	int    jointTagCurrentI;    // 电机目标电流 Target current of motor
//	float  jointTagSpeedMoto;   // 电机目标速度 Target speed of motor
//	float  jointTagPosJ;        // 目标关节角　 Target position of joint in radian
//	uint16 jointErrorNum;       // 关节错误码   Joint error of joint num
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

	// 信息发送，默认向最新建立起连接的客户端发送消息
	websocket_server* server_handle = static_cast<websocket_server*>(arg);
	if (!server_handle->handle_set.empty()) {
		server_handle->send_message(server_handle->handle_set.back(), data.dump());
	}
}

//末端速度信息推送
void callback_RealTime_End_speed(double speed, void* arg)
{
	nlohmann::json data;
	data["package_type"] = "JointStatus";
	data["content"]["speed"] = speed;

	// 信息发送，默认向最新建立起连接的客户端发送消息
	websocket_server* server_handle = static_cast<websocket_server*>(arg);
	if (!server_handle->handle_set.empty()) {
		server_handle->send_message(server_handle->handle_set.back(), data.dump());
	}
}

//机械臂事件信息推送
//typedef struct {
//	RobotEventType  eventType;       //事件类型号
//	int             eventCode;       //
//	std::string     eventContent;    //事件内容
//}RobotEventInfo;
void callback_Robot_event(const aubo_robot_namespace::RobotEventInfo* eventInfo, void* arg)
{
	nlohmann::json data;
	data["package_type"] = "JointStatus";
	
	data["content"]["RobotEventType"] = eventInfo->eventType;
	data["content"]["int"] = eventInfo->eventCode;
	data["content"]["std::string"] = eventInfo->eventContent;

	// 信息发送，默认向最新建立起连接的客户端发送消息
	websocket_server* server_handle = static_cast<websocket_server*>(arg);
	if (!server_handle->handle_set.empty()) {
		server_handle->send_message(server_handle->handle_set.back(), data.dump());
	}

}

void set_all_callback(RSHD rshd, websocket_server *server) {
	int failed_times = 4;

	//设置路点信息推送
	if (RS_SUCC == rs_enable_push_realtime_roadpoint(rshd, true))
	{
		if (!rs_setcallback_realtime_roadpoint(rshd, callback_RealTime_RoadPoint, server))
		{
			std::cerr << "设置实时路点信息推送回调函数 failed" << std::endl;
		}
		else {
			failed_times--;
		}
	}
	else
		std::cerr << "设置启用实时路点信息推送 failed!" << std::endl;


	//设置关节信息推送
	if (RS_SUCC == rs_enable_push_realtime_joint_status(rshd, server))
	{
		if (!rs_setcallback_realtime_joint_status(rshd, callback_RealTime_Joint_status, server))
		{

			std::cerr << "设置实时关节信息推送回调函数 failed" << std::endl;
		}
		else {
			failed_times--;
		}
	}
	else
		std::cerr << "设置启用实时关节信息推送 failed!" << std::endl;

	//设置末端速度信息推送
	if (RS_SUCC == rs_enable_push_realtime_end_speed(rshd, true))
	{
		if (!rs_setcallback_realtime_end_speed(rshd, callback_RealTime_End_speed, server))
		{
			std::cerr << "设置实时末端速度信息推送回调函数 failed" << std::endl;
		}
		else {
			failed_times--;
		}
	}
	else
		std::cerr << "设置启用实时末端速度信息推送 failed!" << std::endl;


	//设置机械臂事件信息推送
	if (!rs_setcallback_robot_event(rshd, callback_Robot_event, server))
	{
		std::cerr << "设置机械臂事件信息推送回调函数 failed" << std::endl;
	}
	else {
		failed_times--;
	}

	if (!failed_times){
		std::cout << "所有消息事件已绑定函数成功" << std::endl;
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
		std::cout << "已经开始接收信息无需重复开启" << std::endl;
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
	std::cout << "开始接收消息" << std::endl;
}

void recive_data::stop_recive_data()
{
	this->Running = false;
	if (this->recive_data_thread.joinable())
	{
		this->recive_data_thread.join();
		std::cout << "已停止接收消息" << std::endl;
	}
}
