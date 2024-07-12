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
//推送消息类型：
//{
//"joint_angle": [10,20,30,40,50,60] , // 6个关节角度，单位为度
//"position" : [1,2,3] ,  // 机械臂末端位置，单位为米
//"rpy" : [1,2,3]   // 机械臂姿态，单位为度
//}

void callback_RealTime_RoadPoint(const aubo_robot_namespace::wayPoint_S* wayPoint, void* arg)
{
	
	nlohmann::json data;

	// 位置信息
	data["position"][0] = wayPoint->cartPos.position.x;
	data["position"][1] = wayPoint->cartPos.position.y;
	data["position"][2] = wayPoint->cartPos.position.z;

	// 姿态信息
	Ori in_ori; Rpy tag_rpy;
	in_ori.w = wayPoint->orientation.w;
	in_ori.x = wayPoint->orientation.x;
	in_ori.y = wayPoint->orientation.y;
	in_ori.z = wayPoint->orientation.z;

	ToEulerAngles(in_ori,tag_rpy);

	data["rpy"][0] = tag_rpy.rx / M_PI * 180;
	data["rpy"][1] = tag_rpy.ry / M_PI * 180;
	data["rpy"][2] = tag_rpy.rz / M_PI * 180;

	// 关节角信息
	for (int i = 0; i < ARM_DOF; ++i) {
		data["joint_angle"].push_back(wayPoint->jointpos[i]);
		//std::cout << "joint_angle" << i << " = " << wayPoint->jointpos[i];
	}

	// 信息发送，默认向最新建立起连接的客户端发送消息
	websocket_server* server_handle = static_cast<websocket_server*>(arg);
	if (!server_handle->handle_set.empty()){
		server_handle->send_message(data.dump());
	}
}

void set_callback(RSHD rshd, websocket_server *server, bool open_callback) {
	//设置路点信息推送
	if (RS_SUCC == rs_enable_push_realtime_roadpoint(rshd, open_callback))
	{
		if (RS_SUCC != rs_setcallback_realtime_roadpoint(rshd, open_callback ? callback_RealTime_RoadPoint:nullptr , server))
		{
			std::cerr << "设置实时路点信息推送回调函数 failed" << std::endl;
		}
		else {
			std::cout << "设置实时路点信息推送回调函数 succ" << std::endl;
		}
	}
	else
		std::cerr << "设置启用实时路点信息推送 failed!" << std::endl;
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

