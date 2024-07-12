#include "control.h"
#include "nlohmann/json.hpp"



void contorl_feedback(std::string msg, websocket_server* server) {
	std::string json_msg = control_move(msg);
	server->send_message(json_msg);
}



std::string control_move(std::string msg) {
	nlohmann::json data = nlohmann::json::parse(msg);
	std::string type = data["type"];
	nlohmann::json result;
	if (type == "joint_control")
	{
		try
		{
			double joint_angle[6];
			for (size_t i = 0; i < 6; i++)
				joint_angle[i] = data["value"][i];
			std::string outcome = try_joint_control(g_rshd, joint_angle);
			result["type"] = int(!outcome.empty());
			result["errMsg"] = outcome;
			result["request_id"] = data["request_id"];
			return result.dump();
		}
		catch (const std::exception&)
		{
			std::cout << "joint_control_erro_value :" << msg << std::endl;
		}
	}
	else if (type == "position_control")
	{
		try
		{
			Pos tag;
			tag.x = data["value"][0];
			tag.y = data["value"][1];
			tag.z = data["value"][2];
			std::string outcome = try_position_control(g_rshd, tag);
			result["type"] = int(!outcome.empty());
			result["errMsg"] = outcome;
			result["request_id"] = data["request_id"];
			return result.dump();
		}
		catch (const std::exception&)
		{
			std::cout << "position_control_erro_value :" << msg << std::endl;
		}
	}
	else if (type == "rpy_control")
	{
		return "error";
	}
	else {
		std::cout << "未知请求: " << type << " msg: " << msg << std::endl;
	}
	return "error";
}



std::string try_position_control(RSHD rshd,Pos pos)
{

	//获取当前路点信息
	aubo_robot_namespace::wayPoint_S wayPoint;

	//逆解位置信息
	aubo_robot_namespace::wayPoint_S targetPoint;

	//目标位置对应的关节角
	double targetRadian[ARM_DOF] = { 0 };

	if (RS_SUCC == rs_get_current_waypoint(rshd, &wayPoint))
	{
		//参考当前姿态逆解得到六个关节角
		if (RS_SUCC == rs_inverse_kin(rshd, wayPoint.jointpos, &pos, &wayPoint.orientation, &targetPoint))
		{
			//将得到目标位置,将6关节角度设置为用户给定的角度（必须在+-175度）
			targetRadian[0] = targetPoint.jointpos[0];
			targetRadian[1] = targetPoint.jointpos[1];
			targetRadian[2] = targetPoint.jointpos[2];
			targetRadian[3] = targetPoint.jointpos[3];
			targetRadian[4] = targetPoint.jointpos[4];
			targetRadian[5] = targetPoint.jointpos[5];

			//轴动到目标位置
			if (RS_SUCC == rs_move_line(rshd, targetRadian, false))
			{
				std::cout << "at target" << std::endl;
				return std::string();
			}
			else
			{
				return std::string("move joint error");
			}

		}
		else
		{
			return std::string("ik failed");
		}

	}
	else
	{
		return std::string("get current waypoint error");
	}


	return std::string("error");
}

std::string try_joint_control(RSHD rshd, double * joint_angle) {

	if (RS_SUCC == rs_move_joint(rshd, joint_angle, false)) {
		return std::string();
	}
	else {
		return std::string("error");
	}
}
