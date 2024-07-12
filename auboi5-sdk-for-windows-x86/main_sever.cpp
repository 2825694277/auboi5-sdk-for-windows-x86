#include "callback_data.h"
#include "main_sever.h"
#include "information_sever.h"
#include "control.h"
std::string ROBOT_ADDR = "192.168.1.100";
int ROBOT_PORT = 8899;
RSHD g_rshd = -1;

websocket_server info_server(8081);
websocket_server control_server(8082,false,contorl_feedback);

void menu_show();
int main(int argc, char* argv[])
{
	std::string c;

	int nRetCode = 0;
	double forceLimit = 0;
	double distLimit = 0;
	bool login_status = false;
	bool is_startup = false;

	while (true)
	{
		std::cout << "尝试登陆目标IP: " << ROBOT_ADDR << ':' << ROBOT_PORT << " 是否修改？ （y or else）" << std::endl;
		std::cout << "输入 ："; std::cin >> c;
		if ('y' == c.c_str()[0])
		{
			std::cout << "地址:";
			std::cin >> ROBOT_ADDR;
			std::cout << "端口:";
			std::cin >> ROBOT_PORT;
		}


		//登录服务器
		if (try_login(g_rshd, ROBOT_ADDR.c_str(), ROBOT_PORT))
		{
			std::cout << "登陆服务器成功" << std::endl;
			//启动机械臂(必须连接真实机械臂）
			
			try_robotStartup(g_rshd);
			
			// 获取当前连接状态
			if( RS_SUCC ==  rs_get_login_status(g_rshd, &login_status))
			{
				std::cout << "login_status : " << login_status << std::endl;
				std::cout << "online" << std::endl;
				menu_show();
				break;
			}
			else 
			{
				std::cout << "offline" << std::endl;
			}
		}
	}
	std::cout << "退出登录" << std::endl;
	//退出登录
	try_logout(g_rshd);
	//反初始化接口库
	rs_uninitialize();

	return 0;
}

void menu_show() {
	std::string c;
	bool online_state = false;
	double joint_yaw[6];
	rs_get_login_status(g_rshd, &online_state);
	while (true)
	{
		if (RS_SUCC != rs_get_login_status(g_rshd, &online_state))
		{
			std::cout << "获取状态失败" << std::endl;
			online_state = false;
		}
		std::cout << "当前连接状态 ：" << online_state << "  当前客户端连接数 ：" << info_server.handle_set.size() << std::endl;
		std::cout << "请输入指令" << std::endl;
		std::cout << "指令 ‘1’ 开始接收并转发数据" <<std::endl;
		std::cout << "指令 ‘2’ 停止接收并转发数据" << std::endl;
		std::cout << "指令 ‘3’ 注销连接" << std::endl;
		std::cout << "指令 ‘4’ 修改关节角" << std::endl;
		std::cout << "指令 ‘5’ 直线运动" << std::endl;

		std::cout << "输入 ："; std::cin >> c;
		switch (c.c_str()[0])
		{
			case '1':set_callback(g_rshd, &info_server,true); break;
			case '2':set_callback(g_rshd, &info_server,false); break;
			case '3':return; break;
			case '4':	std::cout << "输入六个关节角 ："; 

						for (int i = 0; i < 6; i++) {
							std::cin >> joint_yaw[i];
							joint_yaw[i] = joint_yaw[i] / 180.0 * M_PI;
						}

						std::cout << "尝试修改" << std::endl; 
						rs_move_joint(g_rshd, joint_yaw, 1);
						std::cout << "修改完毕" << std::endl; break;
			case '5':Pos tag;
					 std::cout << "输入 ：x="; std::cin >> tag.x;
					 std::cout << "输入 ：y="; std::cin >> tag.y;
					 std::cout << "输入 ：z="; std::cin >> tag.z;
					 try_position_control(g_rshd, tag);
					 std::cout << "尝试直线运动" << std::endl;

					 //for (int i = 0; i < 100;i++) {
						// tag.x += 0.0002;
						// tag.y += 0.0002;
						// tag.z += 0.0002;
						// try_position_control(g_rshd, tag);
						// std::cout << "尝试直线运动" << std::endl;
						// std::this_thread::sleep_for(std::chrono::milliseconds(200));
					 //}
					  break;
			default: std::cout << "未知指令" << std::endl; break;
		}
	}
}