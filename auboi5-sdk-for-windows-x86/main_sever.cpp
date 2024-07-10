#include "example.h"
#include "callback_data.h"
#include "information_sever.h"
std::string ROBOT_ADDR = "192.168.177.179";
int ROBOT_PORT = 8899;

//机械臂控制上下文句柄
RSHD g_rshd = -1;
void menu_show(recive_data* recive_data_handle);
int main(int argc, char* argv[])
{
	std::string c;
	websocket_server info_server(9002);
	int nRetCode = 0;
	double forceLimit = 0;
	double distLimit = 0;
	bool login_status = false;
	bool is_startup = false;

	while (true)
	{
		std::cout << "尝试登陆目标IP: " << ROBOT_ADDR << ':' << ROBOT_PORT << " 是否修改？ （y or else）" << std::endl;
		std::cout << "输入 ："; cin >> c;
		if ('y' == c.c_str()[0])
		{
			std::cout << "地址:";
			std::cin >> ROBOT_ADDR;
			std::cout << "端口:";
			std::cin >> ROBOT_PORT;
		}


		//登录服务器
		if (try_login(g_rshd, ROBOT_ADDR.c_str(), ROBOT_PORT) || 1)
		{
			std::cout << "登陆服务器成功" << std::endl;
			//启动机械臂(必须连接真实机械臂）
			
			try_robotStartup(g_rshd);
			
			// 获取当前连接状态
			if( RS_SUCC ==  rs_get_login_status(g_rshd, &login_status))
			{
				std::cout << "login_status : " << login_status << std::endl;
				std::cout << "online" << std::endl;
				recive_data recive_data_handle(g_rshd, &info_server);
				menu_show(&recive_data_handle);
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

void menu_show(recive_data* recive_data_handle) {
	std::string c;
	bool online_state = false;
	rs_get_login_status(recive_data_handle->Rshd, &online_state);
	while (true)
	{
		if (! RS_SUCC == rs_get_login_status(recive_data_handle->Rshd, &online_state))
		{
			std::cout << "获取状态失败" << std::endl;
			online_state = false;
		}
		std::cout << "当前连接状态 ：" << online_state << "  当前客户端连接数 ：" << recive_data_handle->Server->handle_set.size() << std::endl;
		std::cout << "请输入指令" << std::endl;
		std::cout << "指令 ‘1’ 开始接收并转发数据" <<std::endl;
		std::cout << "指令 ‘2’ 停止接收并转发数据" << std::endl;
		std::cout << "指令 ‘3’ 注销连接" << std::endl;
		std::cout << "输入 ："; cin >> c;
		switch (c.c_str()[0])
		{
			case '1':recive_data_handle->start_recive_data(); break;
			case '2':recive_data_handle->stop_recive_data(); break;
			case '3':return; break;
			default: std::cout << "未知指令" << std::endl; break;
		}
	}
}