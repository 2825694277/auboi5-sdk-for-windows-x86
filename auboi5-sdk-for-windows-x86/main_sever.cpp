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
		std::cout << "���Ե�½Ŀ��IP: " << ROBOT_ADDR << ':' << ROBOT_PORT << " �Ƿ��޸ģ� ��y or else��" << std::endl;
		std::cout << "���� ��"; std::cin >> c;
		if ('y' == c.c_str()[0])
		{
			std::cout << "��ַ:";
			std::cin >> ROBOT_ADDR;
			std::cout << "�˿�:";
			std::cin >> ROBOT_PORT;
		}


		//��¼������
		if (try_login(g_rshd, ROBOT_ADDR.c_str(), ROBOT_PORT))
		{
			std::cout << "��½�������ɹ�" << std::endl;
			//������е��(����������ʵ��е�ۣ�
			
			try_robotStartup(g_rshd);
			
			// ��ȡ��ǰ����״̬
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
	std::cout << "�˳���¼" << std::endl;
	//�˳���¼
	try_logout(g_rshd);
	//����ʼ���ӿڿ�
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
			std::cout << "��ȡ״̬ʧ��" << std::endl;
			online_state = false;
		}
		std::cout << "��ǰ����״̬ ��" << online_state << "  ��ǰ�ͻ��������� ��" << info_server.handle_set.size() << std::endl;
		std::cout << "������ָ��" << std::endl;
		std::cout << "ָ�� ��1�� ��ʼ���ղ�ת������" <<std::endl;
		std::cout << "ָ�� ��2�� ֹͣ���ղ�ת������" << std::endl;
		std::cout << "ָ�� ��3�� ע������" << std::endl;
		std::cout << "ָ�� ��4�� �޸Ĺؽڽ�" << std::endl;
		std::cout << "ָ�� ��5�� ֱ���˶�" << std::endl;

		std::cout << "���� ��"; std::cin >> c;
		switch (c.c_str()[0])
		{
			case '1':set_callback(g_rshd, &info_server,true); break;
			case '2':set_callback(g_rshd, &info_server,false); break;
			case '3':return; break;
			case '4':	std::cout << "���������ؽڽ� ��"; 

						for (int i = 0; i < 6; i++) {
							std::cin >> joint_yaw[i];
							joint_yaw[i] = joint_yaw[i] / 180.0 * M_PI;
						}

						std::cout << "�����޸�" << std::endl; 
						rs_move_joint(g_rshd, joint_yaw, 1);
						std::cout << "�޸����" << std::endl; break;
			case '5':Pos tag;
					 std::cout << "���� ��x="; std::cin >> tag.x;
					 std::cout << "���� ��y="; std::cin >> tag.y;
					 std::cout << "���� ��z="; std::cin >> tag.z;
					 try_position_control(g_rshd, tag);
					 std::cout << "����ֱ���˶�" << std::endl;

					 //for (int i = 0; i < 100;i++) {
						// tag.x += 0.0002;
						// tag.y += 0.0002;
						// tag.z += 0.0002;
						// try_position_control(g_rshd, tag);
						// std::cout << "����ֱ���˶�" << std::endl;
						// std::this_thread::sleep_for(std::chrono::milliseconds(200));
					 //}
					  break;
			default: std::cout << "δָ֪��" << std::endl; break;
		}
	}
}