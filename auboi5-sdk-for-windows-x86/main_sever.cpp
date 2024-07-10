#include "example.h"
#include "callback_data.h"
#include "information_sever.h"
std::string ROBOT_ADDR = "192.168.177.179";
int ROBOT_PORT = 8899;

//��е�ۿ��������ľ��
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
		std::cout << "���Ե�½Ŀ��IP: " << ROBOT_ADDR << ':' << ROBOT_PORT << " �Ƿ��޸ģ� ��y or else��" << std::endl;
		std::cout << "���� ��"; cin >> c;
		if ('y' == c.c_str()[0])
		{
			std::cout << "��ַ:";
			std::cin >> ROBOT_ADDR;
			std::cout << "�˿�:";
			std::cin >> ROBOT_PORT;
		}


		//��¼������
		if (try_login(g_rshd, ROBOT_ADDR.c_str(), ROBOT_PORT) || 1)
		{
			std::cout << "��½�������ɹ�" << std::endl;
			//������е��(����������ʵ��е�ۣ�
			
			try_robotStartup(g_rshd);
			
			// ��ȡ��ǰ����״̬
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
	std::cout << "�˳���¼" << std::endl;
	//�˳���¼
	try_logout(g_rshd);
	//����ʼ���ӿڿ�
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
			std::cout << "��ȡ״̬ʧ��" << std::endl;
			online_state = false;
		}
		std::cout << "��ǰ����״̬ ��" << online_state << "  ��ǰ�ͻ��������� ��" << recive_data_handle->Server->handle_set.size() << std::endl;
		std::cout << "������ָ��" << std::endl;
		std::cout << "ָ�� ��1�� ��ʼ���ղ�ת������" <<std::endl;
		std::cout << "ָ�� ��2�� ֹͣ���ղ�ת������" << std::endl;
		std::cout << "ָ�� ��3�� ע������" << std::endl;
		std::cout << "���� ��"; cin >> c;
		switch (c.c_str()[0])
		{
			case '1':recive_data_handle->start_recive_data(); break;
			case '2':recive_data_handle->stop_recive_data(); break;
			case '3':return; break;
			default: std::cout << "δָ֪��" << std::endl; break;
		}
	}
}