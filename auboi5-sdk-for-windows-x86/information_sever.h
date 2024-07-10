#pragma once
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <thread>
#include <mutex>
#include <vector>
#include <iostream>
#include <functional>


class websocket_server {
public:
    websocket_server(int port = 9002, bool Debug = false) {
        this->server.init_asio();
        this->server.set_reuse_addr(true); // 设置 SO_REUSEADDR 选项 -解决服务器关闭后没有完全释放端口导致报错
        this->server.set_open_handler(boost::bind(&websocket_server::on_open, this, boost::placeholders::_1));
        this->server.set_message_handler(boost::bind(&websocket_server::on_message, this, boost::placeholders::_1, boost::placeholders::_2));
        this->server.set_close_handler(boost::bind(&websocket_server::on_close, this, boost::placeholders::_1));
        this->server.listen(port);
        this->server.start_accept();

        if (!Debug) {
            this->server.clear_access_channels(websocketpp::log::alevel::all);
            this->server.clear_error_channels(websocketpp::log::elevel::all);
        }

        this->server_thread = std::thread([this]() {
            this->server.run(); });
    }

    ~websocket_server() {
        this->server.stop_listening();
        this->server.get_io_service().stop();
        // 在析构函数中等待服务器线程结束
        if (this->server_thread.joinable()) {
            this->server_thread.join();
        }
    }

    void on_open(websocketpp::connection_hdl hdl) {
        std::cout << std::endl << "-------------------------------" << std::endl;
        std::cout << "建立新的连接成功" << std::endl;
        std::cout << "-------------------------------" << std::endl;
        std::lock_guard<std::mutex> lock(connection_mutex);
        handle_set.push_back(hdl);
    }

    void on_message(websocketpp::connection_hdl hdl, websocketpp::server<websocketpp::config::asio>::message_ptr msg) {
        std::cout << std::endl << "-------------------------------" << std::endl;
        std::cout << "接收到消息: " << msg->get_payload() << std::endl;
        std::cout << "-------------------------------" << std::endl;
    }

    void on_close(websocketpp::connection_hdl hdl) {
        std::cout << std::endl << "-------------------------------" << std::endl;
        std::cout << "客户端连接断开，从句柄列表中移除" << std::endl;
        std::cout << "-------------------------------" << std::endl;
        std::lock_guard<std::mutex> lock(connection_mutex);
        for (size_t i = 0; i < handle_set.size(); i++)
        {
            if (!hdl.owner_before(handle_set[i]) && !handle_set[i].owner_before(hdl))
            {
                handle_set.erase(handle_set.begin() + i);
                break;
            }
        }
    }

    void send_message(websocketpp::connection_hdl hdl, const std::string content) {
        // 发送消息时使用互斥锁保护
        std::lock_guard<std::mutex> lock(send_mutex);
        server.send(hdl, content, websocketpp::frame::opcode::text);
    }
    std::vector<websocketpp::connection_hdl> handle_set;
    websocketpp::server<websocketpp::config::asio> server;
private:
    std::mutex send_mutex;
    std::mutex connection_mutex; // 保护 handle_set 的互斥锁
    std::thread server_thread;
};


