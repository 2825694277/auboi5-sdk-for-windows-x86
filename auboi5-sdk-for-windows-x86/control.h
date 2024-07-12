#pragma once
#include "main_sever.h"
#include "information_sever.h"
#include "string"

void contorl_feedback(std::string msg, websocket_server* server);

std::string control_move(std::string msg);

std::string try_position_control(RSHD rshd, Pos pos);

std::string try_joint_control(RSHD rshd, double* joint_angle);
