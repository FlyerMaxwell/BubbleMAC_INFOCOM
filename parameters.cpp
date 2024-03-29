//
// Created by cyx02 on 2022/6/27.
//

#include "parameters.h"

int UpLocSlot = 100;//每个slot单位是0.5ms，更新位置的时间间隔为50ms，故为100个slot.      slot/UpLocSlot即为第几个5ms，或第几个位置
int SlotPerFrame = 200; //每一帧0.1秒，100ms，是200个slot，那么每个slot的时间为0.5毫秒
int Car_Number = 0;
int Max_speed;

int traffic_density;

int counterToChange = 3;
int len_head_resource = 20;
int len_tail_resouce = 20;


// Log parameters
bool log_process_flag = true;    // to log the process
ofstream log_process_file;
bool log_statistic_flag = true;    // to log the statistical target
ofstream log_statistic_file;


// Statistic Parameters
int cnt_cars = 0;
int cnt_tx_collision = 0;     //两个发射碰撞
int cnt_rx_normal = 0;       //正常收包
int cnt_rx_colli = 0;        //产生碰撞的包
int cnt_pkt_tx = 0;          //发射的包个数
int cnt_pkt_tx_normal = 0;   //发射过程中没有TX碰撞的个数
int cnt_frontV_normal;       //收到的前方的正常包
int cnt_rearV_normal;       //收到的后车的正常包
int cnt_frontV_colli;       //收到的前车的碰撞包
int cnt_rearV_Colli;        //收到的后车的碰撞包


struct vehicle* collision_vehicle= nullptr;
