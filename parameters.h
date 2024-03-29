//
// Created by cyx02 on 2022/6/27.
//

#ifndef BUBBLEMAC_INFOCOM_PARAMETERS_H
#define BUBBLEMAC_INFOCOM_PARAMETERS_H

//Bubble MAC related para

// For state
#define state_S 0
#define state_A 1
#define state_O 2

//For QueueRole
#define role_H 0
#define role_I 1
#define role_T 2






//包的状态
#define TX_COLI 0   //同时发射
#define NO_COLI 1   //没有碰撞,正常解包
#define RX_COLI 2   //接收端碰撞


#include <fstream>
using namespace std;

#define OCCUPIED 2
#define ACCESS 1
#define SINGLE 0

extern int UpLocSlot;
extern int SlotPerFrame;
extern int Car_Number;
extern int Max_speed;


extern int traffic_density;
extern int counterToChange;
extern int len_head_resource;
extern int len_tail_resouce;

extern bool log_process_flag;
extern  ofstream log_process_file;

extern bool log_statistic_flag;
extern ofstream log_statistic_file;


extern struct vehicle* collision_vehicle;

// Statistic Parameters
extern int cnt_cars;
extern int cnt_tx_collision;
extern int cnt_rx_normal;
extern int cnt_rx_colli;
extern int cnt_pkt_tx;
extern int cnt_pkt_tx_normal;
extern int cnt_frontV_normal;
extern int cnt_rearV_normal;
extern int cnt_frontV_colli;
extern int cnt_rearV_Colli;


#endif //BUBBLEMAC_INFOCOM_PARAMETERS_H
