//
// Created by cyx02 on 2022/6/27.
//

#ifndef BUBBLEMAC_INFOCOM_PARAMETERS_H
#define BUBBLEMAC_INFOCOM_PARAMETERS_H

#define ROLE_S 0
#define ROLE_H 1
#define ROLE_T 2
#define ROLE_I 3
#define ROLE_A 4   //在申请

#define ROLE_VeMAC 4


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
extern int cnt_pkt_tx;
extern int cnt_tx_collision;
extern int cnt_pkt;
extern int cnt_pkt_1;
extern int cnt_pkt_2;
extern int cnt_received;
extern int traffic_density;
extern int cnt_coli;
extern int counterToChange;
extern int len_head_resource;
extern int len_tail_resouce;

extern bool log_flag;
extern  ofstream logfile;

extern struct vehicle* collision_vehicle;
#endif //BUBBLEMAC_INFOCOM_PARAMETERS_H
