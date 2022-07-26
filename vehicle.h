//
// Created by cyx02 on 2022/6/27.
//

#ifndef BUBBLEMAC_INFOCOM_VEHICLE_H
#define BUBBLEMAC_INFOCOM_VEHICLE_H

#include "packets.h"
#include "common.h"
#include <vector>
#include <unordered_map>
#include "parameters.h"
#include <fstream>
#include <queue>

using namespace std;

// To describe the slot occupied condition
typedef struct slot_info
{
    struct vehicle* aCar;
    int slot;

}Slot_info;

typedef struct vehicle
{
    int handled;                  //  to indicate whether the car has been updated during this time 2表示新车
    //basic info
    char id[20];// id of the car
    char type[10]; //model of the car

    //dynamic info
    double x;
    double y;
    double angle;
    double speed;
    double pos;
    char lane[10];
    //char prev_lane[10];//记录上一个车道
    double slope;
    double flow;
    double speed2;

    double acc = 4.5;  // for this version. this is determined by type.

    //    int turn;  //0 for not turn the lane, 1 for turn the lane.

    //slot info for VeMAC
    //int ve_slot_condition;  // 0 for not occupied; 1 for requesting a slot; 2 for occupied slots
//    int ve_resource_pool;   // 0 for left, 1 for right.
//    int ve_count_srp = 3;//多少个没有被认可可以被接受
//    int ve_check_flag = 1;

    //slot info
//    int slot_condition = OCCUPIED ;
//    int role_condition = ROLE_S;    //0 for single, 1 for tail, 2 for mid, 3 for head.
//    int slot_occupied; //the occupied slot
//    int resource_pool;  //0 for left and 1 for R


//    struct vehicle* THN[200];
//    struct vehicle* prev_OHN[200];

//    int single_timestamp;
//    int acceess_timestamp;
//    int occupied_timestamp;
//    int bubble_flag;


    //Packets Received
    //vector<struct packet*> *packets;

//    //front vehicles and rear vehicles From the last frame. For bubble MAC
//    vector<struct vehicle*> *front_Vehicles;
//    vector<struct vehicle*> *rearV_Vehicles;
//
//    // 储存本车队的时槽信息
//    vector<struct vehicle*> *queue_Vehicles; //用这俩组合作为hashtable吧
////    vector<int> *queue_Vehicles_slot;
//
//    struct vehicle* frontV;
//    struct vehicle* rearV;
//    struct vehicle* forntV_his; //用于比较之前的前后车是否都认可了自己
//    struct vehicle* rearV_his;
//
//    int counter_tx;
//    int counter_rx_TxCollision;
//    int counter_rx_RxCollision;
//    int counter_rx_normal;



    //Bubble MAC Related para
    int appear_timestamp;
    int state;
    int QueueRole;
    int state_timestamp;
    int transmission_slot;

    int resource_pool;  //0 for left and 1 for R

    struct vehicle* SameLaneFrontV;
    struct vehicle* SameLaneRearV;
    struct vehicle* SameLaneFrontV_his;
    struct vehicle* SameLaneRearV_his;

    queue<struct packet*> *packets;
    vector<struct slot_info*> *OHN;
    vector<struct slot_info*> *prev_OHN;


    vector<struct slot_info*> *frontQueue;
    vector<struct slot_info*> *rearQueue;


    struct Duallist neighbours;

    //Commrange
    double commRadius;


}Vehicle;

double distance_between_vehicle(const struct vehicle* aCar, const struct vehicle* bCar);
void printVehilces(struct Duallist *ALL_Vehicles);
void logVehilcesInfo(struct Duallist *ALL_Vehicles, ofstream & logfile);
void logACar(struct vehicle* aCar);

#endif //BUBBLEMAC_INFOCOM_VEHICLE_H
