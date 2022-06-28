//
// Created by cyx02 on 2022/6/27.
//

#include <cstring>
#include <cstdlib>
#include "BubbleProtocol.h"
#include "common.h"
#include "vehicle.h"
#include "parameters.h"
#include <vector>
#include <unordered_map>
#include <algorithm>
using namespace std;

#define TX_COLI 0   //同时发射
#define NO_COLI 1   //没有碰撞,正常解包
#define RX_COLI 2   //接收端碰撞

void bubble_handle_packets(struct Duallist *ALL_Vehicles, int slot){
    struct Item * aItem, *bItem, *cItem;
    struct vehicle *aCar;

    aItem = ALL_Vehicles->head;
    while(aItem != NULL){
        aCar = (struct vehicle*)aItem->datap;

        //初始化记录的变量，TBD
        (aCar->front_Vehicles).clear();
        (aCar->rearV_Vehicles).clear();
        (aCar->queue_Vehicles).clear();
        for(int i = 0; i < SlotPerFrame; i++){
            aCar->OHN[i] = nullptr;
            aCar->THN[i] = nullptr;
        }

        bItem = (struct Item*)aCar->packets.head;
        while(bItem != NULL){
            struct packet* pkt = (struct packet*)bItem->datap;
            if(pkt->condition == TX_COLI){//这个包属于两个同时发射，无法被感知到
                bItem = bItem->next;
            }else if(pkt->condition == NO_COLI){//这种属于能够正常解的包
                //更新OHN
                int index = (pkt->timestamp)%SlotPerFrame;
                aCar->OHN[index] = pkt->srcVehicle;

                //更新THN
                for(int i = 0; i < SlotPerFrame; i++){
                    if(pkt->OHN_snapshot[i]!=NULL)
                        aCar->THN[i] = pkt->OHN_snapshot[i];
                }
                //更新front_Vehicles, rear_Vehicles
                if(strcmp(aCar->lane, pkt->srcVehicle->lane) == 0){ //处于相同车道
                    if(pkt->srcVehicle->pos > aCar->pos){//同车道前方
                        (aCar->front_Vehicles).push_back(pkt->srcVehicle);
                    }else{//同车道后方
                        (aCar->rearV_Vehicles).push_back(pkt->srcVehicle);
                    }

                    // 更新同车道前后最近邻车辆的信息
                    aCar->forntV_his = aCar->frontV;
                    aCar->rearV_his = aCar->rearV_his;
                    aCar->rearV = nearestVehicle(aCar, aCar->rearV_Vehicles);
                    aCar->frontV = nearestVehicle(aCar, aCar->rearV_Vehicles);

                    // 更新车队信息,哪些车用了什么时槽
                    for(auto tmp: pkt->hashtable){
                        struct vehicle* bCar = tmp.first;
                        int slot_index = tmp.second;
                        aCar->queue_Vehicles[bCar] = slot_index;
                    }
                }
            }else if(pkt->condition == RX_COLI){//这种属于1个接收端同时有多个包送达，不能解出包，但是能感知到包的存在
                int index = (pkt->timestamp)%SlotPerFrame;
                aCar->OHN[index] = pkt->srcVehicle;
                bItem = bItem->next;
            }
        }
    }
}

// 找到一个链表（同车道）中，与aCar距离最近的车辆
struct vehicle* nearestVehicle(struct vehicle* aCar, vector<struct vehicle*> vehicleList){
    if(vehicleList.size() == 0){
        return nullptr;
    }
    struct vehicle* ans = vehicleList[0];
    for(int i = 0; i< vehicleList.size();i++){
        if(abs(vehicleList[i]->pos - aCar->pos) < abs(ans->pos - aCar->pos)){
            ans = vehicleList[i];
        }
    }
    return ans;
}

double safe_range(struct vehicle* aCar, struct vehicle* frontV){
    double ans;

    if(frontV == nullptr){
        (aCar->speed)*(aCar->speed)/2/(aCar->speed);
    }else{

    }
}
//determine the communication range
void bubble_protocol_commRange(struct Duallist *ALL_Vehicles, int slot){
    struct Item * aItem;
    struct vehicle *aCar;

    aItem = ALL_Vehicles->head;
    while(aItem != NULL) {
        aCar = (struct vehicle*)aItem->datap;
        if(aCar->frontV == nullptr && aCar->rearV == nullptr){
            aCar->commRadius = safe_range(aCar, aCar->frontV);
        }else if(aCar->frontV == nullptr && aCar->rearV != nullptr){
            aCar->commRadius = max(safe_range(aCar, aCar->frontV), distance_between_vehicle(aCar, aCar->rearV));
        }else if(aCar->frontV != nullptr && aCar->rearV == nullptr){
            aCar->commRadius = max(safe_range(aCar, aCar->frontV), distance_between_vehicle(aCar, aCar->frontV));
        }else{
            aCar->commRadius = max(safe_range(aCar, aCar->frontV),max(distance_between_vehicle(aCar, aCar->frontV), distance_between_vehicle(aCar, aCar->rearV)));
        }
    }
}

//determine the slot
void bubble_protocol_slot(struct Duallist *ALL_Vehicles, int slot){
    struct Item * aItem, *bItem, *cItem;
    struct vehicle *aCar;

    aItem = ALL_Vehicles->head;
    while(aItem != NULL) {
        aCar = (struct vehicle*)aItem->datap;

        //单车的case,单车的时候，随机选择
        if(aCar->role_condition == ROLE_S) {


        }else if(aCar->role_condition == )

    }
}
