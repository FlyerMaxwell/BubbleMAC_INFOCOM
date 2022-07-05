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
#include <algorithm>
#include <iostream>

using namespace std;

#define TX_COLI 0   //同时发射
#define NO_COLI 1   //没有碰撞,正常解包
#define RX_COLI 2   //接收端碰撞


// 添加一下处理听到的一整个车队的时槽使用情况
// 所有车辆处理其收到的一帧之内的packets
void bubble_handle_packets(struct Duallist *ALL_Vehicles, int slot){
    struct Item * aItem, *bItem, *cItem;
    struct vehicle *aCar;

    aItem = ALL_Vehicles->head;
    while(aItem != NULL){
        aCar = (struct vehicle*)aItem->datap;

        //初始化记录的变量，也就是更新这些东西啦
        (aCar->front_Vehicles).clear();
        (aCar->rearV_Vehicles).clear();
        (aCar->queue_Vehicles).clear();
        (aCar->queue_Vehicles_slot).clear();
        aCar->frontV = nullptr;
        aCar->rearV = nullptr;

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
//                    for(auto tmp: pkt->hashtable){
//                        struct vehicle* bCar = tmp.first;
//                        int slot_index = tmp.second;
//                        aCar->queue_Vehicles[bCar] = slot_index;
//                    }
                    for(int ii = 0; ii < pkt->hashtable_slot.size(); ii++){
                        struct vehicle* bCar = pkt->hashtable_vehicles[ii];
                        int slot_index = pkt->hashtable_slot[ii];
                        aCar->queue_Vehicles.push_back(bCar);
                        aCar->queue_Vehicles_slot.push_back(slot_index);
                    }


                }else{//对于非同车道的车，若听到了一个tail，则将其车队信息更新到THN不可用中
                    if(pkt->srcVehicle->role_condition == ROLE_T){
                        for(auto tmp: pkt->hashtable){
                            struct vehicle* bCar = tmp.first;
                            int slot_index = tmp.second;
                            aCar->THN[slot_index] = bCar;
                        }
                    }
                }
                bItem = bItem->next;
            }else if(pkt->condition == RX_COLI){//这种属于1个接收端同时有多个包送达，不能解出包，但是能感知到包的存在
                int index = (pkt->timestamp)%SlotPerFrame;
                aCar->OHN[index] = pkt->srcVehicle;
                bItem = bItem->next;
            }
        }
        aItem = aItem->next;
    }
    //cout<<"hello!!!"<<endl;
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
        return (aCar->speed)*(aCar->speed)/2/(aCar->a)+3;
    }else{
        return ((aCar->speed)*(aCar->speed) - (frontV->speed)*(frontV->speed))/2/aCar->a +3;
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
        aItem = aItem->next;
    }
}


//determine the slot
void bubble_protocol_slot(struct Duallist *ALL_Vehicles, int slot){
    struct Item * aItem;
    struct vehicle *aCar;

    aItem = ALL_Vehicles->head;
    while(aItem != NULL) {
        aCar = (struct vehicle*)aItem->datap;

        //单车的case,单车的时候，随机选择
        if(aCar->role_condition == ROLE_S) {
            slot_determine_for_access(aCar, ROLE_S);
        }else if(aCar->role_condition == ROLE_H){
            if(aCar->slot_condition == ACCESS){
                if(aCar->frontV == nullptr && aCar->rearV != nullptr && aCar->rearV->OHN[aCar->slot_occupied] == aCar){
                    aCar->slot_condition = OCCUPIED;//被后车认可
                    aCar->counter = 0;
                }else{
                    slot_determine_for_access(aCar, ROLE_H);
                }
            }else if(aCar->slot_condition == OCCUPIED){
                if(aCar->frontV == nullptr && aCar->rearV != nullptr && aCar->rearV->OHN[aCar->slot_occupied] == aCar){
                    aCar->slot_condition = OCCUPIED;//被后车认可
                    aCar->counter = 0;
                }else{
                    aCar->counter ++;
                }
                if(aCar->counter == counterToChange){
                    slot_determine_for_access(aCar, ROLE_H);
                }
            }else{
                cout<<"There is an error for slot deter"<<endl;
            }
        }else if(aCar->role_condition == ROLE_T){
            if(aCar->role_condition == ACCESS){
                if(aCar->rearV == nullptr && aCar->frontV != nullptr && aCar->frontV->OHN[aCar->slot_occupied] == aCar){
                    aCar->slot_condition = OCCUPIED;//被后车认可
                    aCar->counter = 0;
                }else{
                    slot_determine_for_access(aCar, ROLE_H);
                }
            }else if(aCar->slot_condition == OCCUPIED){
                if(aCar->rearV == nullptr && aCar->frontV != nullptr && aCar->frontV->OHN[aCar->slot_occupied] == aCar){
                    aCar->slot_condition = OCCUPIED;//被后车认可
                    aCar->counter = 0;
                }else{
                    aCar->counter++;
                }

                if(aCar->counter == counterToChange){
                    slot_determine_for_access(aCar, ROLE_H);
                }
            }
        }else if(aCar->role_condition == ROLE_I){
            if(aCar->role_condition == ACCESS){
                if(aCar->rearV != nullptr && aCar->frontV != nullptr && aCar->frontV->OHN[aCar->slot_occupied] == aCar && aCar->rearV->OHN[aCar->slot_occupied] == aCar){
                    aCar->slot_condition = OCCUPIED;//被后车认可
                    aCar->counter = 0;
                }else{
                    slot_determine_for_access(aCar, ROLE_I);
                }
            }else if(aCar->role_condition == OCCUPIED){
                if(aCar->rearV != nullptr && aCar->frontV != nullptr && aCar->frontV->OHN[aCar->slot_occupied] == aCar && aCar->rearV->OHN[aCar->slot_occupied] == aCar){
                    aCar->slot_condition = OCCUPIED;//被前后车认可
                    aCar->counter = 0;
                }else{
                    aCar->counter++;
                }
                if(aCar->counter == counterToChange){
                    slot_determine_for_access(aCar, ROLE_H);
                }
            }
        }
        aItem = aItem->next;
    }
}


void slot_determine_for_access(struct vehicle* aCar, int role){

    if(aCar->frontV == nullptr && aCar->rearV == nullptr){
        aCar->role_condition = ROLE_S;
        aCar->slot_occupied = choose_slot(aCar, ROLE_S);
        aCar->slot_condition = ACCESS;
    }else if(aCar->frontV == nullptr && aCar->rearV != nullptr){
        aCar->role_condition = ROLE_H;
        aCar->slot_occupied = choose_slot(aCar, ROLE_H);
        aCar->slot_condition = ACCESS;
    }else if(aCar->frontV != nullptr && aCar->rearV == nullptr){
        aCar->role_condition = ROLE_T;
        aCar->slot_occupied = choose_slot(aCar, ROLE_T);
        aCar->slot_condition = ACCESS;
    }else if(aCar->frontV != nullptr && aCar->rearV != nullptr){
        aCar->role_condition = ROLE_I;
        aCar->slot_occupied = choose_slot(aCar, ROLE_I);
        aCar->slot_condition = ACCESS;
    }else{
        cout<<"ERROR to determine a slot for "<< role <<" vehicle"<<endl;
    }
}

//根据角色，在对应的时槽资源块内随机进行时槽选择；拒绝采样
int choose_slot(struct vehicle* aCar, int role){
    int a, b;
    int ans = 0;

        switch(role) {
            case ROLE_H:
                a = 0;
                b = len_head_resource -1;
                while(1){
                    ans = rand()%(b - a + 1) + a;
                    if(aCar->THN[ans]== NULL) break;
                }
                break;
            case ROLE_S:
                a = len_head_resource;
                b = (SlotPerFrame - 1)/2 - len_tail_resouce ;
                while(1){
                    ans = rand()%(b - a + 1) + a;
                    if(aCar->THN[ans]== NULL) break;
                }
                break;
            case ROLE_I:
                a = len_head_resource;
                b = (SlotPerFrame - 1)/2 - len_tail_resouce ;
                while(1){
                    ans = rand()%(b - a + 1) + a;
                    if(aCar->THN[ans]== NULL) break;
                }
                break;
            case ROLE_T:
                a = (SlotPerFrame - 1)/2 - len_tail_resouce;
                b = (SlotPerFrame - 1)/2 ;
                while(1){
                    ans = rand()%(b - a + 1) + a;
                    if(aCar->THN[ans]== NULL) break;
                }
                break;
            case ROLE_VeMAC:
                a = 0;
                b = (SlotPerFrame - 1)/2;
                while(1){
                    ans = rand()%(b - a + 1) + a;
                    if(aCar->THN[ans]== NULL) break;
                }
                break;
        }
        if(aCar->resource_pool == 0)
            return ans;
        else
            return ans + (SlotPerFrame - 1) / 2;
}