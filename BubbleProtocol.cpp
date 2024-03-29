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
#include <queue>

using namespace std;


/*
 *应该按照ve_MAC的感觉来写bubble MAC
 *
 * 时槽只有三种状态，申请、占有（head、 tail、 mid）、随机（单车）
 *
 * 整个的流程如下。对于每个时槽来说，需要根据身份进行具体的决策：
 *
 * 对于随机态（单车）来说：（单车永远没有固定的时槽，其时槽的选择是随机的）
 *      半径选择
 *          -确定广播半径（根据前后最近的车以及安全防护原则）
 *      时槽选择
 *          -先看距离出现是否过了一帧，如果没过，则处于单纯收听阶段，不进行时槽选择（出现的第一帧进行收听）
 *          -若刚好到了出现后的第一帧结束，则作为单车申请一个时槽（此时要根据前后是否有车，决定身份），记录下时槽申请的时间（在这个时间后的一帧内只收听，不决策）
 *          -此时可能继续为单车（前后均无车，避让所有THN的时槽）、变成申请态（三种申请的可能，head（前无车而后有车）、变成tail（后无车而前有车）、变成中间（前后均有车））
 *
 * 对于处于申请态的车来说
 *          -如果处于时槽申请状态，判断是否已经监听满了一帧，如果没满，则处于单纯收听阶段，不进行时槽选择；
            -如果刚好满了一帧，则判断是否得到了相应邻居的认可，如果得到了相应邻居的认可，则切换状态为占用，并保持使用该时槽；
            -否则重新申请一个时槽；

    对于处于占有状态的车来说
            -判断是否继续得到对应的认可，如果没有得到认可，则发生了Merging Collision，重新进行对应位置的申请
            -否则继续占用
 */



void BubbleMAC(struct Duallist *ALL_Vehicles, int slot){
    struct Item * aItem;
    struct vehicle* aCar;

    aItem = ALL_Vehicles->head;
    while(aItem != NULL) {
        aCar = (struct vehicle *) aItem->datap;

        if(aCar->state == state_S){

            //最特殊的case，刚刚出现不满一帧的车
            if(slot < aCar->appear_timestamp + SlotPerFrame){ //对于刚刚出现的车，需要听满一个frame才进行决策

                aItem = aItem->next;
                continue;
            }
            // 更为single 不满一帧的情况，只需要继续调整半径，时槽保持不变
            if((slot - aCar->state_timestamp)%SlotPerFrame != 0){
                deleteOutdatedPackets(aCar->packets, slot);
                UpdateOneFrameInfo(aCar);

                choose_commRange(aCar,slot);

                aItem = aItem->next;
                continue;
            }else{//对于已称为single满一帧的车辆，需要根据上一帧的信息进行决策
                deleteOutdatedPackets(aCar->packets, slot);
                UpdateOneFrameInfo(aCar);

                choose_commRange(aCar,slot);

                if(aCar->SameLaneFrontV == nullptr && aCar->SameLaneRearV == nullptr){

                }else if(aCar->SameLaneRearV == nullptr && aCar->SameLaneRearV != nullptr){

                }else if(aCar->SameLaneFrontV != nullptr && aCar->SameLaneRearV == nullptr){

                }else{

                }


                aItem = aItem->next;
                continue;

            }




        }else if(aCar->state == state_A){



        }else if(aCar->state == state_O){



        }




    }
}
// 更新车辆的packets队列，将其中一帧以外的packets清除掉
void deleteOutdatedPackets(queue<struct packet*> *packets, int slot){

    if(packets->size() == 0)
        return;

    while(packets->size()!=0 && packets->front()->timestamp < slot - SlotPerFrame ){
        free(packets->front());
        packets->pop();
    }

    return;
}

// 根据一帧以内听到的packets，更新前后车、OHN、THN、车队
void UpdateOneFrameInfo(struct vehicle* aCar){
    aCar->SameLaneRearV_his = aCar->SameLaneRearV;
    aCar->SameLaneFrontV_his = aCar->SameLaneFrontV;

    aCar->prev_OHN->clear();

    for(auto tmp: *(aCar->OHN))
        aCar->prev_OHN->push_back(tmp);

    aCar->OHN->clear();
    aCar->THN->clear();





    int len = aCar->packets->size();
    while(len!=0){
        struct packet* pkt = aCar->packets->front();
        aCar->packets->pop();


        //更新OHN
        struct slot_info *info = (struct slot_info*)malloc(sizeof(struct slot_info ));
        info->aCar = pkt->srcVehicle;
        info->slot = pkt->timestamp;
        aCar->OHN->push_back(info);

        //更新THN



        aCar->packets->push(pkt);
        len--;
    }
}









void bubble(struct Duallist *ALL_Vehicles, int slot){
    struct Item * aItem;
    struct vehicle* aCar;

    if(log_process_flag == true){
        log_process_file <<"---------------------------------------------------"<<endl;
        log_process_file << "slot = "<<slot<<endl;
    }

    aItem = ALL_Vehicles->head;
    while(aItem != NULL) {
        aCar = (struct vehicle*)aItem->datap;               //[a,b] (rand()%(b-a+1))+a


//        if(strcmp(aCar->id, "a02.0") == 0){
//            if(aCar->packets->size()!=0){
//                cout<<"-----------------a02.0's packet number:"<<aCar->packets->size()<<endl;
//                for(auto aPkt: *(aCar->packets))
//                    cout<<"slot="<<slot<<"timestamp: "<<aPkt->timestamp<<"  src_vehicle:"<<aPkt->srcVehicle->id<<"  condition:"<<aPkt->condition<<endl;
//                }
//        }


        handle_packets(aCar,slot);   //处理一帧以内收到的包的情况，更新进行决策的参考量;也要更新bubble_flag，看其是否被认可,等于true表明被认可，等于false表明不被认可


        if(log_process_flag == true){
            logACar(aCar);
        }


        if(aCar->slot_condition == SINGLE){
            if(slot < aCar->single_timestamp + SlotPerFrame){ //对于刚刚出现的车，需要听满一个frame才进行决策
                choose_commRange(aCar,slot);
                aItem = aItem->next;
                continue;
            }else if(slot >= aCar->single_timestamp + SlotPerFrame){
                if(aCar->frontV == nullptr && aCar->rearV == nullptr){

                    aCar->slot_occupied = choose_slot(aCar,ROLE_S, slot);//随机选择时槽 single
                    aCar->slot_condition = SINGLE; //保持single不变
                    aCar->single_timestamp = slot;//更新single_timestamp 否则后面每个时槽都会重新选择（我们只要一帧决策一次就好）
                    aCar->role_condition = ROLE_S;
                    choose_commRange(aCar,slot); //决策通信半径

                    //Reset the Indicator
                    aCar->forntV_his = aCar->frontV;
                    aCar->rearV_his = aCar->rearV;
                    aCar->frontV = nullptr;
                    aCar->rearV = nullptr;
                    for(int ii = 0; ii < SlotPerFrame; ii++){
                        aCar->prev_OHN[ii] = aCar->OHN[ii];
                        aCar->OHN[ii] = nullptr;
                        aCar->THN[ii] = nullptr;
                    }
                    //前车，后车，以及他们所对应的时槽使用情况
                    (*(aCar->front_Vehicles)).clear();
                    (*(aCar->rearV_Vehicles)).clear();
                    (*(aCar->queue_Vehicles)).clear();

                    aItem = aItem->next;
                    continue;

                }else if(aCar->frontV == nullptr && aCar->rearV != nullptr){

                    aCar->slot_occupied = choose_slot(aCar,ROLE_H, slot);//随机选择时槽
                    aCar->slot_condition = ACCESS; //变为申请状态
                    aCar->acceess_timestamp = slot;
                    aCar->role_condition = ROLE_H;
                    choose_commRange(aCar,slot); //决策通信半径
                    aItem = aItem->next;
                    continue;

                }else if(aCar->frontV != nullptr && aCar->rearV == nullptr){

                    aCar->slot_occupied = choose_slot(aCar,ROLE_T, slot);//随机选择时槽
                    aCar->slot_condition = ACCESS; //变为申请状态
                    aCar->acceess_timestamp = slot;
                    aCar->role_condition = ROLE_T;
                    choose_commRange(aCar,slot); //决策通信半径
                    aItem = aItem->next;
                    continue;

                }else{

                    aCar->slot_occupied = choose_slot(aCar,ROLE_I, slot);//随机选择时槽
                    aCar->slot_condition = ACCESS; //变为申请状态
                    aCar->acceess_timestamp = slot;
                    aCar->role_condition = ROLE_I;
                    choose_commRange(aCar,slot); //决策通信半径
                    aItem = aItem->next;
                    continue;

                }
            }
        }else if(aCar->slot_condition == ACCESS){
            if(slot < aCar->acceess_timestamp + SlotPerFrame){ //距离上次申请未超过一帧，不处理
                choose_commRange(aCar,slot); //决策通信半径,在一帧的中间要不要调整半径
                aItem = aItem->next;
                continue;
            }else if(slot >= aCar->acceess_timestamp + SlotPerFrame){
                // 确定身份是否被对应位置的车辆认可
                if(IsValidRole(aCar) == true){                 //如果被认可
                    // aCar->slot_occupied = tmp; // 使用上次选择的时槽，本次不更新
                    aCar->slot_condition = OCCUPIED;
                    aCar->occupied_timestamp = slot;
                    choose_commRange(aCar,slot); //决策通信半径
                    aItem = aItem->next;
                    continue;
                }else{                                      //若未得到对应位置车辆的认可
                    aCar->slot_occupied = choose_slot(aCar, aCar->role_condition, slot);
                    aCar->slot_condition = ACCESS;
                    aCar->acceess_timestamp = slot;
                    choose_commRange(aCar, slot);
                    aItem = aItem->next;
                    continue;
                }
            }else{
                cout<<"error! It should be handled before"<<endl;
            }
        }else if(aCar->slot_condition == OCCUPIED){
            //若得到了对应位置车辆的认可，保持不变
            if(IsMergingCollision(aCar, slot)){
                choose_commRange(aCar, slot);
                aItem = aItem->next;
                continue;
            }else{ //若未得到对应位置车辆的认可，重新进行时槽的选择，切换状态
                if(aCar->frontV == nullptr && aCar->rearV == nullptr){

                    aCar->slot_occupied = choose_slot(aCar,ROLE_S, slot);//随机选择时槽 single
                    aCar->slot_condition = SINGLE; //保持single不变
                    aCar->single_timestamp = slot;//更新single_timestamp 否则后面每个时槽都会重新选择（我们只要一帧决策一次就好）
                    aCar->role_condition = ROLE_S;
                    choose_commRange(aCar,slot); //决策通信半径
                    aItem = aItem->next;
                    continue;

                }else if(aCar->frontV == nullptr && aCar->rearV != nullptr){

                    aCar->slot_occupied = choose_slot(aCar,ROLE_H, slot);//随机选择时槽
                    aCar->slot_condition = ACCESS; //变为申请状态
                    aCar->acceess_timestamp = slot;
                    aCar->role_condition = ROLE_H;
                    choose_commRange(aCar,slot); //决策通信半径
                    aItem = aItem->next;
                    continue;

                }else if(aCar->frontV != nullptr && aCar->rearV == nullptr){

                    aCar->slot_occupied = choose_slot(aCar,ROLE_T, slot);//随机选择时槽
                    aCar->slot_condition = ACCESS; //变为申请状态
                    aCar->acceess_timestamp = slot;
                    aCar->role_condition = ROLE_T;
                    choose_commRange(aCar,slot); //决策通信半径
                    aItem = aItem->next;
                    continue;

                }else{

                    aCar->slot_occupied = choose_slot(aCar,ROLE_I, slot);//随机选择时槽
                    aCar->slot_condition = ACCESS; //变为申请状态
                    aCar->acceess_timestamp = slot;
                    aCar->role_condition = ROLE_I;
                    choose_commRange(aCar,slot); //决策通信半径
                    aItem = aItem->next;
                    continue;

                }
            }
        }

    }


    if(log_process_flag == true){
        log_process_file <<endl;
    }
}

// 根据角色，在对应得时槽资源池子里选一个时槽
int choose_slot(struct vehicle* aCar, int role, int slot){
    vector<int> slot_candidate;

    int a, b; //区间的开始和结束

    switch(role){
        case ROLE_H:
            a = 0;
            b = len_head_resource-1;
            break;
        case ROLE_S:
            a = len_head_resource;
            b = (SlotPerFrame/2 -1) - len_tail_resouce ;
            break;
        case ROLE_I:
            a = len_head_resource;
            b = (SlotPerFrame/2 -1) - len_tail_resouce ;
            break;
        case ROLE_T:
            a = SlotPerFrame/2  - len_tail_resouce;
            b = (SlotPerFrame/2 -1) ;
            break;
        default:cerr<<"Strange Role!!!"<<endl;
    }

    //如果是右边，则整体向右偏移
    if(aCar->resource_pool == 1){
        a += SlotPerFrame/2 ;
        b += SlotPerFrame/2;
    }
    //确定出可用的时槽
    for(int ii = a; ii <= b; ii++){
        if(aCar->THN[ii] == nullptr)
            slot_candidate.push_back(ii);
    }
    //进行时槽选择

    int len_candidate = slot_candidate.size();
//    if(len_candidate == 0)
//            return (a+b)/2;
    int choosed_slot = slot_candidate[rand() % len_candidate];

    return choosed_slot;
}




//处理一辆车在过去一帧收到的packets
void handle_packets(struct vehicle* aCar, int slot){
    struct Item *bItem;

    //在何时初始化这些统计量
    if(aCar->role_condition == SINGLE & (slot - aCar->single_timestamp)%SlotPerFrame == 0){


    }else if(aCar->role_condition == ACCESS && (slot - aCar->acceess_timestamp)%SlotPerFrame == 0){
        aCar->forntV_his = aCar->frontV;
        aCar->rearV_his = aCar->rearV;
        aCar->frontV = nullptr;
        aCar->rearV = nullptr;

        for(int ii = 0; ii < SlotPerFrame; ii++){
            aCar->prev_OHN[ii] = aCar->OHN[ii];
            aCar->OHN[ii] = nullptr;
            aCar->THN[ii] = nullptr;
        }

        //前车，后车，以及他们所对应的时槽使用情况
        (*(aCar->front_Vehicles)).clear();
        (*(aCar->rearV_Vehicles)).clear();
        (*(aCar->queue_Vehicles)).clear();

    }else if(aCar->role_condition == OCCUPIED){
        aCar->forntV_his = aCar->frontV;
        aCar->rearV_his = aCar->rearV;
        aCar->frontV = nullptr;
        aCar->rearV = nullptr;

        for(int ii = 0; ii < SlotPerFrame; ii++){
            aCar->prev_OHN[ii] = aCar->OHN[ii];
            aCar->OHN[ii] = nullptr;
            aCar->THN[ii] = nullptr;
        }

        //前车，后车，以及他们所对应的时槽使用情况
        (*(aCar->front_Vehicles)).clear();
        (*(aCar->rearV_Vehicles)).clear();
        (*(aCar->queue_Vehicles)).clear();
    }else{
        cerr<<"Strange Role!!"<<endl;
    }


    int len = (*(aCar->packets)).size();
    for( int ii = len-1; ii>=0; ii-- ){
        struct packet* pkt = (struct packet*)(*(aCar->packets))[ii];

        if(pkt->timestamp != slot){//一帧之外的包就不看了
            return;
        }

        if(pkt->condition == TX_COLI){//这个包属于两个同时发射，无法被感知到
           continue;
        }else if(pkt->condition == NO_COLI){//这种属于能够正常解的包

            //更新OHN
                int index = (pkt->timestamp)%SlotPerFrame;
                aCar->OHN[index] = pkt->srcVehicle;
                aCar->THN[index] = pkt->srcVehicle;

                //更新THN
                for(int i = 0; i < SlotPerFrame; i++){
                    if(pkt->OHN_snapshot[i]!=NULL && pkt->OHN_snapshot[i]!= aCar)
                        aCar->THN[i] = pkt->OHN_snapshot[i];
                }

                //更新front_Vehicles, rear_Vehicles
                if(strcmp(aCar->lane, pkt->srcVehicle->lane) == 0){ //处于相同车道
                    if(pkt->srcVehicle->pos > aCar->pos){//同车道前方
                        (*(aCar->front_Vehicles)).push_back(pkt->srcVehicle);
                    }else{//同车道后方
                        (*(aCar->rearV_Vehicles)).push_back(pkt->srcVehicle);
                    }

                    // 更新同车道前后最近邻车辆的信息

                    struct vehicle* tmp_rear = nearestVehicle(aCar, *(aCar->rearV_Vehicles));
                    struct vehicle* tmp_front = nearestVehicle(aCar, *(aCar->front_Vehicles));


                    if(tmp_rear != nullptr)
                        aCar->rearV = tmp_rear;
                    if(tmp_front != nullptr)
                        aCar->frontV = tmp_front;

                    // 更新车队信息,哪些车用了什么时槽
//                    for(auto tmp: pkt->hashtable){
//                        struct vehicle* bCar = tmp.first;
//                        int slot_index = tmp.second;
//                        aCar->queue_Vehicles[bCar] = slot_index;
//                    }
                    for(int ii = 0; ii < (*(pkt->hashtable_slot)).size(); ii++)
                        (*(aCar->queue_Vehicles)).push_back( (*(pkt->hashtable_vehicles))[ii]);

            }else{//对于非同车道的车，若听到了一个tail，则将其车队信息更新到THN不可用中
                if(pkt->srcVehicle->role_condition == ROLE_T){
//                        for(auto tmp: pkt->hashtable){
//                            struct vehicle* bCar = tmp.first;
//                            int slot_index = tmp.second;
//                            aCar->THN[slot_index] = bCar;
//                        }
                    for(int ii = 0; ii < (*(pkt->hashtable_slot)).size(); ii++){
                        struct vehicle* bCar = (*(pkt->hashtable_vehicles))[ii];
                        int slot_index = (*(pkt->hashtable_slot))[ii];

                        aCar->THN[slot_index] = bCar; //刷新THN
                        (*(aCar->queue_Vehicles)).push_back(bCar);
//                        (*(aCar->queue_Vehicles_slot)).push_back(slot_index);

                    }
                }
            }
        }else if(pkt->condition == RX_COLI){//这种属于1个接收端同时有多个包送达，不能解出包，但是能感知到包的存在
            int index = (pkt->timestamp)%SlotPerFrame;
            aCar->OHN[index] = collision_vehicle;            //填自己的时候就表明发生碰撞了
            aCar->THN[index] = collision_vehicle;
        }
    }

}




//根据此前一帧内收听到的包,看前车或后车是否认可了它，如果被认可，则保持该身份，否则，重新选择身份
bool IsValidRole(struct vehicle* aCar){
    if(aCar->frontV != aCar->forntV_his || aCar->rearV != aCar->rearV_his)
        return false;
    if(aCar->frontV != nullptr && aCar->frontV->OHN[aCar->slot_occupied] != aCar)
        return false;
    if(aCar->rearV != nullptr && aCar->rearV->OHN[aCar->slot_occupied] != aCar)
        return false;

    return true;

}


bool IsMergingCollision(struct vehicle* aCar, int slot){


    int len = (*(aCar->packets)).size();
    for(int ii = len-1; ii >=0; ii--){
        struct packet *pkt = (struct packet*) (*(aCar->packets))[ii];

        if(pkt->timestamp < slot){//只看最近的一个slot即可
            break;
        }

        if(pkt->condition == 0){         //condition为0的包车辆是听不到的，也解不出
            continue;
        }else if(pkt->condition == 1){
            //检测是否全部认可了自己
            if(pkt->OHN_snapshot[aCar->slot_occupied] != aCar){
                for(int ii =0; ii <SlotPerFrame; ii++){
                    if(aCar->prev_OHN[ii] == pkt->srcVehicle){//这里很关键哈，上一帧的OHN
                        return false;
                    }
                }
            }

            continue;
        }else if(pkt->condition == 2){      //对于冲突的包，只需要更新OHN为占用即可
            continue;
        }
    }
    return true;
}



void choose_commRange(struct vehicle* aCar, int slot){

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


//

// 找到一个链表（同车道）中，与aCar距离最近的车辆
struct vehicle* nearestVehicle(struct vehicle* aCar, vector<struct vehicle*> vehicleList){
    if(vehicleList.size() == 0)
        return nullptr;

    struct vehicle* ans = vehicleList[0];
    for(int i = 0; i< vehicleList.size();i++){
        if(abs(vehicleList[i]->pos - aCar->pos) < abs(ans->pos - aCar->pos)){
            ans = vehicleList[i];
        }
    }
    return ans;
}


double safe_range(struct vehicle* aCar, struct vehicle* frontV){
    if(frontV == nullptr)
        return (aCar->speed)*(aCar->speed)/2/(aCar->acc)+ Max_speed*0.1;
    else
        return ((aCar->speed)*(aCar->speed) - (frontV->speed)*(frontV->speed))/2/aCar->acc +Max_speed*0.1;

}
