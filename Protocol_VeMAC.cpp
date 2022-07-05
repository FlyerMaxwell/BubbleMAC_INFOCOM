//
// Created by cyx02 on 2022/7/5.
//

#include "Protocol_VeMAC.h"
#include "parameters.h"
#include <cstdlib>
#include "vehicle.h"
#include "BubbleProtocol.h"


/*
VeMAC
协议过程：
1. 每个车监听一帧，更新OHN和THN的时槽使用情况
2. 根据状态进行时槽选择
    1）若ve_slot_condition == 0, 车辆未进行时槽选择，则其随机选择一个没有占用的slot，并设置slot_condition = 1
    2）若ve_slot_condition == 1, 车辆进行时槽请求，此时判断所有收到的packets中，OHN是否都认可了自己，如果认可了自己，则仍选择ve_slot_occupied，并将ve_slot_condition=2；若并未都认可自己，则继续随机选择时槽
    3）若ve_slot_condition == 2，车辆已选择固定时槽，此时判断所有收到的packets中，OHN是否都认可了自己，如果认可了自己，保持；如果并非都认可了自己，则ve_count_srp--;若ve_count_srp==0,则重新回到2）
3. 选择完后重置OHN和THN为NULL
*/
void ve_mac(struct Duallist *ALL_Vehicles, int slot){

    struct Item *aItem, *bItem;
    struct vehicle* aCar;

    //step1: update OHN and THN
    aItem = ALL_Vehicles->head;
    while(aItem != NULL){
        aCar = (struct vehicle*)aItem->datap;               //[a,b] (rand()%(b-a+1))+a
        aCar->ve_check_flag = 1;//是否被周围邻居都认可

        struct Item* bItem =(struct Item *) aCar->packets.head;
        while(bItem != NULL){
            struct packet* pkt = (struct packet*) bItem->datap;
            if(pkt->condition == 0){         //condition为0的包车辆是听不到的，也解不出
                bItem = bItem->next;
            }else if(pkt->condition == 1){
                int index = (pkt->timestamp)%SlotPerFrame;
                aCar->OHN[index] = pkt->srcVehicle;

                //对于能解出来的包，按照其OHN_snapshot更新自己的THN
                for(int i = 0; i < SlotPerFrame; i++){
                    if(pkt->OHN_snapshot[i]!=NULL)
                        aCar->THN[i] = pkt->OHN_snapshot[i];
                }
                //检测是否全部认可了自己
                if(pkt->OHN_snapshot[aCar->slot_occupied] != aCar)
                    aCar->ve_check_flag = 0;

                bItem = bItem->next;
            }else if(pkt->condition == 2){//对于冲突的包，只需要更新OHN为占用即可
                int index = (pkt->timestamp)%SlotPerFrame;
                aCar->OHN[index] = pkt->srcVehicle;
                bItem = bItem->next;
            }
        }
        aItem = aItem->next;
    }
    //printf("sadasdasdsadasdas\n");

    //step2: choose slot according to ve_slot_condition
    aItem = ALL_Vehicles-> head;
    while(aItem != NULL){
        aCar = (struct vehicle*)aItem->datap;
        aCar->commRadius = 300;

        if(aCar->slot_condition == 0){
            aCar->slot_occupied = choose_slot(aCar,ROLE_VeMAC);
            aCar->slot_condition = 1;
        }else if(aCar->slot_condition == 1){
            if(aCar->ve_check_flag == 1){//若被认可
                aCar->slot_condition = 2;
                aCar->ve_count_srp = 3;
            }else{//若不被认可
                aCar->slot_occupied = choose_slot(aCar, ROLE_VeMAC);
                aCar->slot_condition = 1;
            }
        }else if(aCar->slot_condition == 2){
            if(aCar->ve_check_flag == 1){//这里改成随机选结果不变，说明你的vemac就没有work啊
                aCar->ve_count_srp = 3;
                printf("continue to occupie this slot.\n");
            }else if(aCar->ve_check_flag == 0 && aCar->ve_count_srp > 0){
                aCar->ve_count_srp--;
            }else if(aCar->ve_check_flag == 0 && aCar->ve_count_srp == 0){
                aCar->slot_occupied = choose_slot(aCar, ROLE_VeMAC);
                aCar->slot_condition = 1;
            }
        }
        aItem = aItem->next;
    }

    //step3: 用完了，重置OHN和THN为NULL
    aItem =ALL_Vehicles-> head;
    while(aItem != NULL){
        aCar = (struct vehicle*)aItem->datap;
        for(int i = 0; i< SlotPerFrame; i++){
            aCar->OHN[i] = NULL;
            aCar->THN[i] = NULL;
        }
        aItem = aItem->next;
    }
}

