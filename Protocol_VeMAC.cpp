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
1. 每个车监听之前的一帧，更新OHN和THN的时槽使用情况
2. 根据状态进行时槽选择
    1）若ve_slot_condition == 0, 车辆未进行时槽选择，则其随机选择一个没有占用的slot，并设置slot_condition = 1
    2）若ve_slot_condition == 1, 车辆进行时槽请求，此时判断所有收到的packets中，OHN是否都认可了自己，如果认可了自己，则仍选择ve_slot_occupied，并将ve_slot_condition=2；若并未都认可自己，则继续随机选择时槽
    3）若ve_slot_condition == 2，车辆已选择固定时槽，此时判断所有收到的packets中，OHN是否都认可了自己，如果认可了自己，保持；如果并非都认可了自己，则ve_count_srp--;若ve_count_srp==0,则重新回到2）
3. 选择完后重置OHN和THN为NULL



对于VeMAC的全新理解：
 1. 什么时候进行决策？
 每个slot都要进行决策，决策的依据是之前听到的一个frame。如果没有听完一个完整的frame，则先听完一个frame再进行决策

 2. 广播半径选择多大？
 300米

 3. 广播时槽怎么选？
如果是第一次做决策，则根据上一帧听到的packets。先生成T(x)，即哪些时槽被实用了。让后生成A(x) = T（x）& R或者T(x) & L。然后在A（x）中随机选择一个时槽，进行申请。
怎么判断申请是否成功了呢？再收听一帧，看是否被所有的邻居都接受，若被所有邻居都接受，则切换为占有状态，否则重新选择。

对于一个处于占有状态的车辆，其必须时刻根据上一帧收听到的packets，来判断是否产生了merging collisions，如果产生了，则需要重新选择时槽。

slot_occupied: 表示占用的时槽
ve_slot_condition: 表示是正在申请还是已经占用


具体程序应该咋写呢？
对于每个时槽来说，
    先看距离出现是否过了一帧，如果没过，则处于单纯收听阶段，不进行时槽选择；（出现的第一帧进行收听）
    若刚好到了出现后的第一帧结束，则申请一个时槽，记录下时槽申请的时间（在这个时间后的一帧内只收听，不决策）

    如果处于时槽申请状态，判断是否已经监听满了一帧，如果没满，则处于单纯收听阶段，不进行时槽选择；
    如果刚好满了一帧，则判断是否得到了所有邻居的认可，如果得到了所有邻居的认可，则切换状态未占用，并保持使用该时槽；
    否则重新申请一个时槽；

    如果处于占用状态，判断刚听到的一个slot内是否有不认可他的节点，如果有，则发生了merging collision；进行时槽的重新选择，记录时槽选择时间
    如果没有不认可他的节点，则继续使用该时槽并保持

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

