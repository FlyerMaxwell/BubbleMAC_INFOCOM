//
// Created by cyx02 on 2022/7/5.
//

#include "Protocol_80211.h"
#include "parameters.h"
#include "vehicle.h"
#include <cstdlib>

/*
802.11p
协议过程：without retransmission, 等效为随机选择时槽
*/
void mac_80211p(struct Duallist *ALL_Vehicles, int slot){
    // For this protocol, each vehicle randomly choose a slot to transmit its WSM.

    int a = 0;
    int b = SlotPerFrame - 1;

    struct Item *aItem;
    struct vehicle* aCar;

    aItem =ALL_Vehicles-> head;
    while(aItem != NULL){
        aCar = (struct vehicle*)aItem->datap;               //[a,b] (rand()%(b-a+1))+a
        aCar->slot_occupied = rand()%(b - a + 1) + a;       //[0,199]
        //printf("!!!!!!!!!=++++++++++++=%d\n", aCar->slot_occupied);
        aCar->commRadius = 300;                             //set commRange as 300m
        aItem = aItem->next;
    }
}