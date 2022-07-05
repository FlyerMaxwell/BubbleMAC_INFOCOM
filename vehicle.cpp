//
// Created by cyx02 on 2022/6/27.
//

#include "vehicle.h"
#include "math.h"
#include <cstdio>
#include <stddef.h>
#include <iostream>

using namespace  std;
double distance_between_vehicle(const struct vehicle* aCar, const struct vehicle* bCar){
    return sqrt((aCar->x - bCar->x)*(aCar->x - bCar->x) + (aCar->y - bCar->y)*(aCar->y - bCar->y));
}


void printVehilces(struct Duallist *ALL_Vehicles){
    struct Item *aItem;
    struct vehicle* aCar;

    aItem =ALL_Vehicles-> head;
    while(aItem != NULL){
        aCar = (struct vehicle*)aItem->datap;
        cout<<"handled="<<aCar->handled<<" ,id="<<aCar->id<<" ,x="<<aCar->x <<" ,y="<<aCar->y <<" ,CommRange="<<aCar->commRadius <<" ,slot_occupied="<< aCar->slot_occupied <<" ,slot_condition="<<aCar->slot_condition<<" ,pos="<<aCar->pos<<", speed="<<aCar->speed<<", a="<<aCar->a<<", slot_resource="<<aCar->resource_pool<<", angle="<<aCar->angle<<endl;

        aItem = aItem->next;

       // (aCar->queue_Vehicles)[aCar]++;
        //(aCar->queue_Vehicles).clear();
    }
}