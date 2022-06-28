//
// Created by cyx02 on 2022/6/27.
//

#include "vehicle.h"
#include "math.h"
#include <cstdio>
#include <stddef.h>

double distance_between_vehicle(const struct vehicle* aCar, const struct vehicle* bCar){
    return sqrt((aCar->x - bCar->x)*(aCar->x - bCar->x) + (aCar->y - bCar->y)*(aCar->y - bCar->y));
}


void printVehilces(struct Duallist *ALL_Vehicles){
    struct Item *aItem;
    struct vehicle* aCar;

    aItem =ALL_Vehicles-> head;
    while(aItem != NULL){
        aCar = (struct vehicle*)aItem->datap;
        printf("handled=%d, id= %s, x=%lf, y=%lf, CommRage=%lf, slot_occupied=%d, slot_condition=%d \n", aCar->handled, aCar->id, aCar->x, aCar->y, aCar->commRadius, aCar->slot_occupied, aCar->slot_condition);
        aItem = aItem->next;
    }
}