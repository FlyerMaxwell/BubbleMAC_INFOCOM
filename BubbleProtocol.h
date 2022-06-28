//
// Created by cyx02 on 2022/6/27.
//

#ifndef BUBBLEMAC_INFOCOM_BUBBLEPROTOCOL_H
#define BUBBLEMAC_INFOCOM_BUBBLEPROTOCOL_H


#include<vector>
using namespace std;

void bubble_protocol_commRange(struct Duallist * ALL_vehicles, int slot);
void bubble_protocol_slot(struct Duallist *ALL_Vehicles, int slot);


void bubble_protocol(struct Duallist *ALL_Vehicles, int slot);
struct vehicle* nearestVehicle(struct vehicle* aCar, vector<struct vehicle*> &vehicleList);
double safe_range(struct vehicle* aCar, struct vehicle* frontV);



#endif //BUBBLEMAC_INFOCOM_BUBBLEPROTOCOL_H
