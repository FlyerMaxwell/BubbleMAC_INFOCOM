//
// Created by cyx02 on 2022/6/27.
//

#include "packets.h"
#include "parameters.h"
#include <cstdlib>


void log_packet(struct packet * aPkt, int slot, ofstream &logfile){
    logfile<<"  Packet Info:"<<endl;
    logfile<<"    --SrcVehicle="<<aPkt->srcVehicle->id<<", Src_slot_occupied="<<aPkt->srcVehicle->slot_occupied<<", DstVehicle="<<aPkt->dstVehicle->id<<", Src_slot_occupied="<<aPkt->dstVehicle->slot_occupied<<", slot="<<slot<<", condition="<<aPkt->condition<<endl;
    logfile<<"    --ONH_snapshot: ";
    for(int i = 0; i < SlotPerFrame; i++){
        //logfile<<"i="<<i<<" V="<<",";
        if(aPkt->OHN_snapshot[i] != nullptr)
            logfile<<"i="<<i<<" V="<<(aPkt->OHN_snapshot[i])->id<<",";
    }
    logfile<<endl;
}



//generate_packet
struct packet * generate_packet(struct vehicle *aCar, struct vehicle *bCar, int slot, int condition){
    struct packet* pkt;
    pkt = (struct packet*)malloc(sizeof(struct packet));
    pkt->timestamp = slot;
    // pkt->slot_condition = 0;//还没有发生碰撞
    pkt->srcVehicle = aCar;
    pkt->dstVehicle = bCar;
    pkt->condition = condition;

    for(int i = 0; i < SlotPerFrame; i++){
        pkt->OHN_snapshot[i] = aCar->OHN[i];
    }

    pkt->hashtable_vehicles = new vector<struct vehicle*>;   //作用是传递车队信息
    pkt->hashtable_slot = new vector<int>;

    int len = aCar->queue_Vehicles->size();
    for(int i = 0; i< len; i++){
        (*(pkt->hashtable_vehicles)).push_back((*(aCar->queue_Vehicles))[i]);
        (*(pkt->hashtable_slot)).push_back((*(aCar->queue_Vehicles))[i]->slot_occupied);
    }


//    for(int ii = 0; ii< SlotPerFrame; ii++){
//        struct vehicle* bCar = aCar->OHN[ii];
//        int slot_index = ii;
//
//        if (aCar->OHN[ii] != nullptr) {
//            (*(pkt->hashtable_vehicles)).push_back(bCar);
//            (*(pkt->hashtable_slot)).push_back(slot_index);
//        }
//
//    }


    return pkt;
}




