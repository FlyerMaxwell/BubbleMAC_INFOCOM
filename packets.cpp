//
// Created by cyx02 on 2022/6/27.
//

#include "packets.h"
#include "parameters.h"
#include <cstdlib>


void log_packet(struct packet * aPkt, int slot){
    char output_collisions_path[100];
    FILE * Collisions_output;
    sprintf(output_collisions_path, "./simulation_result/bubble_packet_%d_%d.txt", SlotPerFrame, traffic_density);
    //printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%d\n",traffic_density);
    Collisions_output = fopen(output_collisions_path,"a");

    fprintf(Collisions_output,"No: %d, slot: %d, condition: %d\n", cnt_pkt++, slot, aPkt->slot_condition);

    // src
    fprintf(Collisions_output,"src id: %d\n",aPkt->srcVehicle->id);
    // for(int i = 0; i< SlotPerFrame; i++){
    //     fprintf(Collisions_output,"%d ",aPkt->srcVehicle->slot_oneHop[i]);
    // }
    // fprintf(Collisions_output,"\n");
    // for(int i = 0; i< SlotPerFrame; i++){
    //     fprintf(Collisions_output,"%d ",aPkt->srcVehicle->slot_twoHop[i]);
    // }
    // fprintf(Collisions_output,"\n");

    //dst
    fprintf(Collisions_output,"dst id: %d\n",aPkt->dstVehicle->id);
    // for(int i = 0; i< SlotPerFrame; i++){
    //     fprintf(Collisions_output,"%d ",aPkt->dstVehicle->slot_oneHop[i]);
    // }
    // fprintf(Collisions_output,"\n");
    // for(int i = 0; i< SlotPerFrame; i++){
    //     fprintf(Collisions_output,"%d ",aPkt->dstVehicle->slot_twoHop[i]);
    // }
    fprintf(Collisions_output,"\n");

    fclose(Collisions_output);
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

    return pkt;
}




