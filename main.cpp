#include <iostream>
#include "vehicle.h"
#include "parameters.h"
#include "common.h"
#include "UpLocation.h"
#include "communication.h"
#include "BubbleProtocol.h"

using namespace std;

int main() {
    cout << "Hello, World!" << endl;
    srand(0);

    int slot_start = 0;
    int slot_end = 300;
    int slot_step = 1;

    struct Duallist ALL_Vehicles;
    duallist_init(&ALL_Vehicles);

    for(int slot = slot_start; slot < slot_end; slot += slot_step){
        cout<<"slot = "<< slot<<endl;

        if(slot % UpLocSlot == 0){
            init_simulation(&ALL_Vehicles);
            updateLocation(&ALL_Vehicles, slot);
            handle_neighbours(&ALL_Vehicles);
            printf("The location of vehicles has been updated, Current slot = %d\n", slot);
            printVehilces(&ALL_Vehicles);
        }

        //Bubble MAC, determine the communication range and slot
        if(slot % SlotPerFrame == 0){
            bubble_handle_packets(&ALL_Vehicles, slot);
            bubble_protocol(&ALL_Vehicles, slot);
        }


        //handle the transmitter at each slot
        handle_transmitter(&ALL_Vehicles, slot);
        //handle the receiver at each slot
        handle_receiver(&ALL_Vehicles, slot);
    }
    printf("Total Cars: %d\n cnt_pkt_tx: %d\n cnt_pkt_0: %d\n cnt_pkt_1: %d\n cnt_pkt_2: %d\n", Car_Number, cnt_pkt_tx, cnt_pkt_0, cnt_pkt_1, cnt_pkt_2);
    return 0;
}
