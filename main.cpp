#include <iostream>
#include "vehicle.h"
#include "parameters.h"
#include "common.h"
#include "UpLocation.h"
#include "communication.h"
#include "BubbleProtocol.h"
#include "Protocol_80211.h"
#include "Protocol_VeMAC.h"

using namespace std;


#define p_Bubble 0
#define p_80211 1
#define p_VeMAC 2

int main(int argc, char *argv[]) {

    string trace_path;
    int protocol = argv[2][0] - '0'; //0: bubble 1:802.11 2:veMAC

    trace_path = argv[1];       //读取文件路径
    cout<< "Current trace file is from:"<< trace_path<<"..."<<endl;

    switch (protocol) {
        case 0:
            cout<<"Current Protocol is Bubble MAC..."<<endl;
            break;
        case 1:
            cout<<"Current Protocol is 802.11p MAC..."<<endl;
            break;
        case 2:
            cout<<"Current Protocol is VeMAC..."<<endl;
            break;
        default:
            cout<<"error! Please input a right MAC protocol"<<endl;
    }


    srand(0);

    int slot_start = 0;
    int slot_end = 300;
    int slot_step = 1;

    struct Duallist ALL_Vehicles;
    duallist_init(&ALL_Vehicles);


    for(int slot = slot_start; slot < slot_end; slot += slot_step){
        //cout<<"slot = "<< slot<<endl;

        if(slot % UpLocSlot == 0){
            init_simulation(&ALL_Vehicles);
            updateLocation(&ALL_Vehicles, slot, trace_path);
            handle_neighbours(&ALL_Vehicles);
            //cout<<"The location of vehicles has been updated, Current slot = "<<slot<<endl;
            //printVehilces(&ALL_Vehicles);
        }


        if(slot % SlotPerFrame ==0){
            switch (protocol){
                case p_Bubble:
                    bubble_handle_packets(&ALL_Vehicles, slot);
                    bubble_protocol_commRange(&ALL_Vehicles, slot);
                    //cout<<"after commRange deter-----------------------"<<endl;
                    //printVehilces(&ALL_Vehicles);
                    bubble_protocol_slot(&ALL_Vehicles, slot);
                    //cout<<"after slot deter------------------------"<<endl;
                    //printVehilces(&ALL_Vehicles);
                    break;
                case p_80211:
                    mac_80211p(&ALL_Vehicles, slot);
                    //printVehilces(&ALL_Vehicles);
                    break;
                case p_VeMAC:
                    ve_mac(&ALL_Vehicles, slot);
                    //printVehilces(&ALL_Vehicles);
                    break;
                default:break;
            }
        }
//        //--------MAC Protocol-----------//
//         //802.11p
//         if(slot % SlotPerFrame == 0){
//             mac_80211p(&ALL_Vehicles, slot);
//             printVehilces(&ALL_Vehicles);
//         }
//
//        //VeMAC
//        if(slot % SlotPerFrame == 0){
//            ve_mac(&ALL_Vehicles, slot);
//            printVehilces(&ALL_Vehicles);
//        }
//
//        //Bubble MAC, determine the communication range and slot
//        if(slot % SlotPerFrame == 0){
//            bubble_handle_packets(&ALL_Vehicles, slot);
//            bubble_protocol_commRange(&ALL_Vehicles, slot);
//            cout<<"after commRange deter-----------------------"<<endl;
//            printVehilces(&ALL_Vehicles);
//            bubble_protocol_slot(&ALL_Vehicles, slot);
//            cout<<"after slot deter------------------------"<<endl;
//            printVehilces(&ALL_Vehicles);
//        }


        //handle the transmitter at each slot
        handle_transmitter(&ALL_Vehicles, slot);
        //handle the receiver at each slot
        handle_receiver(&ALL_Vehicles, slot);
    }
    printf("Total Cars: %d\n cnt_pkt_tx: %d\n cnt_pkt_0: %d\n cnt_pkt_1: %d\n cnt_pkt_2: %d\n", Car_Number, cnt_pkt_tx, cnt_pkt_0, cnt_pkt_1, cnt_pkt_2);
    return 0;
}
