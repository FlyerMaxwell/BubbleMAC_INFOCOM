#include <iostream>
#include <fstream>
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
    cout<<"hello world"<<endl;

    string trace_path;
    int protocol = argv[2][0] - '0'; //0: bubble 1:802.11 2:veMAC

    trace_path = argv[1];       //读取文件路径


    trace_path = "C:\\Users\\cyx02\\Desktop\\transformed\\transformed\\carposition_";

    cout<< "Current trace file is from:"<< trace_path <<"..."<<endl;
    cout<<"Current Protocol is Bubble MAC..."<<endl;



    //仿真的详细log路径，记录所有发生的事件
    string log_dir = ".\\log_";
    log_dir += to_string(protocol);
    log_dir += ".txt";

    logfile.open(log_dir);
    if(!logfile){cerr <<"create file error \n"; return 1;}


    srand(0);

    int slot_start = 0; //每个slot是0.5毫秒
    int slot_end = 600; //19200 挂掉了
    int slot_step = 1;

    struct Duallist ALL_Vehicles;
    duallist_init(&ALL_Vehicles);

    logfile << "The simulation is starting..."<<endl;
    logfile <<"slot start from "<< slot_start<<" to "<<slot_end<<", the slot step is "<<slot_step<<endl<<endl;

    Car_Number = 0;

    for(int slot = slot_start; slot < slot_end; slot += slot_step){
         //cout<<"slot = "<< slot<<endl;

         //更新位置
        if(slot % UpLocSlot == 0){
            init_simulation(&ALL_Vehicles);
            updateLocation(&ALL_Vehicles, slot, trace_path);
            handle_neighbours(&ALL_Vehicles);
            //cout<<"The location of vehicles has been updated, Current slot = "<<slot<<endl;
            //printVehilces(&ALL_Vehicles);

//            logfile<<"Event: Update location! Current slot = "<<slot<<", Car_Number="<<Car_Number<<endl;
//            logVehilcesInfo(&ALL_Vehicles, logfile);
//            logfile<<endl;
        }

        bubble(&ALL_Vehicles, slot);
//        if(slot % SlotPerFrame == 0){
//            bubble_handle_packets(&ALL_Vehicles, slot);
//            bubble_protocol_commRange(&ALL_Vehicles, slot);
//                    // cout<<"after commRange deter-----------------------"<<endl;
//                    //printVehilces(&ALL_Vehicles);
//            bubble_protocol_slot(&ALL_Vehicles, slot);
//                    //cout<<"after slot deter------------------------"<<endl;
//                    //printVehilces(&ALL_Vehicles);
//                    //if(slot >=SlotPerFrame){
//            logfile<<"Event: Bubble MAC! Current slot = "<<slot<<endl;
//            logfile<<"After the MAC, the info of vehicles are...."<<endl;
//            logVehilcesInfo(&ALL_Vehicles, logfile);
//            logfile<<endl<<endl;
//
//        }
////
//        //handle the transmitter at each slot
//       // if(slot>=SlotPerFrame)
//            logfile<<"Handle tx:"<<endl;
//
        handle_transmitter(&ALL_Vehicles, slot);
//
//        //if(slot>=SlotPerFrame)
//            logfile<<endl;
//
//        //handle the receiver at each slot
//        //if(slot>=SlotPerFrame)
//            logfile<<"Handle rx:"<<endl;
//
        handle_receiver(&ALL_Vehicles, slot);
//
//        //if(slot>=SlotPerFrame)
//            logfile<<endl;
    }
    printf(" Total Cars: %d\n Total slot:%d \n cnt_pkt_tx: %d\n cnt_pkt_0: %d\n cnt_pkt_1: %d\n cnt_pkt_2: %d\n Receive Rate=%lf \n", Car_Number,slot_end,  cnt_pkt_tx, cnt_pkt_0, cnt_pkt_1, cnt_pkt_2, (double)(cnt_pkt_1)/(cnt_pkt_0+cnt_pkt_1+cnt_pkt_2));

    logfile.close();
    return 0;
}
