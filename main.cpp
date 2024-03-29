#include <iostream>
#include <fstream>
#include "vehicle.h"
#include "parameters.h"
#include "common.h"
#include "UpLocation.h"
#include "communication.h"
#include "BubbleProtocol.h"
#include "string.h"


using namespace std;

int main(int argc, char *argv[]) {

    // Set the trace path and traffic density
    string trace_path;
//    int protocol = argv[2][0] - '0'; //0: bubble 1:802.11 2:veMAC
//    trace_path = argv[1];       //读取文件路径
    trace_path = "C:\\Users\\cyx02\\Desktop\\transformed\\transformed\\carposition_";
    traffic_density = 240;
    Max_speed = 22;

    cout<< "Current trace file is from:"<< trace_path <<"..."<<endl;
    cout<<"Current Protocol is Bubble MAC..."<<endl;


    // Set the dir of log
    string log_dir_process = ".\\log_process_";
    log_dir_process += to_string(traffic_density);
    log_dir_process += ".txt";

    string log_dir_statistic = ".\\log_statistic_";
    log_dir_statistic += to_string(traffic_density);
    log_dir_statistic += ".txt";

    log_process_file.open(log_dir_process);
    if(!log_process_file){cerr <<"create file error \n"; return 1;}

    log_statistic_file.open(log_dir_statistic);
    if(!log_statistic_file){cerr <<"create file error \n"; return 1;}


    srand(0);

    int slot_start = 0; //每个slot是0.5毫秒
    int slot_end =1000; //19200 挂掉了
    int slot_step = 1;

    struct Duallist ALL_Vehicles;
    duallist_init(&ALL_Vehicles);

    log_process_file << "The simulation is starting..."<<endl;
    log_process_file <<"slot start from "<< slot_start<<" to "<<slot_end<<", the slot step is "<<slot_step<<endl<<endl;

    // Set a collision flag. All collisions point to this address
    collision_vehicle = new struct vehicle;
    strcpy(collision_vehicle->id,"rx_collision");


    // Start Simulation
    for(int slot = slot_start; slot < slot_end; slot += slot_step){
        if(slot %SlotPerFrame == 0)
            cout<<"Frame = "<< slot/SlotPerFrame<<endl;

         //Update Location every UpLocSlot slot
        if(slot % UpLocSlot == 0){
            init_simulation(&ALL_Vehicles);
            updateLocation(&ALL_Vehicles, slot, trace_path);
            handle_neighbours(&ALL_Vehicles);
        }

        //Bubble MAC, determine the CommRange and Slot for each Vehicle
        bubble(&ALL_Vehicles, slot);

        // Communication
        handle_transmitter(&ALL_Vehicles, slot);
        handle_receiver(&ALL_Vehicles, slot);

        if(log_statistic_flag == true && (slot % SlotPerFrame) == 0){
            log_statistic_file<<slot/SlotPerFrame<<" "<<cnt_cars<<" "<<cnt_tx_collision<<" "<<cnt_rx_normal<<" "<<cnt_rx_colli<<" "<<cnt_pkt_tx<<" "<<cnt_pkt_tx_normal<<" "<<cnt_frontV_normal<<" "<<cnt_rearV_normal<<" "<<cnt_frontV_colli<<" "<<cnt_rearV_Colli<<endl;

            // Reset the Statisical Para
            cnt_cars = 0;
            cnt_tx_collision = 0;  //两个发射碰撞
            cnt_rx_normal = 0;  //正常收包
            cnt_rx_colli = 0;  //产生碰撞的包
            cnt_pkt_tx = 0;
            cnt_pkt_tx_normal = 0;
            cnt_frontV_normal = 0;
            cnt_rearV_normal = 0;
            cnt_frontV_colli = 0;
            cnt_rearV_Colli = 0;
        }

    }
    //printf(" Total Cars: %d\n Total slot:%d \n cnt_pkt_tx: %d\n cnt_tx_collision: %d\n cnt_pkt_1: %d\n cnt_pkt_2: %d\n Receive Rate=%lf \n", Car_Number,slot_end,  cnt_pkt_tx, cnt_tx_collision, cnt_pkt_1, cnt_pkt_2, (double)(cnt_pkt_1)/(cnt_tx_collision+cnt_pkt_1+cnt_pkt_2));
    cout<<" Total Cars:"<<Car_Number<<endl;
    cout<<" Total Slot:"<<slot_end<<endl;
    log_process_file.close();
    return 0;
}
