//
// Created by cyx02 on 2022/6/27.
//

#include "UpLocation.h"
#include "common.h"
#include <stddef.h>
#include "vehicle.h"
#include <cstdio>
#include <stdlib.h>
#include "parameters.h"
#include "string.h"
#include <string>
#include <iostream>

int init_simulation(struct Duallist *ALL_Vehicles){
    struct Item *aItem;
    struct vehicle *aCar;
    aItem = ALL_Vehicles->head;
    while (aItem != NULL){
        aCar = (struct vehicle*)aItem->datap;

        //---需要初始化的内容---//
        aCar->handled = 0;
        //---需要初始化的内容---//

        aItem = aItem->next;
    }
    return 0;
}



// Update location.
void updateLocation(struct Duallist *ALL_Vehicles, int slot, string trace_path){
    FILE *fin = NULL;
    int flag;
    int timestep;
    struct Item *aItem, *bItem;
    //struct neighbour_car* tNeigh, *nNeigh, *bNeigh;
    struct vehicle *aCar, *bCar;
    int car_count = 0;

    trace_path += to_string(slot/20);//每个时槽是5毫秒，而车俩位置为50ms更新一次次
    trace_path += ".txt";

    //printf("Loading vehilces...\n");

    //sprintf(file_path, "C:\\Users\\cyx02\\Desktop\\SUMO\\highway\\transformerd\\carposition_%d.txt", slot);
    //fin = fopen(file_path, "r");

    //printf("%s\n", trace_path.c_str());
    fin = fopen(trace_path.c_str(), "r");
    if(!fin){cerr <<"create file error \n"; return ;}

    //读取文件，添加车辆
    while(fscanf(fin, "%d", &timestep)!=-1){
        struct vehicle *new_car;
        new_car=(struct vehicle*)malloc(sizeof(struct vehicle));

        new_car->handled = 2;//新车
        //load location information
        fscanf(fin, "%s", new_car->id);
        fscanf(fin, "%lf", &new_car->x);
        fscanf(fin, "%lf", &new_car->y);
        fscanf(fin, "%lf", &new_car->angle);
        fscanf(fin, "%s", new_car->type);
        fscanf(fin, "%lf", &new_car->speed);
        fscanf(fin, "%lf", &new_car->pos);
        fscanf(fin, "%s", new_car->lane);
        fscanf(fin, "%lf", &new_car->slope);
        fscanf(fin, "%lf", &new_car->flow);
        fscanf(fin, "%lf", &new_car->speed2);
        new_car->acc = 4.5;


        new_car->appear_timestamp = slot;
        new_car->state = state_S;
        new_car->QueueRole = -1;
        new_car->state_timestamp = slot;
        new_car->transmission_slot = -1;

        if(new_car->angle >= 0 && new_car->angle <180) new_car->resource_pool = 1;
        else new_car->resource_pool = 0;

        new_car->SameLaneFrontV = nullptr;
        new_car->SameLaneFrontV_his = nullptr;
        new_car->SameLaneRearV = nullptr;
        new_car->SameLaneRearV_his = nullptr;
        new_car->commRadius = 0;

        duallist_init(&new_car->neighbours);


        //查找new_Car是否已经存在， 若存在，flag=true；若不存在，则flag = false;遍历一次ALL_Vehicles双链表，看是否已经存在（id是否相等），若相等则flag=true；若不相等，则flag=false
        flag = false;
        bItem = (struct Item*)ALL_Vehicles->head;
        while(bItem != NULL){
            bCar = (struct vehicle*)bItem->datap;
            if (!strcmp(bCar->id, new_car->id)) {
                flag = true;
                break;
            }
            bItem = bItem->next;
        }

        //若之前已存在，则更新其运动学信息
        if(flag == true){
            bCar->x = new_car->x;
            bCar->y = new_car->y;
            bCar->angle = new_car->angle;
            bCar->speed = new_car->speed;
            bCar->pos = new_car->pos;
            strcpy(bCar->lane, new_car->lane);
            bCar->slope = new_car->slope;
            bCar->flow = new_car->flow;
            bCar->speed2 = new_car->speed2;


            bCar->resource_pool = new_car->resource_pool;
            bCar->handled = 1;//已有的车辆且处理

            free(new_car);
        }

        //若之前不存在,则添加新车
        if (flag == false){

            new_car->packets = new queue<struct packet*>;
            new_car->OHN = new vector<struct slot_info*>;
            new_car->THN = new vector<struct slot_info*>;
            new_car->prev_OHN = new vector<struct slot_info*>;
            new_car->frontQueue = new vector<struct slot_info*>;
            new_car->rearQueue = new vector<struct slot_info*>;


            Car_Number++;

            duallist_add_to_tail(ALL_Vehicles, new_car);
        }

    }

    fclose(fin);

    //处理消失的车(离开地图)，直接去掉即可。我的协议可以应对这种case，无非就是下个时刻听不到它了呗。没问题
    aItem = ALL_Vehicles->head;
    while(aItem != NULL){
        aCar = (struct vehicle*)aItem->datap;
        if(aCar->handled == 0){

            //防止内存泄露
           while(!(aCar->packets->empty())){
               free(aCar->packets->front());
               aCar->packets->pop();
           }
           for(auto tmp: *(aCar->OHN))
               free(tmp);
            for(auto tmp: *(aCar->THN))
                free(tmp);
           for(auto tmp :*(aCar->prev_OHN))
               free(tmp);
           for(auto tmp: *(aCar->frontQueue))
               free(tmp);
           for(auto tmp : *(aCar->rearQueue))
               free(tmp);

            delete aCar->packets;//车辆离开之前要进行统计再删除
            delete aCar->OHN;
            delete aCar->THN;
            delete aCar->prev_OHN;
            delete aCar->frontQueue;
            delete aCar->rearQueue;

            //节点删除
            struct Item* deleteItem = aItem;
            aItem = aItem->next;
            duallist_pick_item(ALL_Vehicles, deleteItem);

        }else{
            car_count++;
            cnt_cars++;
            aItem = aItem->next;
        }
    }

    return;
}

//handle neighbors： 处理邻居，将所有车辆的所在的九宫格内的车挂载到其潜在的neighbors中,即每个车辆的neighbors就是当前九宫格内的邻居。暴力，两层循环。这里暴力是为了后面每次遍历的时候能少遍历一点
void handle_neighbours(struct Duallist *ALL_Vehicles){
    struct Cell *aCell, *nCell;
    struct Item *aItem, *nItem;
    struct vehicle* aCar, *nCar;

    aItem =ALL_Vehicles-> head;
    while(aItem != NULL){
        aCar = (struct vehicle*)aItem->datap;
        duallist_destroy(&(aCar->neighbours), NULL);//先把之前的清空掉
        nItem = ALL_Vehicles-> head;
        while(nItem != NULL){
            nCar = (struct vehicle*)nItem->datap;
            //id不相等且处于两千米以内，则将其加入到neighbors
            if(strcmp(nCar->id, aCar->id)!=0 && distance_between_vehicle(aCar,nCar) < 300){
                duallist_add_to_tail(&(aCar->neighbours), nCar);//将id不同的车加入到neighbor list。
            }
            nItem = nItem->next;
        }
        aItem = aItem->next;
    }
}


