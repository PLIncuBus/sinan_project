#ifndef __RTK_H
#define __RTK_H

#include "main.h"


typedef struct 
{
    float lon;//经度
    float lat;//纬度
    float Ait;//高度
}RTK_Pos_t;


typedef struct 
{
    uint8_t buffer[255];
    RTK_Pos_t Pos;

}RTK_t;

extern RTK_t RTK; 


#endif

