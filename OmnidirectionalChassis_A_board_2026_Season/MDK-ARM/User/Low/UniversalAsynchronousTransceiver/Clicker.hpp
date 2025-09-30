#ifndef _CLICKER_HPP_
#define _CLICKER_HPP_ 

#include "stdint.h"
#include "string.h"

extern uint8_t pData[18];

namespace Clicker{

    class DR16
    {
        public:

            struct 
            { 
                uint16_t ch1; //ÓÒ±ßx
                uint16_t ch0; //ÓÒ±ßy
                uint16_t ch3; //×ó±ßx  ¶«±±Ìì
                uint16_t ch2; //×ó±ßy
                uint8_t s1;
                uint8_t s2;
            }rc;
            struct 
            {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
            }mouse;
            struct 
            {
                uint16_t v;
            }key;
            struct
            {
                uint16_t reserved;
            }reserved_;
            

            float vx_left;
            float vy_left;
            float vx_right; 
            float vy_right;
            float Roller;

            float mou_x;
            float mou_y;
            float mou_z;

            bool SpeedControl_flag = false;
            bool PositionControl_flag = false;
            bool EnablePosition_flag = false;
            bool DisablePosition_flag = false;
            bool sentry = false;//bushi

            bool right_above = false;
            bool right_under = false;
            bool right_left  = false;
            bool right_right = false;

            bool normal = true;
            int transform_flag = 0;

        void DR16DataUpdata(uint8_t *pData);
    };
}


#endif
