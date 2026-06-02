#ifndef _CONTRAL_CLAW_H
#define _CONTRAL_CLAW_H

#include "system_init.h"
#include "contral.h"

//通用高度
/*对于物料转盘抓取时候的80-100mm
转盘  物块5.5cm处爪子高度
80mm  13.5cm 99

85mm 14.0cm 105

90mm 14.5cm  111

95mm 15.0cm 117

100mm 15.5cm 123

计算公式为：爪子脉冲高度=99+(转盘实际高度-80)/20*24

*/
#define claw_most_up    265   //            升到最高的位置            
#define camera_position  2       //               扫码的位置
#define claw_most_down  14       //          降低到最低的位置
#define circle_capture1  80  //          一层放置靶心识别的高度
#define circle_capture3  60   //     任务二物块放置专有打靶高度
#define circle_capture2  114    //物料盘颜色识别位置，要高于等会抓取物块的高度 //

//普通经典物块
#define claw_block_get    40      //      地上抓物块的高度  4

#define put_block_down   35      //      地上放物块的高度  4

extern int16_t claw_block_get1 ;    //     从物料转盘上抓物块的高度    需要进行更改的，设为变量，可以软件进行修改  //

#define claw_block_put   200 //        车上放物块的位置

#define get_block_down    195   //       从车上抓取要把物块放下去的位置  168
//#define get_block2_down    190   //       从车上抓取要把物块2放下去的位置  168

#define put_block_down2   87   //     把物块叠放在二层的高度   //

#define claw_block_putF2  165    //车上把物块放到转盘二层的高度   //


#define servo1_angle_color   10     //舵机1颜色识别状态下的角度
#define servo4_angle_color  10     //舵机4颜色识别状态下的角度

//任务二物块
#define servo1_angle_init 110     //舵机1正常状态下的角度
#define servo4_angle_init 110      //舵机4正常状态下的角度

#define servo1_angle_task2  10     //舵机1任务2状态下的角度
#define servo4_angle_task2 10     //舵机4任务2状态下的角度


//奖杯关于高度

#define claw_block2_get    52      //      地上抓奖杯的高度  28

#define put_block2_down   50       //      地上放奖杯的高度，一般用于机械臂收回情况放季军奖杯

#define claw_block2_get1  103    //     从物料转盘上抓物块2的高度

#define claw_block2_put   210 //        车上放奖杯的位置

#define get_block2_down    212   //       从车上抓取要把奖杯放下去的位置

#define put_block2_down2   87   //     把物块2叠放在二层的高度

#define put_block_champion 31  //     冠军领奖台的位置  34

#define put_block_runnerup 10  //     亚军军领奖台的位置  12

#define put_block_num3   0//         季军领奖台的位置，一般用于机械臂伸出放季军奖杯




typedef struct {
	int16_t position_now;             //当前值
	int16_t position_target;           //目标值
	int16_t position_temp;             //临时值（中间差值）
}CLAW_POSITION;

extern CLAW_POSITION claw;





void claw_position(int16_t position);
void claw_position2(int16_t position);

void arrive_camera(void);
void arrive_most_up(void);
void arrive_most_down(void);
void arrive_circle_capture(void);
void arrive_circle_capture2(void);
void arrive_color_reco(void);

void arrive_block_get(void);
void arrive_block_put(void);
void arrive_block_get1(void);
void arrive_car_put(void);
void arrive_car_get(void);
void arrive_car_get2(void);
void arrive_put_down2(void);
void arrive_block_putF2(void);

void arrive_block_put_champion(void);
void arrive_block_put_runnerup(void);
void arrive_block_put_num3(void);

void arrive_block2_get(void);
void arrive_block2_put(void);
void arrive_block2_get1(void);
void arrive_car2_put(void);
void arrive_car2_get(void);
void arrive_put2_down2(void);



void claw_get_block1(void);
void claw_get_block2(void);
void claw_get_block3(void);
void claw_get_block4(void);
void claw_get_block5(void);

void claw_task2_reco(void);

void claw_put_block1(void);
void claw_put_block2(void);
void claw_put_block3(void);	
void claw_put_block4(void);
void claw_put_block5(void);

void claw_get2_block1(void);
void claw_get2_block2(void);
void claw_get2_block3(void);

void claw_put_num3(void);
void claw_put_runnerup(void);
void claw_put_champion(void);
void claw_put2_block(void);





void claw_home(void);



#endif
