#include "contral_claw.h"

CLAW_POSITION claw;

int16_t claw_block_get1 = 103 ;   //103

/********************
函数功能 : 爪子位置函数  单位mm
输入参数 : 无
输出参数 ：无
直径 16mm 
转一圈  8*2*3.14=50.24 
x/mm = 3200/50.24
x = 64*mm
**********************/
void claw_position(int16_t position)
{
	uint32_t pulse=0;
	claw.position_now = claw.position_now + position; // 更新当前位置
	
	if(claw.position_now<0 || claw.position_now>claw_most_up)
	{
		claw.position_now=claw.position_now - position;
	}
	else
	{
		// 方向
		if(position>0)
		{
				Motor1_DIR(1);   // 向上
		}
		else
		{
				position=-position;
				Motor1_DIR(0);   // 向下
		}
		pulse=(64*position);
		distance1=pulse;
		stepPosition1=0;
		MSD_Move1(pulse,170,170,220); //24 24 30
		while(1)
		{
			if(stepPosition1 == distance1)
			{
				break;
			}
		}
	}
}
/********************
函数功能 : 爪子位置函数  单位mm
输入参数 : 无
输出参数 ：无
是否结束由外部控制
*********/
void claw_position2(int16_t position)
{
	uint32_t pulse=0;
	claw.position_now = claw.position_now + position; // 更新当前位置
	if(claw.position_now<0 || claw.position_now>claw_most_up)
	{
		claw.position_now=claw.position_now - position;
	}
	else
	{
		// 方向
		if(position>0)
		{
				Motor1_DIR(1);   // 向上
		}
		else
		{
				position=-position;
				Motor1_DIR(0);   // 向下
		}
		pulse=(64*position);
		distance1=pulse;
		stepPosition1=0;
		MSD_Move1(pulse,100,100,200);
	}
}

////////////////////////////////////////////////////////////爪子高度
/********************
函数功能 : 到达摄像头扫码的位置
输入参数 : 无
输出参数 ：无
**********************/
void arrive_camera(void)
{
	claw.position_target = camera_position;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}

/********************
函数功能 : 到达最高处
输入参数 : 无
输出参数 ：无
**********************/
void arrive_most_up(void)
{
	//Servo_SetAngle14(-110,-116);
	claw.position_target = claw_most_up;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}

/********************
函数功能 : 到达最低处
输入参数 : 无
输出参数 ：无
**********************/
void arrive_most_down(void)
{
	claw.position_target = claw_most_down;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}

/********************
函数功能 : 到达靶心识别的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_circle_capture(void)
{
	claw.position_target = circle_capture1;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}

/********************
函数功能 : 到达二层靶心识别的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_circle_capture2(void)
{
	claw.position_target = circle_capture3;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}

/********************
函数功能 : 到达物料盘识别颜色的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_color_reco(void)
{
	claw.position_target = circle_capture2;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}


//////经典物块的高度

/********************
函数功能 : 到达地面抓物块的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_block_get(void)
{
	claw.position_target = claw_block_get;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}

/********************
函数功能 : 到达地面放物块的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_block_put(void)
{
	claw.position_target = put_block_down;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}

/********************
函数功能 : 到达冠军领奖台的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_block_put_champion(void)
{
	claw.position_target = put_block_champion;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}


/********************
函数功能 : 到达亚军领奖台的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_block_put_runnerup(void)
{
	claw.position_target = put_block_runnerup;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}

/********************
函数功能 : 到达季军领奖台的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_block_put_num3(void)
{
	claw.position_target = put_block_num3 ;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}

/********************
函数功能 : 到达从物料盘抓物块的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_block_get1(void)
{
	claw.position_target = claw_block_get1;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}

/********************
函数功能 : 到达从物料盘抓物块的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_block_putF2(void)
{
	claw.position_target = claw_block_putF2;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}



//////

/********************
函数功能 : 到达在车上放物块的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_car_put(void)
{
	claw.position_target = claw_block_put;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}


/********************
函数功能 : 到达车上抓物块1的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_car_get(void)
{
	claw.position_target = get_block_down;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}


/********************
函数功能 : 到达车上抓物块2的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_car_get2(void)
{
	claw.position_target = get_block2_down;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}


/********************
函数功能 : 到达物块放在第二层高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_put_down2(void)
{
	claw.position_target = put_block_down2;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}


////针对于决赛物料2的新动作与高度

/********************
函数功能 : 到达地面抓物块2的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_block2_get(void)
{
	claw.position_target = claw_block2_get;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}

/********************
函数功能 : 到达地面放物块2的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_block2_put(void)
{
	claw.position_target = put_block2_down;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}

/********************
函数功能 : 到达从物料盘抓物块2的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_block2_get1(void)
{
	claw.position_target = claw_block2_get1;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}


/********************
函数功能 : 到达在车上放物块2的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_car2_put(void)
{
	claw.position_target = claw_block2_put;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}


/********************
函数功能 : 到达车上抓物块2的高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_car2_get(void)
{
	claw.position_target = get_block2_down;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}

/********************
函数功能 : 到达物块2放在第二层高度
输入参数 : 无
输出参数 ：无
**********************/
void arrive_put2_down2(void)
{
	claw.position_target = put_block2_down2;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}


////////////////////////////////////////////////////爪子具体动作

////对于经典物块
/********************
函数功能 : 爪子从地面把物块放车上位置1
输入参数 : 无
输出参数 ：无
**********************/
void claw_get_block1(void)
{
	claw_turn0();
	claw_open();
	delay_ms(300);
	arrive_block_get();
	delay_ms(300);
	claw_close();
	delay_ms(400);
	arrive_most_up();
	claw_turn1();
	delay_ms(600);
	arrive_car_put();
	claw_open1();
//	delay_ms(200);
	arrive_most_up();	
	delay_ms(300);
	claw_turn0();
	//support_turn120();
	claw_open();
	delay_ms(400);

}

/********************
函数功能 : 爪子从地面把物块放车上位置2
输入参数 : 无
输出参数 ：无
**********************/
void claw_get_block2(void)
{
	claw_turn0();
	claw_open();
	delay_ms(300);
	arrive_block_get();
	delay_ms(300);
	claw_close();
	delay_ms(400);
	arrive_most_up();
	claw_turn2();
	delay_ms(600);
	arrive_car_put();
	claw_open1();
//	delay_ms(200);
	arrive_most_up();
	delay_ms(300);
	claw_turn0();	
	//support_turn120();
	claw_open();
	delay_ms(400);

}

/********************
函数功能 : 爪子从地面把物块放车上位置3
输入参数 : 无
输出参数 ：无
**********************/
void claw_get_block3(void)
{
	claw_turn0();
	claw_open();
	delay_ms(300);
	arrive_block_get();
	delay_ms(300);
	claw_close();
	delay_ms(400);
	arrive_most_up();
	claw_turn3();
	delay_ms(600);
	arrive_car_put();
	claw_open1();
//	delay_ms(200);
	arrive_most_up();	
	delay_ms(300);
	claw_turn0();	
	//support_turn120();
	claw_open();
	delay_ms(400);

}

/********************
函数功能 : 爪子从地面把物块放车上位置4
输入参数 : 无
输出参数 ：无
**********************/
void claw_get_block4(void)
{
	claw_turn0();
	claw_open();
	delay_ms(300);
	arrive_block_get();
	delay_ms(300);
	claw_close();
	delay_ms(400);
	arrive_most_up();
	claw_turn4();
	delay_ms(800);
	arrive_car_put();
	claw_open1();
//	delay_ms(200);
	arrive_most_up();	
	delay_ms(300);
	claw_turn0();	
	//support_turn120();
	claw_open();
	delay_ms(400);

}

/********************
函数功能 : 爪子从地面把物块放车上位置5
输入参数 : 无
输出参数 ：无
**********************/
void claw_get_block5(void)
{
	claw_turn0();
	claw_open();
	delay_ms(300);
	arrive_block_get();
	delay_ms(300);
	claw_close();
	delay_ms(400);
	arrive_most_up();
	claw_turn5();
	delay_ms(1200);
	arrive_car_put();
	claw_open1();
//	delay_ms(200);
	arrive_most_up();	
	delay_ms(300);
	claw_turn0();	
	//support_turn120();
	claw_open();
	delay_ms(400);

}
/********************
函数功能 : 爪子从物料盘把物块放车上 
输入参数 : 无
输出参数 ：无
**********************/
// void claw_get_block1(void)
// {
// 	claw_turn0();
// 	arrive_block_get1();
// 	delay_ms(200);
// 	claw_close();
// 	delay_ms(200);
// 	arrive_most_up();
// 	claw_turn1();
// 	delay_ms(400);
// 	arrive_car_put();
// 	claw_open1();
// 	delay_ms(200);
// 	arrive_most_up();
// 	claw_turn0();
// 	claw_open();
// 	delay_ms(200);
// 	arrive_color_reco();//再次回到识别物料颜色的高度
// 	support_turn120();
	
// }


/********************
函数功能 : 爪子从车上位置1把物块放地上
输入参数 : 无
输出参数 ：无
**********************/
void claw_put_block1(void)
{
    
	arrive_most_up(); 
	delay_ms(200);
	claw_open1();   
	claw_turn1();
	delay_ms(1200);
	arrive_car_get();
//	delay_ms(300);
	claw_close();
	delay_ms(200);
	arrive_most_up(); 
    delay_ms(200);
	claw_turn0();
    delay_ms(500);
	arrive_block_put();
	delay_ms(300);	
	claw_open();
	arrive_circle_capture();
	//support_turn120();
	
}

/********************
函数功能 : 爪子从车上位置2把物块放地上
输入参数 : 无
输出参数 ：无
**********************/
void claw_put_block2(void)
{
    
	arrive_most_up(); 
	delay_ms(200);
	claw_open1();   
	claw_turn2();
	delay_ms(1200);
	arrive_car_get();
//	delay_ms(300);
	claw_close();
	delay_ms(200);
	arrive_most_up(); 
    delay_ms(200);
	claw_turn0();
    delay_ms(500);
	arrive_block_put();
	delay_ms(300);	
	claw_open();
	arrive_circle_capture();
	//support_turn120();
	
}


/********************
函数功能 : 爪子从车上位置3把物块放地上
输入参数 : 无
输出参数 ：无
**********************/
void claw_put_block3(void)
{
    
	arrive_most_up(); 
	delay_ms(200);
	claw_open1();   
	claw_turn3();
	delay_ms(1200);
	arrive_car_get();
//	delay_ms(300);
	claw_close();
	delay_ms(200);
	arrive_most_up(); 
    delay_ms(200);
	claw_turn0();
    delay_ms(500);
	arrive_block_put();
	delay_ms(300);	
	claw_open();
	arrive_circle_capture();
	//support_turn120();
	
}


/********************
函数功能 : 爪子从车上位置4把物块放地上
输入参数 : 无
输出参数 ：无
**********************/
void claw_put_block4(void)
{
    
	arrive_most_up(); 
	delay_ms(200);
	claw_open1();   
	claw_turn4();
	delay_ms(1200);
	arrive_car_get();
//	delay_ms(300);
	claw_close();
	delay_ms(200);
	arrive_most_up(); 
    delay_ms(200);
	claw_turn0();
    delay_ms(500);
	arrive_block_put();
	delay_ms(200);	
	claw_open();
	arrive_circle_capture();
	//support_turn120();
	
}


/********************
函数功能 : 爪子从车上位置5把物块放地上
输入参数 : 无
输出参数 ：无
**********************/
void claw_put_block5(void)
{
    
	arrive_most_up(); 
	delay_ms(200);
	claw_open1();   
	claw_turn5();
	delay_ms(1200);
	arrive_car_get();
//	delay_ms(300);
	claw_close();
	delay_ms(200);
	arrive_most_up(); 
    delay_ms(200);
	claw_turn0();
    delay_ms(500);
	arrive_block_put();
	delay_ms(200);	
	claw_open();
	arrive_circle_capture();
	//support_turn120();
	
}

/********************
函数功能 : 把机械臂调整到靶心识别的动作
输入参数 : 无
输出参数 ：无
**********************/
void claw_task2_reco(void)
{

	//arrive_most_up();
	delay_ms(200);
	claw_turn0();
	delay_ms(200);
	Servo_Stretch();
	delay_ms(300);
	//高度降低到任务二奖杯放置专有打靶高度
	arrive_circle_capture2();



}




/********************
函数功能 : 爪子从车上把冠军放台子上
输入参数 : 无
输出参数 ：无
**********************/
void claw_put_champion(void)
{
    Servo_default();
	arrive_most_up(); 
	delay_ms(200);
	claw_open1();   
	claw_turn1();
	delay_ms(1200);
	arrive_car_get2();
//	delay_ms(300);
	claw_close2();
	delay_ms(200);
	arrive_most_up(); 
	delay_ms(200);
	Servo_Stretch();
    delay_ms(500);
	claw_turn0();
    delay_ms(400);
	arrive_block_put_champion();
	delay_ms(200);	
	claw_open();
	arrive_circle_capture();
	//support_turn120();
	
}

/********************
函数功能 : 爪子从车上把亚军放台子上
输入参数 : 无
输出参数 ：无
**********************/
void claw_put_runnerup(void)
{
    Servo_default();
	arrive_most_up(); 
	delay_ms(200);
	claw_open1();   
	claw_turn2();
	delay_ms(1200);
	arrive_car_get2();
//	delay_ms(300);
	claw_close2();
	delay_ms(200);
	arrive_most_up(); 
	delay_ms(200);
	Servo_Stretch();
    delay_ms(500);
	claw_turn0();
    delay_ms(500);
	arrive_block_put_runnerup();
	delay_ms(200);	
	claw_open();
	arrive_circle_capture();
	//support_turn120();
	
}
/********************
函数功能 : 爪子从车上把季军放台子上
输入参数 : 无
输出参数 ：无
**********************/
void claw_put_num3(void)
{
    Servo_default();
	arrive_most_up(); 
	delay_ms(200);
	claw_open1();   
	claw_turn3();
	delay_ms(1200);
	arrive_car_get2();
//	delay_ms(300);
	claw_close2();
	delay_ms(200);
	arrive_most_up(); 
	 delay_ms(500);
	Servo_Stretch();
    delay_ms(500);
	claw_turn0();
	delay_ms(200);
	arrive_block_put_num3();
	delay_ms(200);	
	claw_open();
	arrive_circle_capture();
	//support_turn120();
	
}


/********************
函数功能 : 爪子把物块从车上放到二层
输入参数 : 无
输出参数 ：无
**********************/
// void claw_put_blockF2(void)
// {

// 	arrive_most_up();
// 	claw_open1();       
// 	claw_turn1();
// 	delay_ms(600);
// 	arrive_car_get();
// 	claw_close();
// 	delay_ms(300);	
// 	arrive_most_up(); 
//     delay_ms(200);
// 	claw_turn0();
// 	delay_ms(300);
// 	arrive_put_down2();
// 	delay_ms(200);	
// 	claw_open();
// 	delay_ms(300);
// 	arrive_most_up();
// 	support_turn120();

// }


/********************
函数功能 : 爪子把物块从车上放到转盘高度
输入参数 : 无
输出参数 ：无
**********************/
// void claw_put_block2(void)
// {

// 	arrive_most_up();
// 	claw_open1();       
// 	claw_turn1();
// 	delay_ms(600);
// 	arrive_car_get();
// 	claw_close();
// 	delay_ms(300);	
// 	arrive_most_up(); 
//     delay_ms(200);
// 	claw_turn0();
// 	delay_ms(300);
// 	arrive_block_get1();//和抓物料的高度一样先
// 	delay_ms(500);	
// 	claw_open();
// 	delay_ms(300);
// 	arrive_most_up();
// 	support_turn120();

// }


///对于决赛物块2的爪子具体动作


/********************
函数功能 : 爪子从地面把奖杯放车上位置1
输入参数 : 无
输出参数 ：无
**********************/
void claw_get2_block1(void)
{
	claw_turn0();
	claw_open();
	delay_ms(300);
	arrive_block2_get();
	delay_ms(300);
	claw_close();
	delay_ms(400);
	arrive_most_up();
	delay_ms(200);
	// Servo_Stretch();
	// delay_ms(200);
	claw_turn1();
	delay_ms(600);
	// Servo_default();
	// delay_ms(200);
	arrive_car2_put();
	claw_open1();
//	delay_ms(200);
	arrive_most_up();	
	delay_ms(300);
	claw_turn0();
	//support_turn120();
	claw_open();


}

/********************
函数功能 : 爪子从地面把奖杯放车上位置2
输入参数 : 无
输出参数 ：无
**********************/
void claw_get2_block2(void)
{
	claw_turn0();
	claw_open();
	delay_ms(300);
	arrive_block2_get();
	delay_ms(300);
	claw_close();
	delay_ms(400);
	arrive_most_up();
	delay_ms(200);
	Servo_Stretch();
	delay_ms(200);
	claw_turn2();
	delay_ms(700);
	Servo_default();
	delay_ms(700);
	arrive_car2_put();
	delay_ms(200);
	claw_open1();
//	delay_ms(200);
	arrive_most_up();	
	delay_ms(300);
	claw_turn0();
	//support_turn120();
	claw_open();



}

/********************
函数功能 : 爪子从地面把奖杯放车上位置3
输入参数 : 无
输出参数 ：无
**********************/
void claw_get2_block3(void)
{
	claw_turn0();
	claw_open();
	delay_ms(300);
	arrive_block2_get();
	delay_ms(300);
	claw_close();
	delay_ms(400);
	arrive_most_up();
	delay_ms(200);
	Servo_Stretch();
	delay_ms(200);
	claw_turn3();
	delay_ms(700);
	Servo_default();
	delay_ms(900);
	arrive_car2_put();
	delay_ms(200);
	claw_open1();
//	delay_ms(200);
	arrive_most_up();	
	delay_ms(300);
	claw_turn0();
	//support_turn120();
	claw_open();


}

/********************
函数功能 : 爪子从物料盘把物块2放车上 
输入参数 : 无
输出参数 ：无
**********************/
// void claw_get2_block1(void)
// {
// 	claw_turn0();
// 	arrive_block2_get1();
// 	delay_ms(200);
// 	claw_close2();
// 	delay_ms(200);
// 	arrive_most_up();
// 	claw_turn1();
// 	delay_ms(800);
// 	arrive_car2_put();
// 	claw_open1();
// 	delay_ms(200);
// 	arrive_most_up();
// 	claw_turn0();
// 	claw_open();
// 	delay_ms(200);
// 	arrive_color_reco();//再次回到识别物料颜色的高度
// 	support_turn120();
	
// }


/********************
函数功能 : 爪子从车上季军位置把奖杯放地上
输入参数 : 无
输出参数 ：无
**********************/
void claw_put2_block(void)
{
    
    Servo_default();
	arrive_most_up(); 
	delay_ms(200);
	claw_open1();   
	claw_turn3();
	delay_ms(1200);
	arrive_car_get2();
//	delay_ms(300);
	claw_close2();
	delay_ms(200);
	arrive_most_up(); 
	delay_ms(500);
	claw_turn0();
	delay_ms(400);
	//arrive_block_put_num3();
	arrive_block2_put();
	delay_ms(200);	
	claw_open();
	arrive_circle_capture();
	
}

/********************
函数功能 : 爪子把物块2从车上放到二层
输入参数 : 无
输出参数 ：无
**********************/
// void claw_put2_blockF2(void)
// {

// 	arrive_most_up();
// 	claw_open1();       
// 	claw_turn1();
// 	delay_ms(600);
// 	arrive_car2_get();
// 	claw_close2();
// 	delay_ms(300);	
// 	arrive_most_up(); 
//     delay_ms(200);
// 	claw_turn0();
// 	delay_ms(300);
// 	arrive_put2_down2();
// 	delay_ms(500);	
// 	claw_open();
// 	delay_ms(300);
// 	arrive_most_up();
// 	support_turn120();

// }


/********************
函数功能 : 爪子把物块2从车上放到转盘高度
输入参数 : 无
输出参数 ：无
**********************/
// void claw_put2_block2(void)
// {

// 	arrive_most_up();
// 	claw_open1();       
// 	claw_turn1();
// 	delay_ms(600);
// 	arrive_car2_get();
// 	claw_close2();
// 	delay_ms(300);	
// 	arrive_most_up(); 
//     delay_ms(200);
// 	claw_turn0();
// 	delay_ms(300);
// 	arrive_block2_get1();//和抓物料的高度一样先
// 	delay_ms(500);	
// 	claw_open();
// 	delay_ms(300);
// 	arrive_most_up();
// 	support_turn120();

// }










/********************
函数功能 : 爪子归位
输入参数 : 无
输出参数 ：无
**********************/
void claw_home(void)
{
	arrive_most_up();
	Servo_default();
	delay_ms(200);
	claw_open();
	delay_ms(300);  
	claw_turn3();
}

