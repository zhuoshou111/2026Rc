#include "system_init.h"


/*********************************************


瓜兵出击队 2025/9/12

2025  Robcup中国机器人大赛 

河北石家庄  2025/10/17

车型智能搬运
总决赛



*********************************************/

void test(void); //测试函数
//也就是相对于初始Z轴0度的偏角
int16_t base_angle = 0; //基准角度,根据上位机要求角度跟随用，比如上位机调用转90度，基准角度会加90度，反方向会减去90度

//获得扫码信息，全局变量
int16_t code1 =0; 
int16_t code2 =0; 	



int main(void)
{		
	system_Init();
	contral_motor_Init();
	claw_Init();
	delay_ms(300);
	//arrive_most_up(); //到达最高度
	ResetAng_Z(); 
	// 初始化动作

	
	arrive_most_up();
	delay_ms(300);
	claw_turn3();
	delay_ms(200);
	claw_open();

			int angle = 90;	
			stepPosition=0;		
			MOTOR_Angle(90);
			while(1)
			{
				if(stepPosition == angle_temp)
				{
					break;
				}
			}	


			

            delay_ms(100);

			//陀螺仪微调操作，不清零Z轴陀螺仪，使用基准角度跟随上位机调用的角度变化，微调即为让其转正
			//是上位机调用Motor_Align()函数来实现
			//MOTOR_Align();

			MOTOR_Align();


	while(1)              //主代码
	{


		// I2C_Read_Sensor(color_value);
		// color = getClosestColor(color_value);


		OLED_ClearArea(0, 48, 128, 16); //清除OLED屏幕
		OLED_Printf(0,0,OLED_8X16,"Angle=%.3f",global_angle);
		OLED_Printf(0,16,OLED_8X16,"BaseAngle=");
		OLED_ShowSignedNum(64,16,base_angle,3,OLED_8X16);
		//	OLED_Printf(0,32,OLED_8X16,"Err_Angle=%.3f",global_angle-base_angle);

		OLED_Printf(0,48,OLED_8X16,"%d %d %d %d",color_value[0], color_value[1], color_value[2],color);
		
		// delay_ms(1000);
		OLED_Update();

	

		if(Key_Get() == 1)//一键启动，如果按下，给工控机发送启动指令，同时令目标角度和串口屏显示为0
		{
			base_angle = 0;
			arrive_most_up();
			delay_ms(200);
			Servo_default();
			delay_ms(200);
			claw_turn0();
			delay_ms(400);
			arrive_most_down(); //到达最低高度

			//test();

			// arrive_camera();//最低高度
			// delay_ms(2000);
			// arrive_color_reco();//物料颜色识别高度
			// delay_ms(2000);
			// arrive_block_get1();//物料转盘上抓物块的高度
			// delay_ms(2000);
            
			delay_ms(500);
			UART_SendPacket2UP(0x02);
			UART_SendPacket2UP(0x02);
			UART_SendPacket2UP(0x02);
			UART_SendPacket2UP(0x02);	
		}  
     
		if(Serial1_GetRxFlag() == 1)//接收树莓派消息
		{
			uart_handle();
			UART_SendPacket2UP(0x01);
		}

		// if(Serial5_GetRxFlag() == 1)//如果接收到了扫码发来的数据就处理
		// {

		// 	delay_ms(300);//等待数据接收完全
		// 	UART5_ParseCode(UART5_RX_BUF,&code1,&code2);//解析出二维码数据，此时UART5_BUX中依然存放的是二维码数据	
		// 	u2_printf("tt3.txt=\"%d+%d\"",code1,code2);//多次发送给串口屏
		// 	delay_ms(10);
		// 	u2_printf("tt3.txt=\"%d+%d\"",code1,code2); 
		// 	delay_ms(10);
		// 	u2_printf("t3.txt=\"%d+%d\"",code1,code2);
		// 	delay_ms(10);
		// 	u2_printf("t3.txt=\"%d+%d\"",code1,code2);
		// 	delay_ms(10);
		// 	u2_printf("tt3.txt=\"%d+%d\"",code1,code2);
		// 	delay_ms(10);			

		// }
		
	}	 
	

} 


void test(void) //测试程序
{




	claw_get_block1();
	delay_ms(2000);
	claw_get_block2();
	delay_ms(2000);
	claw_get_block3();
	delay_ms(2000);
	claw_get_block4();
	delay_ms(2000);
	claw_get_block5();
	delay_ms(2000);
	claw_put_block1();
	delay_ms(2000);
	claw_put_block2();
	delay_ms(2000);
	claw_put_block3();
	delay_ms(2000);
	claw_put_block4();
	delay_ms(2000);
	claw_put_block5();
	delay_ms(2000);


	claw_get2_block1();
	delay_ms(2000);
	claw_get2_block2();
	delay_ms(2000);
	claw_get2_block3();
	delay_ms(2000);
	claw_put_champion();
	delay_ms(2000);
	claw_put_runnerup();
	delay_ms(2000);
	claw_put_num3();
	delay_ms(2000);
	claw_put2_block();
	delay_ms(2000);
	arrive_most_up();
	delay_ms(200);
	Servo_default();
	delay_ms(2000);

	// claw_get_block1();
	// delay_ms(2000);
	// claw_get_block2();
	// delay_ms(2000);
	// claw_get_block3();
	// delay_ms(2000);
	// claw_get_block4();
	// delay_ms(2000);
	// claw_get_block5();
	// delay_ms(2000);
	// claw_put_block1();
	// delay_ms(2000);
	// claw_put_block2();
	// delay_ms(2000);
	// claw_put_block3();
	// delay_ms(2000);
	// claw_put_block4();
	// delay_ms(2000);
	// claw_put_block5();



}
