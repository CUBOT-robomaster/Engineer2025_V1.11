#include "auto.h"
#include "gold.h"
#include "hardware_config.h"
#include "servo.h"
#include "dr16.h"
#include "joint_control.h"
#include "air.h"
#include "vt13.h"
#include "A1_stm32.h"
#include "inverse.h"
#include "controllerl.h"
int8_t   auto_flag=0;//进入自动阶段标志位，防止再去执行其他自动阶段
int8_t   move_back_flag=0;//需要把后退发给底盘的标志
int16_t  get_single_silver_count=0;//取单银矿时间戳
int8_t   get_single_silver_flag=0;//取单银矿标志位
int16_t  get_double_silver_count=0;//取双银矿时间戳
int8_t   get_double_silver_flag=0;//取双银矿标志位
int16_t  get_double_gold_count=0;//取双金矿时间戳
int8_t   get_double_gold_flag=0;//取双金矿标志位
int16_t  get_single_gold_count=0;//取单金矿时间戳
int8_t   get_single_gold_flag=0;//取单金矿标志位
int16_t  arm_get_single_gold_count=0;//机械臂取单金矿时间戳
int8_t   arm_get_single_gold_flag=0;//机械臂取单金矿标志位

int16_t  move_feed_count=0;//兑矿前置时间戳
int8_t   move_feed_right_flag=0;//兑矿前置标志位
int8_t   move_feed_left_flag=0;//兑矿前置标志位


int16_t  associate_stretch_count=0;//交接时间戳
int8_t   associate_stretch_left_flag=0;//交接左侧取矿机构标志位
int8_t   associate_stretch_right_flag=0;//交接右侧取矿机构标志位
int8_t associate_arm_left_flag=0;//机械臂交接左侧取矿机构过度标志位

uint8_t associate_stretch_auto_flag=0;//自动判断交接标志
uint8_t get_gold_flag=0;//判断取得是金矿
uint8_t get_silver_flag=0;//判断取得是银矿
uint8_t gold_left_flag=0;//判断是否已经交接左侧金矿

uint8_t get_double_gold_manual_flag=0;//判断是否在手动取矿

void get_single_silver(void);//机械臂取单银矿
void get_double_silver(void);//机械臂和右侧机构取双银矿
void get_double_gold(void);//左右侧机构取双金矿


/**
* @brief 右侧取单银矿
  */ 
//自动阶段开始前需要对位，所以舵机初始位置应往回收
void get_single_silver_stretch(void)
{    
	 get_single_silver_count++;//最大溢出时间65535
	if(get_single_silver_count==10)
	{
		 get_silver_flag=1;
	}
     if(get_single_silver_count>100&&get_single_silver_count<1100)//先升起
		Auto_right_up_goldmining(&Goldmining_t,-12770,15);
	 if(get_single_silver_count==500)
		position_right=2820;
	 if(get_single_silver_count==1200)//调制舵机角度
	    position_right=2640;
	 if(get_single_silver_count>1200&&get_single_silver_count<2500)//伸出
		Auto_right_out_goldmining(&Goldmining_t,27235,20);
	 if(get_single_silver_count==2000) //开气泵，右侧常开不需要开电磁阀
		Vacuum_pump_main(open);
	 if(get_single_silver_count>2800&&get_single_silver_count<3500)//抬升降下去
		Auto_right_up_goldmining(&Goldmining_t,-4528,15);
	 if(get_single_silver_count>4000&&get_single_silver_count<5800)//吸到矿石后抬起来
		Auto_right_up_goldmining(&Goldmining_t,-37000,20);
	 //伸出收回，舵机往上，底盘后退（需要测试三者的先后处理关系，舵机速度和加速度可能要调）
	  if(get_single_silver_count==6300)
		move_back_flag=1;
	  if(get_single_silver_count==7300)
		move_back_flag=0;
	  if(get_single_silver_count>6100&&get_single_silver_count<7500)//伸出
		Auto_right_out_goldmining(&Goldmining_t,0,20);
	  if(get_single_silver_count==5500)//调制舵机角度
	    position_right=550;
	 if(get_single_silver_count>6800&&get_single_silver_count<8800)//抬升降下来
		Auto_right_up_goldmining(&Goldmining_t,-1000,20);
	 if(get_single_silver_count>8900)//最后结束置0
	 {
		 //Vacuum_pump_main(close);
		 auto_flag=0;
		 get_single_silver_flag=0;
		 get_single_silver_count=0;
	 }
	 
}
/**
* @brief 机械臂取单银矿
  */ 
void get_single_silver(void)
{
   get_single_silver_count++;
   if(get_single_silver_count==10)
   {
	  arm_relese(close);
      Manipulator.pitch1_deg.angle_target=-93;
	  Manipulator.pitch2_deg.angle_target=34;
   }
   if(get_single_silver_count==500)
    Manipulator.pitch3_deg.angle_target=-32;
   if(get_single_silver_count==2500)
	   Vacuum_pump_vice(open);

   if(get_single_silver_count==3500)
   {
	  auto_flag=0;
      Manipulator.pitch1_deg.angle_target=-70.23;
	  Manipulator.pitch2_deg.angle_target=37;
	   Manipulator.pitch3_deg.Accel=0.0002;//拉高加速度，提高关节速度
	  Manipulator.pitch3_deg.angle_target=-30; 
   }   
      if(get_single_silver_count==4000)
   {
      Manipulator.pitch1_deg.angle_target=-48.46;
	  Manipulator.pitch2_deg.angle_target=57;
	  Manipulator.pitch3_deg.angle_target=-68;
   }  
//    if(get_single_silver_count==6000)
//		move_back_flag=1;
//	if(get_single_silver_count==7500)
//		move_back_flag=0;
		
      if(get_single_silver_count==4500)
   {
      Manipulator.pitch1_deg.angle_target=-48.46;
	  Manipulator.pitch2_deg.angle_target=57;
	  Manipulator.pitch3_deg.angle_target=-12;
   }
  
	   
         if(get_single_silver_count==5700)
   {
      Manipulator.pitch1_deg.angle_target=-30;
	  Manipulator.pitch2_deg.angle_target=30;
	   Manipulator.pitch3_deg.angle_target=-16;
   }   
   if(get_single_silver_count>6200)
   {
	  //Vacuum_pump_vice(0);
	  get_single_silver_flag=0;
	  get_single_silver_count=0;
   }
   
}

/**
* @brief 取双银矿
  */ 
void get_double_silver(void)
{
	
	/*机械臂*/
    get_double_silver_count++;
   if(get_double_silver_count==100)
   {
	  arm_relese(close);
      Manipulator.pitch1_deg.angle_target=-93;
	  Manipulator.pitch2_deg.angle_target=34;
	  get_silver_flag=1;
   }
   
   if(get_double_silver_count==500)
    Manipulator.pitch3_deg.angle_target=-32;
   
   if(get_double_silver_count==3000)
	   Vacuum_pump_vice(open);//开泵

   if(get_double_silver_count==3800)
   {
      Manipulator.pitch1_deg.angle_target=-70.23;
	  Manipulator.pitch2_deg.angle_target=37;
	  Manipulator.pitch3_deg.Accel=0.0002;//拉高加速度，提高关节速度
	  Manipulator.pitch3_deg.angle_target=-34; 
   }   
      if(get_double_silver_count==4200)
   {
      Manipulator.pitch1_deg.angle_target=-61.81;
	  Manipulator.pitch2_deg.angle_target=57;
	  Manipulator.pitch3_deg.angle_target=-72;
   }  
		
      if(get_double_silver_count==5700)
   {
      Manipulator.pitch1_deg.angle_target=-48.46;
	  Manipulator.pitch2_deg.angle_target=58;
	  Manipulator.pitch3_deg.angle_target=-14;
   }
      if(get_double_silver_count==6400)
   {
      Manipulator.pitch1_deg.angle_target=0;
	  Manipulator.pitch2_deg.angle_target=0;
	   Manipulator.pitch3_deg.angle_target=-8;
   }   
   
   /*取矿机构*/
   if(get_double_silver_count>10&&get_double_silver_count<900)//先升起
		Auto_right_up_goldmining(&Goldmining_t,-12770,18);
   if(get_double_silver_count==500)
		position_right=2820;
   if(get_double_silver_count==1000)//调制舵机角度
	    position_right=2640;
   if(get_double_silver_count>1000&&get_double_silver_count<2300)//伸出
		Auto_right_out_goldmining(&Goldmining_t,28235,25);
   if(get_double_silver_count==1800) //开气泵，右侧常开不需要开电磁阀
		Vacuum_pump_main(open);
   if(get_double_silver_count>2300&&get_double_silver_count<2900)//抬升降下去
		Auto_right_up_goldmining(&Goldmining_t,-5028,20);
   if(get_double_silver_count>3200&&get_double_silver_count<4600)//吸到矿石后抬起来
		Auto_right_up_goldmining(&Goldmining_t,-38000,20);
   if(get_double_silver_count==4701)//调制舵机角度
	    position_right=2300;
   if(get_double_silver_count>5700&&get_double_silver_count<7500)//伸出
		Auto_right_out_goldmining(&Goldmining_t,500,20);
   if(get_double_silver_count==5100)//调制舵机角度
	    position_right=550;
   if(get_double_silver_count>5700&&get_double_silver_count<7700)//抬升降下来
		Auto_right_up_goldmining(&Goldmining_t,-1000,22);
   
   
    if(get_double_silver_count==5001)
		move_back_flag=1;
	if(get_double_silver_count==6100)
		move_back_flag=0;
	
    if(get_double_silver_count>8000)
    {
	   // Vacuum_pump_vice(0);
	   auto_flag=0;
	   get_double_silver_flag=0;
	   get_double_silver_count=0;
    }
   
}


/**
* @brief 取双金矿
  */ 
void get_double_gold(void)
{
    get_double_gold_count++;
	if(get_double_gold_count==2)
	{
		get_gold_flag = 1;
		gold_left_flag = 1;
	}
    if(get_double_gold_count>0&&get_double_gold_count<1500)//抬升
    {   Auto_right_up_goldmining(&Goldmining_t,-23025,20);
        Auto_left_up_goldmining(&Goldmining_t,25233,20);}
    if(get_double_gold_count>3000&&get_double_gold_count<3100)//开气泵
    {
        Vacuum_pump_main(open);
        valve_fetch(open);
    }
    if(get_double_gold_count>1200&&get_double_gold_count<2000)//调整吸盘
    {
		acceleration1=40;
		acceleration2=40;
		velocity1=55;
        velocity2=55;
        position_right=1600;
        position_left=620;
    }
    if(get_double_gold_count>2400&&get_double_gold_count<2900)//下降一点
    {    Auto_right_up_goldmining(&Goldmining_t,-18065,20);
        Auto_left_up_goldmining(&Goldmining_t,17053,20);
    }
    if(get_double_gold_count>2500&&get_double_gold_count<5000)//伸出
    {
        Auto_right_out_goldmining(&Goldmining_t,42360,28);
        Auto_left_out_goldmining(&Goldmining_t,-26310,28);
    }
    if(get_double_gold_count>5500&&get_double_gold_count<6500)//带矿抬升
    {
        Auto_right_up_goldmining(&Goldmining_t,-27965,17);
        Auto_left_up_goldmining(&Goldmining_t,27568,17);
    }
    if(get_double_gold_count>7000&&get_double_gold_count<9500)
    {
        Auto_right_out_goldmining(&Goldmining_t,0,28);//带矿收回
        Auto_left_out_goldmining(&Goldmining_t,0,28);
    }
	if(get_double_gold_count==7700)
	{move_back_flag=1;}
	if(get_double_gold_count==9700)
	{move_back_flag=0;}
	if(get_double_gold_count==10000)
	{ 
		acceleration1=15;
		acceleration2=15;
		velocity1=30;
        velocity2=30;
		position_right=500;
        position_left=280;
	}
	if(get_double_gold_count>10000&&get_double_gold_count<11500)
	{
		Auto_right_up_goldmining(&Goldmining_t,-500,20);
		Auto_left_up_goldmining(&Goldmining_t,500,20);
	}
	
    if(get_double_gold_count>12000)
    {
        auto_flag=0;
        get_double_gold_count=0;
        get_double_gold_flag=0;
        //Vacuum_pump_main(close);
		// valve_fetch(close);
    }
 }

 void get_double_gold_manual(void)
{
    get_double_gold_count++;
	if(get_double_gold_count<10)
	{
		get_gold_flag = 1;
		gold_left_flag = 1;
	}
    if(get_double_gold_count>0&&get_double_gold_count<1500)//抬升
    {   Auto_right_up_goldmining(&Goldmining_t,-23025,20);
        Auto_left_up_goldmining(&Goldmining_t,25233,20);}
    if(get_double_gold_count>3000&&get_double_gold_count<3100)//开气泵
    {
        Vacuum_pump_main(open);
        valve_fetch(open);
    }
    if(get_double_gold_count>1200&&get_double_gold_count<2000)//调整吸盘
    {
		acceleration1=40;
		acceleration2=40;
		velocity1=55;
        velocity2=55;
        position_right=1600;
        position_left=620;
    }
    if(get_double_gold_count>2400&&get_double_gold_count<2900)//下降一点
    {    Auto_right_up_goldmining(&Goldmining_t,-18065,20);
        Auto_left_up_goldmining(&Goldmining_t,16053,20);
    }
    if(get_double_gold_count>2500&&get_double_gold_count<4500)//伸出
    {
        Auto_right_out_goldmining(&Goldmining_t,42860,28);
        Auto_left_out_goldmining(&Goldmining_t,-26010,20);
    }
	if(get_double_gold_count>4500&&get_double_gold_count<5500)//带矿抬升
    {
        Auto_right_up_goldmining(&Goldmining_t,-27265,17);
        Auto_left_up_goldmining(&Goldmining_t,26068,17);
    }
	if(get_double_gold_count==5550)
    {
	  get_double_gold_manual_flag = 1;
       get_double_gold_flag = 0;
		auto_flag = 0;
    }
    if(get_double_gold_count==6100)
    {
       get_double_gold_manual_flag = 0;
	  //move_back_flag=1;
    }
    if(get_double_gold_count>6200&&get_double_gold_count<8000)
    {
        Auto_right_out_goldmining(&Goldmining_t,800,25);//带矿收回
        Auto_left_out_goldmining(&Goldmining_t,0,25);
    }
	if(get_double_gold_count==8000)
		move_back_flag=1;
	if(get_double_gold_count==9000)
	{ 
		acceleration1=15;
		acceleration2=15;
		velocity1=30;
        velocity2=30;
		position_right=500;
        position_left=280;
		move_back_flag=0;

	}
	if(get_double_gold_count>9000&&get_double_gold_count<10500)
	{
		Auto_right_up_goldmining(&Goldmining_t,-800,20);
		Auto_left_up_goldmining(&Goldmining_t,800,20);
	}
	
    if(get_double_gold_count>11000)
    {
        auto_flag=0;
        get_double_gold_count=0;
        get_double_gold_flag=0;
        //Vacuum_pump_main(close);
		// valve_fetch(close);
    }
 }

/**
* @brief 右侧取单金矿
  */ 	
void get_single_gold(void)
{
    get_single_gold_count++;
	if(get_single_gold_count==20)
	{
		get_gold_flag = 1;
	}
    if(get_single_gold_count>100&&get_single_gold_count<2000)//抬升
    {   Auto_right_up_goldmining(&Goldmining_t,-24560,20);}
    if(get_single_gold_count>3000&&get_single_gold_count<3100)//开气泵
    {
        Vacuum_pump_main(open);
    }
    if(get_single_gold_count>1500&&get_single_gold_count<3000)//调整吸盘
    {
        position_right=1600;
    }
    if(get_single_gold_count>3000&&get_single_gold_count<4000)//下降一点
    {    Auto_right_up_goldmining(&Goldmining_t,-18065,15);
    }
    if(get_single_gold_count>3500&&get_single_gold_count<5000)//伸出
    {
        Auto_right_out_goldmining(&Goldmining_t,26455,25);
    }
	 if(get_single_gold_count==5010)//伸出
	 {
		 get_single_gold_flag =0;
		get_double_gold_manual_flag = 1;
		 auto_flag = 0;
	 }
//	if(get_single_gold_count>5200&&get_single_gold_count<6000)//收回一点
//    {
//        Auto_right_out_goldmining(&Goldmining_t,24455,15);
//    }
//    if(get_single_gold_count>6500&&get_single_gold_count<7500)//带矿抬升
//    {
//        Auto_right_up_goldmining(&Goldmining_t,-24640,15);
//        
//    }
    
    if(get_single_gold_count>5500&&get_single_gold_count<6800)
    {
        Auto_right_out_goldmining(&Goldmining_t,5,25);//带矿收回
    }
//	if(get_single_gold_count==8500)
//		move_back_flag=1;
//	if(get_single_gold_count==9500)
//		move_back_flag=0;
    if(get_single_gold_count==6500)
    {
        position_right=500;
    }
    if(get_single_gold_count>7000&&get_single_gold_count<8500)
    {  
        Auto_right_up_goldmining(&Goldmining_t,-835,20);
    }
    if(get_single_gold_count>9000)//最后结束置0
     {
		 get_double_gold_manual_flag = 0; 
         auto_flag=0;
         get_single_gold_flag=0;
         get_single_gold_count=0;
         //Vacuum_pump_main(close);
     }
    
    
 }

 
///**
//* @brief 交接右侧矿石
//*/ 
// void associate_stretch_right()
//{
//	associate_stretch_count++;
//	if(associate_stretch_count == 10)
//	{
//		//auto_flag=1;
//		arm_relese(close);
//		Vacuum_pump_vice(open);
//		Manipulator.pitch1_deg.angle_target=-28;
//		Manipulator.pitch2_deg.angle_target=30;
//	}
//	if(associate_stretch_count>300&&associate_stretch_count<1300)//下降一点
//    {    Auto_right_up_goldmining(&Goldmining_t,-8000,10);
//    }
//	 if(associate_stretch_count == 200)
//	{
//		roll2_feed = 0;
//		if(get_gold_flag == 1)
//		{
//			Manipulator.roll2_deg.angle_target=-85;
//			get_gold_flag = 0;
//		}
//		else
//			Manipulator.roll2_deg.angle_target=0;
//		Manipulator.yaw_deg.angle_target=15;
//		Manipulator.roll1_deg.angle_target=-92;
//	}
//	if(associate_stretch_count == 300)
//	{
//		Manipulator.pitch3_deg.angle_target=-105;
//		Manipulator.pitch3_deg.Accel=0.0005;//拉高加速度，提高关节速度
//	}
//	if(associate_stretch_count == 1600)
//	{
//		if( inver_start==0 )//是否在自动兑矿，否则退出自动模式
//			auto_flag=0;
//		Manipulator.pitch3_deg.min_velocity=0.09;//拉高速度，提高响应
//		Manipulator.yaw_deg.angle_target=-34;
//		//Manipulator.pitch3_deg.angle_target=-56;
//	}
//	if(associate_stretch_count >= 1600&&associate_stretch_count<3600)
//		Manipulator.pitch3_deg.angle_target = -93-(Manipulator.Dm_8006_yaw.pos*RtA - Manipulator.yaw_deg.angle_init);//将pitch3的目标角度与yaw联合，保证水平
//	if(associate_stretch_count == 3300)
//	{	
//		Vacuum_pump_main(close);
//	}
//	if(associate_stretch_count>3500&&associate_stretch_count<4300)//下降一点
//    {  
//		Auto_right_up_goldmining(&Goldmining_t,-900,14);
//    }
//	if(associate_stretch_count == 4500)
//	{
//		Manipulator.pitch1_deg.angle_target=-50;
//		Manipulator.pitch2_deg.angle_target=65;
//		
//	}
//	if(associate_stretch_count == 5900)
//	{
//		Manipulator.yaw_deg.angle_target=0;
//	}
//		if(associate_stretch_count == 5700)
//	{
//		Manipulator.pitch3_deg.angle_target=-15;
//		Manipulator.roll1_deg.angle_target=0;
//		//position_right=2820;
//	}
//		if(associate_stretch_count == 5500)
//	{
//		Manipulator.pitch1_deg.angle_target=-48;
//		Manipulator.pitch2_deg.angle_target=50;
//		
//	}
//	 if(associate_stretch_count>6000)//最后结束置0
//     {
//         associate_stretch_right_flag=0;
//         associate_stretch_count=0;
//         //Vacuum_pump_main(close);
//     }
//}

float target_p[3] = {-498,0,440},target_deg[3] = {0,0,180};
/**
* @brief 交接右侧矿石(使用逆解)
*/ 
 void associate_stretch_right()
{
	associate_stretch_count++;
	if(associate_stretch_count == 10)
	{
		//auto_flag=1;
		arm_relese(close);
		Vacuum_pump_vice(open);
		target_deg[0] = 0;
		target_deg[1] = 0;
		target_deg[2] = 93;
		target_p[0] = -140;
		target_p[1] = 82;
		target_p[2] = 300;
		
	}
	if(associate_stretch_count>300&&associate_stretch_count<1300)
		Auto_right_up_goldmining(&Goldmining_t,-8000,10);
	if(associate_stretch_count>200&&associate_stretch_count<300)
    {    
		vision_coil_test(target_deg,target_p);
		if(get_gold_flag == 1)
			Manipulator.roll2_deg.angle_target -= 88;
		else{}
    }
	
	if(associate_stretch_count>1800&&associate_stretch_count<=2800)
	{
		if(get_gold_flag == 1)
			target_p[1] += 0.15;
		else
			target_p[1] += 0.12;
		vision_coil_test(target_deg,target_p);
		if(get_gold_flag == 1)
		{
			Manipulator.roll2_deg.angle_target -= 88;
		}
		else{}
		Manipulator.pitch3_deg.angle_target -= 3;		
	}
	 if(associate_stretch_count == 3000)
	{
		if(get_gold_flag == 1)
		{
			get_gold_flag = 0;
		}
	}
	if(associate_stretch_count == 1800)
	{
		if( inver_start==0 )//是否在自动兑矿，否则退出自动模式
			auto_flag=0;
	}
	if(associate_stretch_count == 2800)
	{	
		Vacuum_pump_main(close);
	}
	if(associate_stretch_count>3100&&associate_stretch_count<3600)//下降一点
    {  
		Auto_right_up_goldmining(&Goldmining_t,-900,16);
    }
	if(associate_stretch_count == 3600)
	{
		Manipulator.pitch1_deg.angle_target=-50;
		Manipulator.pitch2_deg.angle_target=65;
		
	}
	if(associate_stretch_count == 4200)
	{
		Manipulator.yaw_deg.angle_target=0;
		Manipulator.yaw_deg.Accel=0.0002;
	}
		if(associate_stretch_count == 4000)
	{
		Manipulator.pitch3_deg.angle_target=-15;
		Manipulator.roll1_deg.angle_target=0;
		Manipulator.roll2_deg.angle_target = 0;
		//position_right=2820;
	}
	 if(associate_stretch_count>4500)//最后结束置0
     {
         associate_stretch_right_flag=0;
         associate_stretch_count=0;
		 Manipulator.yaw_deg.Accel=0.00012;
         //Vacuum_pump_main(close);
     }
}





//void associate_stretch_left()
//{
//	associate_stretch_count++;
//	if(associate_stretch_count == 10)
//	{
//		arm_relese(close);
//		Vacuum_pump_vice(open);
//		Manipulator.pitch1_deg.angle_target=-40;
//		Manipulator.pitch2_deg.angle_target=30;
//		Manipulator.roll2_deg.angle_target=-93;
//	}
//	if(associate_stretch_count>300&&associate_stretch_count<1300)
//    {    Auto_left_up_goldmining(&Goldmining_t,8000,10);
//    }
//	if(associate_stretch_count == 300)
//	{
//		Manipulator.pitch3_deg.Accel = 0.0005;//拉高加速度，提高关节速度
//		Manipulator.pitch3_deg.angle_target=-110;
//	}
//    if(associate_stretch_count == 300)
//	{
//		Manipulator.yaw_deg.angle_target=-11;
//		Manipulator.roll1_deg.angle_target=86;
//	}
//	if(associate_stretch_count == 1500)
//	{
//		Manipulator.yaw_deg.angle_target=0;
//	}
//	if(associate_stretch_count >= 1600&&associate_stretch_count <= 3500)
//	{
//		Manipulator.pitch3_deg.Accel=0.0001;//拉低加速度，满足pitch3与yaw的运动速度保证吸盘角度
//		Manipulator.pitch3_deg.min_velocity=0.09;//拉高速度，提高响应
//		Manipulator.pitch3_deg.angle_target=-103+(Manipulator.Dm_8006_yaw.pos*RtA - Manipulator.yaw_deg.angle_init);//将pitch3的目标角度与yaw联合，保证水平;//满足pitch3与yaw的运动速度保证吸盘角度

//	}
//	if(associate_stretch_count == 2000)
//	{
//		auto_flag=0;
//		//Manipulator.pitch3_deg.min_velocity=0.02;//拉低速度，满足pitch3与yaw的运动速度保证吸盘角度
//		Manipulator.yaw_deg.angle_target=45;//满足pitch3与yaw的运动速度保证吸盘角度
//	}
//	if(associate_stretch_count == 3000)
//	{	
//		valve_fetch(close);
//	}
//	if(associate_stretch_count>3500&&associate_stretch_count<4300)//下降一点
//    {  
//		Auto_left_up_goldmining(&Goldmining_t,900,14);
//    }
//	if(associate_stretch_count == 4500)
//	{
//		Manipulator.pitch1_deg.angle_target=-54;
//		Manipulator.pitch2_deg.angle_target=65;
//		
//	}
//	if(associate_stretch_count == 5500)
//	{
//		Manipulator.yaw_deg.angle_target=0;
//	}
//		if(associate_stretch_count == 6200)
//	{

//		Manipulator.pitch3_deg.angle_target=-15;
//		Manipulator.roll1_deg.angle_target=0;
//		Manipulator.roll2_deg.angle_target=0;
//		//position_left = 1000;
//	}
//		if(associate_stretch_count == 7200)
//	{
//		Manipulator.pitch1_deg.angle_target=-20;
//		Manipulator.pitch2_deg.angle_target=30;
//	}
//	 if(associate_stretch_count>8000)//最后结束置0
//     {
//         auto_flag=0;
//         associate_stretch_left_flag=0;
//         associate_stretch_count=0;
//         //Vacuum_pump_main(close);
//     }
//	
//}


/**
* @brief 交接左侧矿石(使用逆解)
*/ 
void associate_stretch_left()
{
	associate_stretch_count++;
	if(associate_stretch_count == 10)
	{
		arm_relese(close);
		Vacuum_pump_vice(open);
		target_deg[0] = 0;
		target_deg[1] = -18;
		target_deg[2] = -94;
		
		target_p[0] = -180;
		target_p[1] = -82;
		target_p[2] = 305;
	}
	if(associate_stretch_count>300&&associate_stretch_count<1300)
		Auto_left_up_goldmining(&Goldmining_t,8000,10);
	if(associate_stretch_count>300&&associate_stretch_count<400)
    {    
		vision_coil_test(target_deg,target_p);
		Manipulator.roll2_deg.angle_target -= 90;
    }
	if(associate_stretch_count == 300)
	{
		//Manipulator.pitch3_deg.Accel = 0.0005;//拉高加速度，提高关节速度
		//Manipulator.pitch3_deg.min_velocity=0.09;//拉高速度，提高响应
	}

	if(associate_stretch_count >= 1600&&associate_stretch_count <= 3100)
	{
		target_p[0] += 0.035;//sin(18)*117
		target_p[1] -= 0.111;//cos(18)*117
		vision_coil_test(target_deg,target_p);
		Manipulator.roll2_deg.angle_target -= 90;
	}
	if(associate_stretch_count == 2000)
	{
		auto_flag=0;
	}
	if(associate_stretch_count == 3200)
	{	
		valve_fetch(close);
	}
	if(associate_stretch_count>3500&&associate_stretch_count<4300)//下降一点
    {  
		Auto_left_up_goldmining(&Goldmining_t,900,14);
    }
	if(associate_stretch_count == 4500)
	{
		Manipulator.pitch1_deg.angle_target=-54;
		Manipulator.pitch2_deg.angle_target=65;
		
	}
	if(associate_stretch_count == 5500)
	{
		Manipulator.yaw_deg.angle_target=0;
	}
		if(associate_stretch_count == 6200)
	{

		Manipulator.pitch3_deg.angle_target=-15;
		Manipulator.roll1_deg.angle_target=0;
		Manipulator.roll2_deg.angle_target=0;
		//position_left = 1000;
	}
		if(associate_stretch_count == 7200)
	{
		Manipulator.pitch1_deg.angle_target=-20;
		Manipulator.pitch2_deg.angle_target=30;
	}
	 if(associate_stretch_count>8000)//最后结束置0
     {
         auto_flag=0;
		 gold_left_flag = 0;
         associate_stretch_left_flag=0;
         associate_stretch_count=0;
         //Vacuum_pump_main(close);
     }

}


void associate_stretch_auto()//一个键位区分取哪边矿
{
	if(get_gold_flag)
	{
		if(gold_left_flag)
		{
			associate_stretch_left_flag = 1;
			gold_left_flag = 0;
		}
		else
		{
			associate_stretch_right_flag =1;
		}
	}
	else 
	{
		associate_stretch_right_flag = 1;
		get_silver_flag =0;
	}
	associate_stretch_auto_flag = 0;

	
}

//void associate_arm_left()//机械臂将矿石交接到取矿机构切换吸盘面
//{
//	associate_stretch_count++;
//	if(associate_stretch_count == 1)
//	{
//		position_left=260;
//		Vacuum_pump_main(open);
//		Manipulator.pitch1_deg.angle_target = -57;
//		Manipulator.pitch2_deg.angle_target = 86;
//		Manipulator.pitch3_deg.angle_target = -110;
//		Manipulator.roll2_deg.angle_target = 28;
//	}
//	if(associate_stretch_count == 1000)
//		Manipulator.yaw_deg.angle_target = 54;
//	if(associate_stretch_count>1600&&associate_stretch_count<2500)
//		Auto_left_up_goldmining(&Goldmining_t,8000,12);
//	if(associate_stretch_count == 2500)
//	{
//		valve_fetch(open);
//		Manipulator.pitch1_deg.angle_target = -50;
//		Manipulator.pitch2_deg.angle_target = 62;
//		Manipulator.pitch3_deg.angle_target = -99;
//	}
//	if(associate_stretch_count == 3500)
//	{
//		Vacuum_pump_vice(close);
//		arm_relese(open);
//	}
//	if(associate_stretch_count == 3500)
//	{
//		auto_flag = 0;
//		Manipulator.pitch1_deg.angle_target = -52;
//		Manipulator.pitch2_deg.angle_target = 88;
//		Manipulator.pitch3_deg.angle_target = -110;
//		Manipulator.yaw_deg.angle_target = 0;
//		Manipulator.roll2_deg.angle_target = 0;
//	}
//	if(associate_stretch_count == 4500)
//	{
//		arm_relese(close);
//		Manipulator.pitch1_deg.angle_target=-43;
//		Manipulator.pitch2_deg.angle_target=39;
//		Manipulator.pitch3_deg.angle_target = -115;
//		Manipulator.roll1_deg.angle_target=88;
//		Manipulator.roll2_deg.angle_target = -12;
//	}
//	if(associate_stretch_count == 6000)
//	{
//		Vacuum_pump_vice(open);
//		arm_relese(close);
//		Manipulator.yaw_deg.angle_target=35;
//		Manipulator.pitch3_deg.Accel=0.0008;
//		Manipulator.pitch3_deg.min_velocity=0.02;
//		Manipulator.pitch3_deg.angle_target=-80;
//	}
//	if(associate_stretch_count == 7000)
//	{	
//		valve_fetch(close);
//	}
//	if(associate_stretch_count>7000&&associate_stretch_count<7800)//下降一点
//    {  
//		Auto_left_up_goldmining(&Goldmining_t,900,14);
//    }
//	if(associate_stretch_count == 8000)
//	{
//		Manipulator.pitch1_deg.angle_target=-54;
//		Manipulator.pitch2_deg.angle_target=65;
//		
//	}
//	if(associate_stretch_count == 9000)
//	{
//		Manipulator.yaw_deg.angle_target=0;
//	}
//		if(associate_stretch_count == 9700)
//	{
//		Manipulator.pitch3_deg.angle_target=-15;
//		Manipulator.roll1_deg.angle_target=0;
//		Manipulator.roll2_deg.angle_target=0;
//		//position_left = 1000;
//	}
//		if(associate_stretch_count == 10200)
//	{
//		Manipulator.pitch1_deg.angle_target=-20;
//		Manipulator.pitch2_deg.angle_target=30;
//	}
//	 if(associate_stretch_count>11000)//最后结束置0
//     {
//         associate_arm_left_flag=0;
//         associate_stretch_count=0;
//         //Vacuum_pump_main(close);
//     }
//}

void associate_arm_left()//机械臂将矿石交接到取矿机构切换吸盘面(使用逆解，未写完)
{
     associate_stretch_count++;
	if(associate_stretch_count == 1)
	{
		position_left=260;
		Vacuum_pump_main(open);
		Manipulator.pitch1_deg.angle_target = -57;
		Manipulator.pitch2_deg.angle_target = 86;
		Manipulator.pitch3_deg.angle_target = -110;
		Manipulator.roll2_deg.angle_target = 28;
	}
	if(associate_stretch_count == 1000)
		Manipulator.yaw_deg.angle_target = 54;
	if(associate_stretch_count>1600&&associate_stretch_count<2500)
		Auto_left_up_goldmining(&Goldmining_t,8000,12);
	if(associate_stretch_count == 2000)
		valve_fetch(open);
	if(associate_stretch_count == 2500)
	{
		Manipulator.pitch1_deg.angle_target = -50;
		Manipulator.pitch2_deg.angle_target = 62;
		Manipulator.pitch3_deg.angle_target = -99;
	}
	if(associate_stretch_count == 3200)
	{
		Vacuum_pump_vice(close);
		arm_relese(open);
	}
	
	if(associate_stretch_count == 3201)
	{
		auto_flag = 0;
		Manipulator.pitch1_deg.angle_target = -52;
		Manipulator.pitch2_deg.angle_target = 88;
		Manipulator.pitch3_deg.angle_target = -110;
		Manipulator.yaw_deg.angle_target = 0;
		Manipulator.roll2_deg.angle_target = 0;
	}
	
	if(associate_stretch_count == 3900)
	{
		arm_relese(close);
		Vacuum_pump_vice(open);
		target_deg[0] = 0;
		target_deg[1] = -18;
		target_deg[2] = -93;
		
		target_p[0] = -197;
		target_p[1] = -82;
		target_p[2] = 305;
	}
	if(associate_stretch_count>4200&&associate_stretch_count<5200)
		Auto_left_up_goldmining(&Goldmining_t,8000,10);
	if(associate_stretch_count>4200&&associate_stretch_count<4300)
    {    
		vision_coil_test(target_deg,target_p);
		Manipulator.roll2_deg.angle_target -= 6;
    }
//	if(associate_stretch_count == 4500)
//	{
		//Manipulator.pitch3_deg.Accel = 0.0005;//拉高加速度，提高关节速度
		//Manipulator.pitch3_deg.min_velocity=0.09;//拉高速度，提高响应
//	}

	if(associate_stretch_count >= 5300&&associate_stretch_count <= 6200)
	{
		target_p[0] += 0.035;//sin(18)*117
		target_p[1] -= 0.111;//cos(18)*117
		vision_coil_test(target_deg,target_p);
		Manipulator.roll2_deg.angle_target -= 6;
	}
	if(associate_stretch_count == 6500)
	{	
		valve_fetch(close);
	}
	if(associate_stretch_count>6500&&associate_stretch_count<7500)//下降一点
    {  
		Auto_left_up_goldmining(&Goldmining_t,900,14);
    }
	if(associate_stretch_count == 6700)
	{
		Manipulator.pitch1_deg.angle_target=-54;
		Manipulator.pitch2_deg.angle_target=65;
		
	}
	if(associate_stretch_count == 7700)
	{
		Manipulator.yaw_deg.angle_target=0;
	}
		if(associate_stretch_count == 7800)
	{

		Manipulator.pitch3_deg.angle_target=-15;
		Manipulator.roll1_deg.angle_target=0;
		Manipulator.roll2_deg.angle_target=0;
		//position_left = 1000;
	}
		if(associate_stretch_count == 7800)
	{
		Manipulator.pitch1_deg.angle_target=-20;
		Manipulator.pitch2_deg.angle_target=30;
	}
	 if(associate_stretch_count>8500)//最后结束置0
     {
         associate_arm_left_flag=0;
         associate_stretch_count=0;
         //Vacuum_pump_main(close);
     }
	
}

void servo_reset()//舵机复位
{
	position_right=2820;
	position_left=1000;
}
float roll2_feed = 0;
float roll1_feed = 0;
float pitch3_feed = 30;
float pitch1_feed = -30;
float pitch2_feed = 20;
float yaw_feed = 0;
void move_feed_right()//兑矿前移
{
	move_feed_count ++;
	if(move_feed_count==10)
	{
		servo_move(4,50,740);
		
	}
	if(move_feed_count==50)
	{
		servo_move(2,30,475);
		pitch1_feed = -60;
		pitch2_feed = 85;
		yaw_feed = 0;
		Manipulator.pitch1_deg.angle_target=pitch1_feed;
		Manipulator.pitch2_deg.angle_target=pitch2_feed;
		Manipulator.yaw_deg.angle_target=yaw_feed;
	}
	if(move_feed_count==500)
	{
		
		roll1_feed = 90;
		pitch3_feed = -90;
		Manipulator.roll1_deg.angle_target=roll1_feed;
		Manipulator.pitch3_deg.angle_target=pitch3_feed;
		move_back_flag = 2;
	}
	
	if(move_feed_count==1500)
	{
		move_back_flag = 0;
		move_feed_right_flag=0;
		move_feed_count = 0;
		auto_flag = 0;
	}
	
	
	
}
void move_feed_left()//兑矿前置位移
{
	move_feed_count ++;
	if(move_feed_count==10)
	{
		servo_move(4,50,500);
		
	}
	if(move_feed_count==50)
	{
		servo_move(2,30,475);
		pitch1_feed = -55;
		pitch2_feed = 75;
		yaw_feed = 0;
		Manipulator.pitch1_deg.angle_target=pitch1_feed;
		Manipulator.pitch2_deg.angle_target=pitch2_feed;
		Manipulator.yaw_deg.angle_target=yaw_feed;
	}
	if(move_feed_count==500)
	{
		roll1_feed = 90;
		pitch3_feed = 90;
		Manipulator.roll1_deg.angle_target=roll1_feed;
		Manipulator.pitch3_deg.angle_target=pitch3_feed;
		move_back_flag = 3;
	}
	
	if(move_feed_count==1500)
	{
		move_back_flag = 0;
		move_feed_left_flag=0;
		move_feed_count = 0;
		auto_flag = 0;
	}
}
/**
* @brief 获取机械臂取单金关节参数
  */ 	
float get_single_gold_theta[3];
int8_t test_start = 0;
//550,100---550,150
float arm_get_single_gold_solvement(float x,float y)
{
	//计算模型下目标角度
	float theta1,theta2,L_c;
	float L1 = 330;
	float L2 = 357;
	L_c = sqrt(x*x+y*y);
    theta1 = asinf(y/L_c);
	theta2 = acosf((L1*L1 + L_c*L_c - L2*L2)/(2*L1*L_c));
	get_single_gold_theta[0] = rad2deg(theta1 + theta2) - 90;
	
	get_single_gold_theta[1] = acosf((L1*L1 + L2*L2 - L_c*L_c)/(2*L1*L2));
	get_single_gold_theta[1] = rad2deg(get_single_gold_theta[1]) - 90;
	
	get_single_gold_theta[2] = 93 - (get_single_gold_theta[0] + get_single_gold_theta[1]);
	
	//转换到实际目标角度
		Manipulator.pitch1_deg.angle_target = get_single_gold_theta[0] - 75;
		Manipulator.pitch2_deg.angle_target = get_single_gold_theta[1] + 80;
		Manipulator.pitch3_deg.angle_target = get_single_gold_theta[2] - 89;
}

float arm_get_single_gold()
{
	arm_get_single_gold_count++;
	
	if(arm_get_single_gold_count < 10)
	{
		arm_get_single_gold_solvement(550,100);
		arm_relese(close);
		Vacuum_pump_vice(open);
	}
		
	if(arm_get_single_gold_count == 900)
		arm_get_single_gold_flag = 0;
	
	if(arm_get_single_gold_count > 1100 && arm_get_single_gold_count <= 1600 && arm_get_single_gold_count%10 == 0)
		arm_get_single_gold_solvement(550,100+(arm_get_single_gold_count-1100)/10);
	if(arm_get_single_gold_count == 1800)
	{
		auto_flag = 1;
		move_back_flag = 1;
	}
	
	if(arm_get_single_gold_count == 3200)
	{
		auto_flag = 0;
		move_back_flag = 0;
		arm_get_single_gold_count = 0;
		arm_get_single_gold_flag = 0;
		Manipulator.pitch1_deg.angle_target = 0;
		Manipulator.pitch2_deg.angle_target = 0;
		Manipulator.pitch3_deg.angle_target = 0;
	}
	
}

/**
* @brief 获取自动模式标志位
  */ 		
void auto_judge()//
{
	if(move_angle==0)
	{
//		if(rc_Ctrl.rc.s2==1&&auto_flag==0)
//		{
//			if(rc_Ctrl.rc.sw>1224)
//			{
//			  auto_flag=1;
//			  get_double_gold_flag=1;
//			}
//		}
//		if(rc_Ctrl.rc.s2==1&&auto_flag==0)
//		{
//			if(rc_Ctrl.rc.sw<924)
//			{
//			  auto_flag=1;
//			  get_single_gold_flag=1;
//			}
//		}
		
//		if(rc_Ctrl.rc.s2==1&&auto_flag==0)
//		{
//			if(rc_Ctrl.rc.sw<924)
//			{
//				auto_flag=1;
//				get_double_silver_flag=1;
//			}
//		}
//		if(rc_Ctrl.rc.s2==1&&auto_flag==0)
//		{
//			if(rc_Ctrl.rc.sw>1124)
//			{
//				auto_flag=1;
//				get_single_silver_flag=1;
//			}
//		}
	
		if((rc_Ctrl.rc.s2==1 && auto_flag==0))
		{
			if(rc_Ctrl.rc.ch3<924)
			{
				servo_reset();
			}
		}
		if((rc_Ctrl.key_shift==1&&rc_Ctrl.key_X==1&&rc_Ctrl.key_ctrl==1))
		{
			servo_reset();
		   Vacuum_pump_main(close);
		   valve_fetch(close);
			gold_reset_flag =0;
		}
	
		if(rc_Ctrl.key_ctrl==1&&rc_Ctrl.key_C==1&&rc_Ctrl.key_shift==0&&auto_flag==0)//机械臂单独取银矿
		{
			auto_flag=1;
			get_single_silver_flag=1;
		}
		if(rc_Ctrl.key_ctrl==1&&rc_Ctrl.key_shift==1&&rc_Ctrl.key_C==1&&auto_flag==0)//取双银矿
		{
			auto_flag=1;
			get_double_silver_flag=1;
		}
		if(rc_Ctrl.key_ctrl==1&&rc_Ctrl.key_shift==1&&rc_Ctrl.key_B==1&&auto_flag==0)//取双金矿
		{
			auto_flag=1;
			get_double_gold_flag=1;
			
		}
		if(rc_Ctrl.key_ctrl==1&&rc_Ctrl.key_shift==0&&rc_Ctrl.key_B==1&&auto_flag==0)//取单金矿
		{
			auto_flag=1;
			get_single_gold_flag=1;
		}
		if(rc_Ctrl.key_shift==1&&rc_Ctrl.key_ctrl==0&&rc_Ctrl.key_V==1&&auto_flag==0)//机械臂取单金
		{
			arm_get_single_gold_flag=1;
		}
		if(rc_Ctrl.key_shift==1&&rc_Ctrl.key_Z==1&&auto_flag==0&&CC_back_flag == 0)
		{
			auto_flag=1;
			associate_stretch_auto_flag=1;
		}
		if(rc_Ctrl.key_ctrl==1&&rc_Ctrl.key_Z==1&&auto_flag==0&&CC_back_flag==0)
		{
			auto_flag=1;
			associate_arm_left_flag=1;
		}
		if(rc_Ctrl.key_shift==1&&rc_Ctrl.key_D==1&&auto_flag==0)
		{
			auto_flag=1;
			move_feed_right_flag = 1;
		}
		if(rc_Ctrl.key_shift==1&&rc_Ctrl.key_A==1&&auto_flag==0)
		{
			auto_flag=1;
			move_feed_left_flag = 1;
		}
		if(rc_Ctrl.key_shift==1&&rc_Ctrl.key_R==1)//取消自动模式，机械臂收回
		{
			Manipulator.yaw_deg.angle_target = 0;
			Manipulator.roll2_deg.angle_target = 0;
			Manipulator.roll1_deg.angle_target = 0;
			Manipulator.pitch3_deg.angle_target = 0;
			Manipulator.pitch1_deg.angle_target = 0;
			Manipulator.pitch2_deg.angle_target = 0;
			auto_flag=0;
			
			get_single_silver_flag = 0;
			get_single_gold_flag = 0;
			get_double_silver_flag = 0;
			get_double_gold_flag = 0;
			associate_stretch_left_flag=0;//交接左侧取矿机构标志位
			associate_stretch_right_flag=0;//交接右侧取矿机构标志位
			associate_arm_left_flag=0;//机械臂交接左侧取矿机构过度标志位
			arm_get_single_gold_flag = 0;
			
			get_single_silver_count=0;
			get_single_gold_count=0;
			get_double_silver_count=0;
			get_double_gold_count=0;
			associate_stretch_count=0;//交接时间戳
			arm_get_single_gold_count=0;
		}
		
		/*vt13*/
		if((vT13.key_shift==1&&vT13.key_X==1&&vT13.key_ctrl==1))
		{
			servo_reset();
		  Vacuum_pump_main(close);
		  valve_fetch(close);
		  gold_reset_flag =0;
		}
	
		if(vT13.key_ctrl==1&&vT13.key_C==1&&vT13.key_shift==0&&auto_flag==0)//机械臂单独取银矿
		{
			auto_flag=1;
			get_single_silver_flag=1;
		}
		if(vT13.key_ctrl==1&&vT13.key_shift==1&&vT13.key_C==1&&auto_flag==0)//取双银矿
		{
			auto_flag=1;
			get_double_silver_flag=1;
		}
		if(vT13.key_ctrl==1&&vT13.key_shift==1&&vT13.key_B==1&&auto_flag==0)//取双金矿
		{
			auto_flag=1;
			get_double_gold_flag=1;
		}
		if(vT13.key_ctrl==1&&vT13.key_shift==0&&vT13.key_B==1&&auto_flag==0)//取单金矿
		{
			auto_flag=1;
			get_single_gold_flag=1;
		}
		if(vT13.key_shift==1&&vT13.key_ctrl==0&&vT13.key_V==1&&auto_flag==0)//取双金矿
		{
			arm_get_single_gold_flag=1;
		}
		if(vT13.rc.mode_sw==1&&vT13.key_shift==1&&vT13.key_Z==1&&auto_flag==0&&CC_back_flag==0)
		{
			auto_flag=1;
			associate_stretch_auto_flag=1;
		}
		if(vT13.key_ctrl==1&&vT13.key_Z==1&&auto_flag==0)
		{
			auto_flag=1;
			associate_arm_left_flag=1;
		}
		if(vT13.key_shift==1&&vT13.key_D==1&&auto_flag==0)
		{
			auto_flag=1;
			move_feed_right_flag = 1;
		}
		if(vT13.key_shift==1&&vT13.key_A==1&&auto_flag==0)
		{
			auto_flag=1;
			move_feed_left_flag = 1;
		}
		if(vT13.key_shift==1&&vT13.key_R==1)//取消自动模式，机械臂收回
		{
			Manipulator.yaw_deg.angle_target = 0;
			Manipulator.roll2_deg.angle_target = 0;
			Manipulator.roll1_deg.angle_target = 0;
			Manipulator.pitch3_deg.angle_target = 0;
			Manipulator.pitch1_deg.angle_target = 0;
			Manipulator.pitch2_deg.angle_target = 0;
			auto_flag=0;
			
			get_single_silver_flag = 0;
			get_single_gold_flag = 0;
			get_double_silver_flag = 0;
			get_double_gold_flag = 0;
			associate_stretch_left_flag=0;//交接左侧取矿机构标志位
			associate_stretch_right_flag=0;//交接右侧取矿机构标志位
			associate_arm_left_flag=0;//机械臂交接左侧取矿机构过度标志位
			arm_get_single_gold_flag=0;
			
			get_single_silver_count=0;
			get_single_gold_count=0;
			get_double_silver_count=0;
			get_double_gold_count=0;
			associate_stretch_count=0;//交接时间戳
			arm_get_single_gold_count=0;
   
		}
		
		
	}
		 if(rc_Ctrl.rc.s2==1)
	{
		arm_relese(open);
	    Vacuum_pump_main(close);
		Vacuum_pump_vice(close);
		valve_fetch(close);
	}
	if(rc_Ctrl.rc.s1==2)
	{
		position_right=2820;//右侧
        position_left=1000;
	}
	if((rc_Ctrl.key_R == 1 && rc_Ctrl.key_ctrl == 1)||(vT13.key_R == 1 && vT13.key_ctrl == 1))
	{
	    Vacuum_pump_main(close);
		Vacuum_pump_vice(close);
		valve_fetch(close);
		arm_relese(open);
	}
	if(rc_Ctrl.rc.s2==2)
	{
		Vacuum_pump_main(open);
		Vacuum_pump_vice(open);
		arm_relese(close);
		valve_fetch(open);
		position_right=1600;
        position_left=620;
		
	}
	if((rc_Ctrl.key_shift==1&&rc_Ctrl.key_W==1&&move_feed_left_flag == 0&&move_feed_right_flag ==0)||(vT13.key_shift==1&&vT13.key_W==1&&move_feed_left_flag == 0&&move_feed_right_flag == 0))
	{
		 Manipulator.yaw_deg.angle_target=10;
		 Manipulator.pitch1_deg.angle_target=-30;
		 Manipulator.pitch2_deg.angle_target=5;
	}
	
	if((rc_Ctrl.key_shift==1&&rc_Ctrl.key_S==1)||(vT13.key_shift==1&&vT13.key_S==1))
	{
		 Manipulator.yaw_deg.angle_target=0;
		 Manipulator.pitch1_deg.angle_target=0;
		 Manipulator.pitch2_deg.angle_target=0;
		 Manipulator.pitch3_deg.angle_target=0;
	}
	
	
	
//		 if(rc_Ctrl.rc.s2==2)
//	{
//	    Vacuum_pump_main(open);
//	}
}

/**
* @brief 根据自动模式标志位执行自动模式
  */ 	
void auto_mode()//
{
	if(get_single_silver_flag==1)
	   get_single_silver();
    if(get_single_gold_flag==1)
	   get_single_gold();
	if(arm_get_single_gold_flag==1)
	   arm_get_single_gold();
	if(get_double_gold_flag==1)
	   get_double_gold_manual();
	if(get_double_silver_flag==1)
	   get_double_silver();
	if(associate_stretch_auto_flag==1)
		associate_stretch_auto();
	if(associate_stretch_right_flag==1)
		associate_stretch_right();
	if(associate_stretch_left_flag==1)
		associate_stretch_left();
	if(associate_arm_left_flag==1)
		associate_arm_left();
	if(move_feed_right_flag == 1)
		move_feed_right();
	if(move_feed_left_flag == 1)
		move_feed_left();
	
	if(rc_Ctrl.isOnline == 0)
	{
		get_single_silver_flag = 0;
		get_single_gold_flag = 0;
		get_double_silver_flag = 0;
		get_double_gold_flag = 0;
		arm_get_single_gold_flag=0;
		associate_stretch_left_flag=0;//交接左侧取矿机构标志位
		associate_stretch_right_flag=0;//交接右侧取矿机构标志位
		associate_arm_left_flag=0;//机械臂交接左侧取矿机构过度标志位
		
		auto_flag = 0;
		
		get_single_silver_count=0;
		get_single_gold_count=0;
		get_double_silver_count=0;
		get_double_gold_count=0;
		associate_stretch_count=0;//交接时间戳
		arm_get_single_gold_count=0;
	}

}
