#include "gold.h"
#include "pid.h"
#include "hardware_config.h"
#include "vt13.h"
#include "auto.h"
float stretch0_out_limit_min=-35000;
float stretch0_out_limit_max= 0;
float stretch0_up_limit_min = 0;
float stretch0_up_limit_max=27125;
float stretch1_out_limit_min= 0;
float stretch1_out_limit_max=49695;
float stretch1_up_limit_min = -38000;
float stretch1_up_limit_max=0;

int8_t gold_reset_flag=0;
int16_t gold_reset_count=0;
//取矿机构初始化
void Goldmining_Init(Goldmining*goldmining,BasePID_Object motor_strechout_speed_pid,BasePID_Object motor_strechup_speed_pid,BasePID_Object motor_strechout_angle_pid,BasePID_Object motor_strechup_angle_pid,CanNumber canx){
    MotorInit(&goldmining->Motors2006.goldmotor[0],128,Motor2006,CAN2,0x201);//左侧机构抬升
    MotorInit(&goldmining->Motors2006.goldmotor[1],128,Motor2006,CAN2,0x202);//左侧机构伸出
    MotorInit(&goldmining->Motors2006.goldmotor[2],128,Motor2006,CAN2,0x203);//右侧机构抬升
    MotorInit(&goldmining->Motors2006.goldmotor[3],128,Motor2006,CAN2,0x204);//右侧机构伸出
    //turnpid
    BasePID_Init(&goldmining->Motors2006.turnpid[0],motor_strechup_speed_pid.Kp,motor_strechup_speed_pid.Ki,motor_strechup_speed_pid.Kd,motor_strechup_speed_pid.KiPartDetachment);
    BasePID_Init(&goldmining->Motors2006.turnpid[1],motor_strechout_speed_pid.Kp,motor_strechout_speed_pid.Ki,motor_strechout_speed_pid.Kd,motor_strechout_speed_pid.KiPartDetachment);
    BasePID_Init(&goldmining->Motors2006.turnpid[2], motor_strechup_angle_pid.Kp, motor_strechup_angle_pid.Ki, motor_strechup_angle_pid.Kd, motor_strechup_angle_pid.KiPartDetachment);
    BasePID_Init(&goldmining->Motors2006.turnpid[3],motor_strechout_angle_pid.Kp,motor_strechout_angle_pid.Ki,motor_strechout_angle_pid.Kd,motor_strechout_angle_pid.KiPartDetachment);
}
//角度累加函数
void Angle_add(Goldmining *goldmining,RC_Ctrl *rc_ctrl){
    if(rc_ctrl->rc.s1==1){
    if(rc_ctrl->rc.ch3>1400&&rc_Ctrl.isOnline==1){
            goldmining->stretchup[0].Target_Angle+=7;
        }
    if(rc_ctrl->rc.ch3<800&&rc_Ctrl.isOnline==1){
            goldmining->stretchup[0].Target_Angle-=7;
        }
    if(rc_ctrl->rc.ch2>1400&&rc_Ctrl.isOnline==1){
            goldmining->stretchout[0].Target_Angle-=7;
    }
    if(rc_ctrl->rc.ch2<800&&rc_Ctrl.isOnline==1){
            goldmining->stretchout[0].Target_Angle+=7;
    }
    if(rc_ctrl->rc.ch1>1400&&rc_Ctrl.isOnline==1){
            goldmining->stretchup[1].Target_Angle-=7;
        }
    if(rc_ctrl->rc.ch1<800&&rc_Ctrl.isOnline==1){
            goldmining->stretchup[1].Target_Angle+=7;
        }
    if(rc_ctrl->rc.ch0>1400&&rc_Ctrl.isOnline==1){
            goldmining->stretchout[1].Target_Angle+=7;
    }
    if(rc_ctrl->rc.ch0<800&&rc_Ctrl.isOnline==1){
            goldmining->stretchout[1].Target_Angle-=7;
    }
	
	
  }
	if(get_double_gold_manual_flag==1)
	{
		if(vT13.rc.ch2>1400&&rc_Ctrl.isOnline==1){
			   goldmining->stretchup[0].Target_Angle+=3;
		}
		if(vT13.rc.ch2<800&&rc_Ctrl.isOnline==1){
				goldmining->stretchup[0].Target_Angle-=3;
			}
		if(vT13.rc.ch3>1400&&rc_Ctrl.isOnline==1){
				goldmining->stretchout[0].Target_Angle-=7;
		}
		if(vT13.rc.ch3<800&&rc_Ctrl.isOnline==1){
				goldmining->stretchout[0].Target_Angle+=7;
		}
		if(vT13.rc.ch1>1400&&rc_Ctrl.isOnline==1){
				goldmining->stretchup[1].Target_Angle-=3;
			}
		if(vT13.rc.ch1<800&&rc_Ctrl.isOnline==1){
				goldmining->stretchup[1].Target_Angle+=3;
			}
		if(vT13.rc.ch0>1400&&rc_Ctrl.isOnline==1){
				goldmining->stretchout[1].Target_Angle+=7;
		}
		if(vT13.rc.ch0<800&&rc_Ctrl.isOnline==1){
				goldmining->stretchout[1].Target_Angle-=7;
		}
		
		if(rc_Ctrl.rc.ch3>1400&&rc_Ctrl.isOnline==1){
			   goldmining->stretchup[0].Target_Angle+=3;
		}
		if(rc_Ctrl.rc.ch3<800&&rc_Ctrl.isOnline==1){
				goldmining->stretchup[0].Target_Angle-=3;
			}
		if(rc_Ctrl.rc.ch2>1400&&rc_Ctrl.isOnline==1){
				goldmining->stretchout[0].Target_Angle-=7;
		}
		if(rc_Ctrl.rc.ch2<800&&rc_Ctrl.isOnline==1){
				goldmining->stretchout[0].Target_Angle+=7;
		}
		if(rc_Ctrl.rc.ch1>1400&&rc_Ctrl.isOnline==1){
				goldmining->stretchup[1].Target_Angle-=3;
			}
		if(rc_Ctrl.rc.ch1<800&&rc_Ctrl.isOnline==1){
				goldmining->stretchup[1].Target_Angle+=3;
			}
		if(rc_Ctrl.rc.ch0>1400&&rc_Ctrl.isOnline==1){
				goldmining->stretchout[1].Target_Angle+=7;
		}
		if(rc_Ctrl.rc.ch0<800&&rc_Ctrl.isOnline==1){
				goldmining->stretchout[1].Target_Angle-=7;
		}
	
	}
//左右抬升伸出限幅
    if(goldmining->stretchup[0].Target_Angle<stretch0_up_limit_min){
        goldmining->stretchup[0].Target_Angle=stretch0_up_limit_min;
    }
    if(goldmining->stretchup[0].Target_Angle>stretch0_up_limit_max){
        goldmining->stretchup[0].Target_Angle=stretch0_up_limit_max;
    }
    if(goldmining->stretchup[1].Target_Angle<stretch1_up_limit_min){
        goldmining->stretchup[1].Target_Angle=stretch1_up_limit_min;
    }
    if(goldmining->stretchup[1].Target_Angle>stretch1_up_limit_max){
        goldmining->stretchup[1].Target_Angle=stretch1_up_limit_max;
    }
    if(goldmining->stretchout[0].Target_Angle<stretch0_out_limit_min){
        goldmining->stretchout[0].Target_Angle=stretch0_out_limit_min;
    }
    if(goldmining->stretchout[0].Target_Angle>stretch0_out_limit_max){
        goldmining->stretchout[0].Target_Angle=stretch0_out_limit_max;
    }
    if(goldmining->stretchout[1].Target_Angle<stretch1_out_limit_min){
        goldmining->stretchout[1].Target_Angle=stretch1_out_limit_min;
    }
    if(goldmining->stretchout[1].Target_Angle>stretch1_out_limit_max){
        goldmining->stretchout[1].Target_Angle=stretch1_out_limit_max;
    }
	if(rc_ctrl->isOnline==1 && gold_reset_flag ==1)
	{
//角度解算
    if(goldmining->Motors2006.goldmotor[0].Data.Angle<-100&&goldmining->stretchup[0].angle_last>100){
        goldmining->stretchup[0].angle+=360+goldmining->Motors2006.goldmotor[0].Data.Angle-goldmining->stretchup[0].angle_last;
    }
    else if(goldmining->Motors2006.goldmotor[0].Data.Angle>100&&goldmining->stretchup[0].angle_last<-100){
        goldmining->stretchup[0].angle+=-360+goldmining->Motors2006.goldmotor[0].Data.Angle-goldmining->stretchup[0].angle_last;
        
    }
    else{goldmining->stretchup[0].angle+=goldmining->Motors2006.goldmotor[0].Data.Angle-goldmining->stretchup[0].angle_last;
        
    }
    goldmining->stretchup[0].angle_last=goldmining->Motors2006.goldmotor[0].Data.Angle;
    
    //
    if(goldmining->Motors2006.goldmotor[1].Data.Angle<-100&&goldmining->stretchout[0].angle_last>100){
        goldmining->stretchout[0].angle+=360+goldmining->Motors2006.goldmotor[1].Data.Angle-goldmining->stretchout[0].angle_last;
    }
    else if(goldmining->Motors2006.goldmotor[1].Data.Angle>100&&goldmining->stretchout[0].angle_last<-100){
        goldmining->stretchout[0].angle+=-360+goldmining->Motors2006.goldmotor[1].Data.Angle-goldmining->stretchout[0].angle_last;
        
    }
    else{goldmining->stretchout[0].angle+=goldmining->Motors2006.goldmotor[1].Data.Angle-goldmining->stretchout[0].angle_last;
        
    }
    goldmining->stretchout[0].angle_last=goldmining->Motors2006.goldmotor[1].Data.Angle;
     
    //
    if(goldmining->Motors2006.goldmotor[2].Data.Angle<-100&&goldmining->stretchup[1].angle_last>100){
        goldmining->stretchup[1].angle+=360+goldmining->Motors2006.goldmotor[2].Data.Angle-goldmining->stretchup[1].angle_last;
    }
    else if(goldmining->Motors2006.goldmotor[2].Data.Angle>100&&goldmining->stretchup[1].angle_last<-100){
        goldmining->stretchup[1].angle+=-360+goldmining->Motors2006.goldmotor[2].Data.Angle-goldmining->stretchup[1].angle_last;
        
    }
    else{goldmining->stretchup[1].angle+=goldmining->Motors2006.goldmotor[2].Data.Angle-goldmining->stretchup[1].angle_last;
        
    }
    goldmining->stretchup[1].angle_last=goldmining->Motors2006.goldmotor[2].Data.Angle;
    
    //
    if(goldmining->Motors2006.goldmotor[3].Data.Angle<-100&&goldmining->stretchout[1].angle_last>100){
        goldmining->stretchout[1].angle+=360+goldmining->Motors2006.goldmotor[3].Data.Angle-goldmining->stretchout[1].angle_last;
    }
    else if(goldmining->Motors2006.goldmotor[3].Data.Angle>100&&goldmining->stretchout[1].angle_last<-100){
        goldmining->stretchout[1].angle+=-360+goldmining->Motors2006.goldmotor[3].Data.Angle-goldmining->stretchout[1].angle_last;
        
    }
    else{goldmining->stretchout[1].angle+=goldmining->Motors2006.goldmotor[3].Data.Angle-goldmining->stretchout[1].angle_last;
        
    }
	  goldmining->stretchout[1].angle_last=goldmining->Motors2006.goldmotor[3].Data.Angle;
    
      goldmining ->Motors2006.goldmotor[0].Data.Output=Stretch_speedPID_control((BasePID_Object*)goldmining->Motors2006.turnpid,Stretch_anglePID_control((BasePID_Object*)goldmining->Motors2006.turnpid+2,goldmining ->stretchup[0].Target_Angle,goldmining ->stretchup[0].angle),goldmining ->Motors2006.goldmotor[0].Data.SpeedRPM);
      goldmining ->Motors2006.goldmotor[1].Data.Output=Stretch_speedPID_control((BasePID_Object*)goldmining->Motors2006.turnpid+1,Stretch_anglePID_control((BasePID_Object*)goldmining->Motors2006.turnpid+3,goldmining ->stretchout[0].Target_Angle,goldmining ->stretchout[0].angle),goldmining ->Motors2006.goldmotor[1].Data.SpeedRPM);
      goldmining ->Motors2006.goldmotor[2].Data.Output=Stretch_speedPID_control((BasePID_Object*)goldmining->Motors2006.turnpid,Stretch_anglePID_control((BasePID_Object*)goldmining->Motors2006.turnpid+2,goldmining ->stretchup[1].Target_Angle,goldmining ->stretchup[1].angle),goldmining ->Motors2006.goldmotor[2].Data.SpeedRPM);
      goldmining ->Motors2006.goldmotor[3].Data.Output=Stretch_speedPID_control((BasePID_Object*)goldmining->Motors2006.turnpid+1,Stretch_anglePID_control((BasePID_Object*)goldmining->Motors2006.turnpid+3,goldmining ->stretchout[1].Target_Angle,goldmining ->stretchout[1].angle),goldmining ->Motors2006.goldmotor[3].Data.SpeedRPM);
      for(int i=0;i<4;i++){
          if(goldmining ->Motors2006.goldmotor[i].Data.Output>13000)
          {goldmining ->Motors2006.goldmotor[i].Data.Output=13000;}
          if(goldmining ->Motors2006.goldmotor[i].Data.Output<-13000)
          {goldmining ->Motors2006.goldmotor[i].Data.Output=-13000;}
      }
      MotorFillData(&Goldmining_t.Motors2006.goldmotor[0],goldmining ->Motors2006.goldmotor[0].Data.Output);
      MotorFillData(&Goldmining_t.Motors2006.goldmotor[1],goldmining ->Motors2006.goldmotor[1].Data.Output);
      MotorFillData(&Goldmining_t.Motors2006.goldmotor[2],goldmining ->Motors2006.goldmotor[2].Data.Output);
      MotorFillData(&Goldmining_t.Motors2006.goldmotor[3],goldmining ->Motors2006.goldmotor[3].Data.Output);

  }
}
void Angle_add_init(Goldmining *goldmining)//初始化last值，保证累加器从0开始
{

	goldmining->stretchup[0].angle_last=goldmining->Motors2006.goldmotor[0].Data.Angle;
	goldmining->stretchout[0].angle_last=goldmining->Motors2006.goldmotor[1].Data.Angle;
	goldmining->stretchup[1].angle_last=goldmining->Motors2006.goldmotor[2].Data.Angle;
	goldmining->stretchout[1].angle_last=goldmining->Motors2006.goldmotor[3].Data.Angle;
	
	goldmining->stretchout[0].angle=0;
	goldmining->stretchout[1].angle=0;
	goldmining->stretchup[0].angle=0;
	goldmining->stretchup[1].angle=0;
	
	goldmining->stretchup[0].Target_Angle = 0;
	goldmining->stretchup[1].Target_Angle = 0;
	goldmining->stretchout[0].Target_Angle = 0;
	goldmining->stretchout[1].Target_Angle = 0;

}

void Auto_left_up_goldmining(Goldmining *goldmining,float  auto_target,float  auto_param)
{
    if(auto_target>goldmining->stretchup[0].Target_Angle)
    {
        goldmining->stretchup[0].Target_Angle+=auto_param;
    }
    if(auto_target<goldmining->stretchup[0].Target_Angle)
    {
        goldmining->stretchup[0].Target_Angle-=auto_param;
    }
    
}
void Auto_right_up_goldmining(Goldmining *goldmining,float auto_target,float  auto_param)
{
    if(auto_target>goldmining->stretchup[1].Target_Angle)
    {
        goldmining->stretchup[1].Target_Angle+=auto_param;
    }
    if(auto_target<goldmining->stretchup[1].Target_Angle)
    {
        goldmining->stretchup[1].Target_Angle-=auto_param;
    }
   
}
void Auto_left_out_goldmining(Goldmining *goldmining,float  auto_target,float  auto_param)
{
    if(auto_target>goldmining->stretchout[0].Target_Angle)
    {
        goldmining->stretchout[0].Target_Angle+=auto_param;
    }
    if(auto_target<goldmining->stretchout[0].Target_Angle)
    {
        goldmining->stretchout[0].Target_Angle-=auto_param;
    }
}
void Auto_right_out_goldmining(Goldmining *goldmining,float  auto_target,float  auto_param)
{
    if(auto_target>goldmining->stretchout[1].Target_Angle)
    {
        goldmining->stretchout[1].Target_Angle+=auto_param;
    }
    if(auto_target<goldmining->stretchout[1].Target_Angle)
    {
        goldmining->stretchout[1].Target_Angle-=auto_param;
    }
}

void gold_reset(Goldmining*goldmining)//取矿机构复位函数
{
	if(gold_reset_flag==0)
	{
		gold_reset_count++;
		if(gold_reset_count<1500)
		{
			MotorFillData(&Goldmining_t.Motors2006.goldmotor[0],-180);
			MotorFillData(&Goldmining_t.Motors2006.goldmotor[1],1000);
			MotorFillData(&Goldmining_t.Motors2006.goldmotor[2],300);
			MotorFillData(&Goldmining_t.Motors2006.goldmotor[3],-1300);
		}
		if(gold_reset_count>1500)
		{
			gold_reset_count=0;
			gold_reset_flag=1;
//			goldmining->stretchup[0].angle_last=0;
//			goldmining->stretchup[0].angle=0;
//			goldmining->stretchup[1].angle_last=0;
//			goldmining->stretchup[1].angle=0;
//			goldmining->stretchout[0].angle_last=0;
//			goldmining->stretchout[0].angle=0;
//			goldmining->stretchout[1].angle_last=0;
//			goldmining->stretchout[1].angle=0;
			Angle_add_init(goldmining);
			MotorFillData(&Goldmining_t.Motors2006.goldmotor[0],0);
		    MotorFillData(&Goldmining_t.Motors2006.goldmotor[1],-0);
            MotorFillData(&Goldmining_t.Motors2006.goldmotor[2],0);
            MotorFillData(&Goldmining_t.Motors2006.goldmotor[3],-0);
			gold_reset_count=0;
		}
	}
	
	
};
	

