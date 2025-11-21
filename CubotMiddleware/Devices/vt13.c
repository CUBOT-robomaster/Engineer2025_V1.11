#include "vt13.h"

VT13 vT13;//遥控
/*vt13遥控器*/
void vt13_recv_datas_modify(uint8_t *pdata,VT13* vt13 )
{  

	uint16_t Header;
	uint16_t crc16_temp;
	Header = pdata[0]<<8| pdata[1];
	//crc16_temp = Get_CRC16_Check((unsigned char*)pdata, 2+rc_data_lenth, 0xFFFF);
	if(Header == 0xA953)
	{
		vt13->onlineCheckCnt ++ ;
		/*注意新遥控的ch2和ch3和dt7是反的*/
		vt13->rc.ch0 = 	(pdata[2]| (pdata[3] << 8)) & 0x07ff;
		vt13->rc.ch1 = 	((pdata[3]>>3)| (pdata[4] << 5)) & 0x07ff;
		vt13->rc.ch2 =	((pdata[4] >> 6) | (pdata[5] << 2) |(pdata[6] << 10)) & 0x07ff; 						//< Channel 2
		vt13->rc.ch3 = 	((pdata[6] >> 1) | (pdata[7] << 7)) & 0x07ff; 	
		vt13->rc.mode_sw = ((pdata[7]>>4)&0x03);
		if(vt13->rc.mode_sw > 1)
			vt13->rc.mode_sw = 1;//需要修改
		vt13->rc.go_home = ((pdata[7]>>6)&0x01);
		vt13->rc.Fn= ((pdata[7]>>7));
		vt13->rc.change = (pdata[8]&0x01);
		vt13->rc.sw = ((pdata[8]>>1)|(pdata[9]<<7)) & 0x07ff;
		vt13->rc.photo = (pdata[9]>>4)&0x03;
		vt13->mouse.x = (pdata[10])|(pdata[11]<<8);
		vt13->mouse.y = (pdata[12])|(pdata[13]<<8);
		vt13->mouse.z = (pdata[14])|(pdata[15]<<8);
		vt13->mouse.press_l = pdata[16]&0x03;
		vt13->mouse.press_r = (pdata[16]>>2)&0x03;
		vt13->mouse.press_m = (pdata[16]>>4)&0x03;//鼠标中键
			vt13->keyboard.keyboard14=pdata[17];
			vt13->keyboard.keyboard15=pdata[18];
			vt13->key_W=pdata[17]&0x01;	
			vt13->key_S=(pdata[17]>>1)&0x01;					
			vt13->key_A=(pdata[17]>>2)&0x01;
			vt13->key_D=(pdata[17]>>3)&0x01;
			vt13->key_shift=(pdata[17]>>4)&0x01;	
			vt13->key_ctrl=(pdata[17]>>5)&0x01;
			vt13->key_Q=(pdata[17]>>6)&0x01;		
			vt13->key_E=(pdata[17]>>7)&0x01;
			vt13->key_B=(pdata[18]>>7)&0x01;
			vt13->key_V=(pdata[18]>>6)&0x01;				
			vt13->key_C=(pdata[18]>>5)&0x01;
			vt13->key_X=(pdata[18]>>4)&0x01;					
			vt13->key_Z=(pdata[18]>>3)&0x01;					
			vt13->key_G=(pdata[18]>>2)&0x01;			
			vt13->key_F=(pdata[18]>>1)&0x01;
			vt13->key_R=(pdata[18])&0x01;													
		
			PC_keybroad_filter_vt13(&vT13);
		
	}
}

void VT13Init( VT13 * RC_Ctl)
{
		RC_Ctl->rc.ch0=1024;
		RC_Ctl->rc.ch1=1024;
		RC_Ctl->rc.ch2=1024;
		RC_Ctl->rc.ch3=1024;
		RC_Ctl->rc.sw=1024;
		RC_Ctl->mouse.x=0;
		RC_Ctl->mouse.y=0;
		RC_Ctl->mouse.z=0;		
		RC_Ctl->key_Q_flag=0;
		RC_Ctl->key_E_flag=0;   
		RC_Ctl->key_R_flag=0;
		RC_Ctl->key_F_flag=0;
		RC_Ctl->key_G_flag=0;
		RC_Ctl->key_Z_flag=0;
		RC_Ctl->key_X_flag=0;
		RC_Ctl->key_C_flag=0;
		RC_Ctl->key_V_flag=0;
		RC_Ctl->key_B_flag=0;
}
	
void PC_keybroad_filter_vt13(VT13* RC_Ctl)
{
	if(RC_Ctl->rc.mode_sw!=0){
	static uint16_t key_W_cnt,key_A_cnt,key_S_cnt,key_D_cnt,key_ctrl_cnt,
								 key_shift_cnt,mouse_press_l_cnt,mouse_press_r_cnt,
								key_C_cnt,key_F_cnt,key_G_cnt,key_Q_cnt,key_E_cnt,
								key_Z_cnt,key_V_cnt,key_X_cnt,key_B_cnt,key_R_cnt,key_shift_ctrl_z_cnt,key_shift_ctrl_x_cnt,key_shift_ctrl_v_cnt,key_shift_Q_cnt;
	static uint16_t key_shift_c_cnt,key_shift_ctrl_r_cnt,key_shift_V_cnt,key_shift_b_cnt,key_ctrl_shift_b_cnt;
			/*   支持连续按 W A S D   */
			//< key_W
	if(RC_Ctl->key_W==1) 
	{
		key_W_cnt++;
		if(key_W_cnt==Key_Filter_Num_vT13)	
		{
			RC_Ctl->key_W_flag=1;	
		}	 
	}   
	else
	{
		RC_Ctl->key_W_flag=0;	
		key_W_cnt=0;	
	}	
			//key_A
	if(RC_Ctl->key_A==1) 
	{
		key_A_cnt++;
		if(key_A_cnt==Key_Filter_Num_vT13)	
		{
			RC_Ctl->key_A_flag=1;
		}	
	}

	else
	{	
		key_A_cnt=0;	
		RC_Ctl->key_A_flag=0;
	}
	//key_S
	if(RC_Ctl->key_S==1) 
	{
		key_S_cnt++;
		if(key_S_cnt==Key_Filter_Num_vT13)	
		{
			RC_Ctl->key_S_flag=1;	
		}			
	}	
	else
	{
		key_S_cnt=0;
		RC_Ctl->key_S_flag=0;
	}		
	//key_D
	if(RC_Ctl->key_D==1) 
	{
		key_D_cnt++;
		if(key_D_cnt==Key_Filter_Num_vT13)	
		{
			RC_Ctl->key_D_flag=1;			
		}	
	}		
	else
	{
		key_D_cnt=0;
		RC_Ctl->key_D_flag=0;
	}
	
		//key_B
	if(RC_Ctl->key_B==1) 
	{
		key_B_cnt++;
		if(key_B_cnt==Key_Filter_Num_vT13)	
		{
			RC_Ctl->key_B_flag=1;
		}	
	}	
	else
	{
		key_B_cnt=0;
		RC_Ctl->key_B_flag=0;
	}
	//key_C
	if(RC_Ctl->key_C==1) 
	{
		key_C_cnt++;
		if(key_C_cnt==Key_Filter_Num_vT13)	
		{
			RC_Ctl->key_C_flag=1;
		}	 
	}  
	else
	{
		key_C_cnt=0;
		RC_Ctl->key_C_flag=0;
	}	
	
	//key_R
	if(RC_Ctl->key_R==1) 
	{
		key_R_cnt++;
		if(key_R_cnt==Key_Filter_Num_vT13)	
		{
			RC_Ctl->key_R_flag++;	
		}
	}   
	else
	{
		key_R_cnt=0;	
	}			
	
	
	//key_F
	if(RC_Ctl->key_F==1) 
	{
		key_F_cnt++;
		if(key_F_cnt==Key_Filter_Num_vT13)	
		{
			RC_Ctl->key_F_flag=1;	
		}	
	}	
	else
	{
		key_F_cnt=0;
		RC_Ctl->key_F_flag=0;	
	}
		//key_X
	if(RC_Ctl->key_X==1) 
	{
		key_X_cnt++;
		if(key_X_cnt==Key_Filter_Num_vT13)	
		{
		RC_Ctl->key_X_flag=1;			
		}	 
	}	
	else
	{
		key_X_cnt=0;
		RC_Ctl->key_X_flag=0;
	}
	
	//key_G
	if(RC_Ctl->key_G==1) 
	{
		key_G_cnt++;
		if(key_G_cnt==Key_Filter_Num_vT13)	
		{
			RC_Ctl->key_G_flag++;
		}	
	}	
	else
	{
		key_G_cnt=0;
//		RC_Ctl.key_G_flag=0;
	}	

	
	//key_Q
	if(RC_Ctl->key_Q==1) 
	{
		key_Q_cnt++;
		if(key_Q_cnt==Key_Filter_Num_vT13)	
		{
		RC_Ctl->key_Q_flag++;			
		}		
	}	
	else
	{
		key_Q_cnt=0;
	}
	//key_E
	if(RC_Ctl->key_E==1) 
	 {
		key_E_cnt++;
		if(key_E_cnt==Key_Filter_Num_vT13)	
		{
			RC_Ctl->key_E_flag++;	
		}	 
	 }
	 else
	 {
		key_E_cnt=0;
	//	RC_Ctl->key_E_flag=0;
	 }	
	//key_Z
	if(RC_Ctl->key_Z==1) 
	{
		key_Z_cnt++;
		if(key_Z_cnt==Key_Filter_Num_vT13)	
		{
			RC_Ctl->key_Z_flag=1;
		}			
	}
	else
	 {
		key_Z_cnt=0;
		RC_Ctl->key_Z_flag=0;
	 } 
	//key_V
		if(RC_Ctl->key_V==1) 
	{
		key_V_cnt++;
		if(key_V_cnt==Key_Filter_Num_vT13)	
		{
//	
			RC_Ctl->key_V_flag=1;
		}			
	}	else
	 {
		key_V_cnt=0;	
	RC_Ctl->key_V_flag=0;
	 } 	 
	//key_ctrl
	if(RC_Ctl->key_ctrl==1) 
	{
		key_ctrl_cnt++;
		if(key_ctrl_cnt==Key_Filter_Num_vT13)	
		{
			RC_Ctl->key_ctrl_flag=1;
			key_ctrl_cnt=0;	
		}	
	} 
	else
	{
		RC_Ctl->key_ctrl_flag=0;
	}	 
	//key_shift 
	 if(RC_Ctl->key_shift==1) 
	{
		key_shift_cnt++;
		if(key_shift_cnt==Key_Filter_Num_vT13)	
		{
			RC_Ctl->key_shift_flag=1;
			key_shift_cnt=0;	
		}	
	 }	
	else
	{
		RC_Ctl->key_shift_flag=0;
	}
	 //mouse_l
	 if(RC_Ctl->mouse.press_l==1)
	 {
		mouse_press_l_cnt++;
		if(mouse_press_l_cnt==Key_Filter_Num_vT13)
		{
			RC_Ctl->mouse.press_l_flag=1;
		}
	 }
	 else
	 {
		 mouse_press_l_cnt=0;
		 RC_Ctl->mouse.press_l_flag=0;
	 }
		 //mouse_r
	 if(RC_Ctl->mouse.press_r==1)
	 {
		mouse_press_r_cnt++;
		if(mouse_press_r_cnt==Key_Filter_Num_vT13)
		{
			RC_Ctl->mouse.press_r_flag++;				
		}
	 }
	 else
	 {
		// RC_Ctl->mouse.press_r_flag=0;
		 mouse_press_r_cnt=0;
	 }
//***************************************************************************	 
 } 
}
	
void KeyBoard_DataUnpack_vt13(VT13 *rc_ctrl)
{
	if(rc_ctrl->rc.mode_sw!=0)
	{
	 if((rc_ctrl->key_W - rc_ctrl->key_S)==0)
	 {
	    if(rc_ctrl->Chassis_Y_Integ>=0)
			{
			  rc_ctrl->Chassis_Y_Integ-=1;
				if(rc_ctrl->Chassis_Y_Integ<0)rc_ctrl->Chassis_Y_Integ=0;
			}
			if(rc_ctrl->Chassis_Y_Integ<0)
			{
			  rc_ctrl->Chassis_Y_Integ+=1;
				if(rc_ctrl->Chassis_Y_Integ>0)rc_ctrl->Chassis_Y_Integ=0;
			}
	 }
	 else
	 {
	    rc_ctrl->Chassis_Y_Integ+=2*(rc_ctrl->key_W-rc_ctrl->key_S);
	    if(rc_ctrl->Chassis_Y_Integ>400)
			{
			   rc_ctrl->Chassis_Y_Integ=400;
			}
			if(rc_ctrl->Chassis_Y_Integ<-400)
			{
			   rc_ctrl->Chassis_Y_Integ=-400;
			}
	 }
	 
	 
	  if(rc_ctrl->key_A-rc_ctrl->key_D==0)
	 {
	    if(rc_ctrl->Chassis_X_Integ>=0)
			{
			  rc_ctrl->Chassis_X_Integ-=1;
				if(rc_ctrl->Chassis_X_Integ<0)rc_ctrl->Chassis_X_Integ=0;
			}
			if(rc_ctrl->Chassis_X_Integ<0)
			{
			  rc_ctrl->Chassis_X_Integ+=1;
				if(rc_ctrl->Chassis_X_Integ>0)rc_ctrl->Chassis_X_Integ=0;
			}
	 }
	 else
	 {
	    rc_ctrl->Chassis_X_Integ+=2*(rc_ctrl->key_D-rc_ctrl->key_A);
	    if(rc_ctrl->Chassis_X_Integ>400)
			{
			   rc_ctrl->Chassis_X_Integ=400;
			}
			if(rc_ctrl->Chassis_X_Integ<-400)
			{
			   rc_ctrl->Chassis_X_Integ=-400;
			}
	 }
 }
		
}