
#ifndef __REFEREE_H
#define __REFEREE_H

#include "main.h"
#include "driver_usart.h"

#define data_addr 7
#define max_single_pack_len 50
#define packs 50
#define frame_header_len 5 
#define pack_len 7+referee2024.frame_info.head.data_len+2
#define BSP_USART3_DMA_RX_BUF_LEN	50

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	
typedef struct
{
	struct
	{
		struct
		{
			uint8_t sof;
			uint16_t data_len;
			uint8_t seq;
			uint8_t crc8;
		}head;
	uint16_t cmd_id;
	uint8_t  frame_tail[2];
  }frame_info;

	/*1. 比赛状态数据： 0x0001。 发送频率： 1Hz*/
	struct  
	{
		uint8_t game_type : 4;
		uint8_t game_progress : 4;
		uint16_t stage_remain_time;
		uint64_t SyncTimeStamp;
	} game_status_t;

	/*2. 比赛结果数据： 0x0002。 发送频率：比赛结束后发送*/
	 struct
	{
		uint8_t winner;
	} game_result_t;
	/*3. 机器人血量数据： 0x0003。发送频率： 1Hz*/
	 struct
	{
		uint16_t red_1_robot_HP;
		uint16_t red_2_robot_HP;
		uint16_t red_3_robot_HP;
		uint16_t red_4_robot_HP;
		uint16_t red_5_robot_HP;
		uint16_t red_7_robot_HP;
		uint16_t red_outpost_HP;
		uint16_t red_base_HP;
		uint16_t blue_1_robot_HP;
		uint16_t blue_2_robot_HP;
		uint16_t blue_3_robot_HP;
		uint16_t blue_4_robot_HP;
		uint16_t blue_5_robot_HP;
		uint16_t blue_7_robot_HP;
		uint16_t blue_outpost_HP;
		uint16_t blue_base_HP;
	} game_robot_HP_t;

	/*4. 0x0101       */ 
	struct 
	{
		uint32_t event_data;
	} event_data_t;

	/*5. 0x0102       */
	struct 
	{
		uint8_t reserved;
		uint8_t supply_robot_id;
		uint8_t supply_projectile_step;
		uint8_t supply_projectile_num;
	}ext_supply_projectile_action_t;
	//0x0104
	struct
	{
		uint8_t level;
		uint8_t offending_robot_id;
		uint8_t count;
	}referee_warning_t;
	//0x0105
	struct
	{
		uint8_t dart_remaining_time;
		uint16_t dart_info;
	}dart_info_t;
	//0x0201
	 struct
	{
		uint8_t robot_id;
		uint8_t robot_level;
		uint16_t current_HP; 
		uint16_t maximum_HP;
		uint16_t shooter_barrel_cooling_value;
		uint16_t shooter_barrel_heat_limit;
		uint16_t chassis_power_limit; 
		uint8_t power_management_gimbal_output : 1;
		uint8_t power_management_chassis_output : 1; 
		uint8_t power_management_shooter_output : 1;
	}robot_status_t;
	//0x0202
	 struct
	{
		uint16_t chassis_voltage;
		uint16_t chassis_current;
		float chassis_power;
		uint16_t buffer_energy;
		uint16_t shooter_17mm_1_barrel_heat;
		uint16_t shooter_17mm_2_barrel_heat;
		uint16_t shooter_42mm_barrel_heat;
	}power_heat_data_t;
	//0x0203
	struct
	{
		float x;
		float y;
		float angle;
	}robot_pos_t;
	//0x0204
	struct
	{
		uint8_t recovery_buff;
		uint8_t cooling_buff;
		uint8_t defence_buff;
		uint8_t vulnerability_buff;
		uint16_t attack_buff;
	}buff_t; 
	//0x0205
	struct
	{
		uint8_t airforce_status;
		uint8_t time_remain;
	}air_support_data_t;
	//0x0206
	struct
	{
		uint8_t armor_id : 4;
		uint8_t HP_deduction_reason : 4;
	}hurt_data_t;
	//0x0207
	struct
	{
		uint8_t bullet_type;
		uint8_t shooter_number;
		uint8_t launching_frequency;
		float initial_speed;
	}shoot_data_t;
	//0x0208
	struct
	{
		uint16_t projectile_allowance_17mm;
		uint16_t projectile_allowance_42mm;
		uint16_t remaining_gold_coin;
	}projectile_allowance_t;
	//0x0209
	struct
	{
		uint32_t rfid_status;
	}rfid_status_t;
	//0x020A
	struct
	{
		uint8_t dart_launch_opening_status;
		uint8_t reserved;
		uint16_t target_change_time;
		uint16_t latest_launch_cmd_time;
	}dart_client_cmd_t;
	//0x020B
	struct
	{
		float hero_x;
		float hero_y;
		float engineer_x;
		float engineer_y;
		float standard_3_x;
		float standard_3_y;
		float standard_4_x;
		float standard_4_y;
		float standard_5_x;
		float standard_5_y;
	}ground_robot_position_t;
	//0x020C
	struct
	{
		uint8_t mark_hero_progress;
		uint8_t mark_engineer_progress;
		uint8_t mark_standard_3_progress;
		uint8_t mark_standard_4_progress;
		uint8_t mark_standard_5_progress;
		uint8_t mark_sentry_progress;
	}radar_mark_data_t;
	//0x020D
	struct
	{
		uint32_t sentry_info;
	}  sentry_info_t;
	//0x020E
	struct
	{
		uint8_t radar_info;
	} radar_info_t;

	//0x0301
	struct
	{
		uint16_t data_cmd_id;
		uint16_t sender_id;
		uint16_t receiver_id;
		uint8_t user_data[113];
	}robot_interaction_data_t;

	//0x0120
	struct
	{
		uint32_t sentry_cmd; 
	} sentry_cmd_t;
	//0x0121
	struct
	{
		uint8_t radar_cmd;
	} radar_cmd_t;
	//0x0302
	struct
	{
		uint8_t data[30];
	}custom_robot_data_t;
	//0x0303
	 struct
	{
		float target_position_x;
		float target_position_y;
		uint8_t cmd_keyboard;
		uint8_t target_robot_id;
		uint8_t cmd_source;
	}map_command_t;
	//0x0304
	struct
	{
		uint16_t mouse_x;
		uint16_t mouse_y;
		uint16_t mouse_z;
		uint8_t left_button_down;
		uint8_t right_button_down;
		uint16_t keyboard_value;
		uint16_t reserved;
	}remote_control_t;
	//0x0305
	struct
	{
		uint16_t target_robot_id;
		float target_position_x;
		float target_position_y;
	}map_robot_data_t;
	//0x0306
	struct
	{
		uint16_t key_value;
		uint16_t x_position:12;
		uint16_t mouse_left:4;
		uint16_t y_position:12;
		uint16_t mouse_right:4;
		uint16_t reserved;
	}custom_client_data_t;
	//0x0307
	struct
	{
		uint8_t intention;
		uint16_t start_position_x;
		uint16_t start_position_y;
		int8_t delta_x[49];
		int8_t delta_y[49];
		uint16_t sender_id;
	}map_data_t;
	//0x0308
	struct
	{ 
		uint16_t sender_id;
		uint16_t receiver_id;
		uint8_t user_data[30];
	} custom_info_t;

}Referee2024;


extern Referee2024 referee2024;
extern uint8_t meta_data[BSP_USART3_DMA_RX_BUF_LEN]__attribute__((at(0x24028000)));
extern UART_RxBuffer uart3_buffer;


unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Referee_Data_Diapcak(uint8_t *data,uint8_t this_time_len);
uint8_t Referee_callback(uint8_t * recBuffer, uint16_t len);
void referee_Data_copy(uint8_t *pdata,uint8_t *copy_buff);
void referee_data(void);

#endif
