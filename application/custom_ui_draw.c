#include "custom_ui_draw.h"
#include "bsp_usart.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "Engineer_behaviour.h"
#include "referee.h"

uint32_t FONT_SIZE = 15;

/*location*/
#define WIDTH 2
#define SCREEN_WIDTH 1080
#define SCREEN_LENGTH 1920
#define CENTER_X 1300
#define CENTER_Y 820
#define VERTICAL 50
#define HORIZONTAL 90
#define ON_COLOR 0
#define OFF_COLOR 8
//��ǰģʽ
#define CURRENT_MODE_X 450 // SPEED
#define CURRENT_MODE_Y 800
#define CURRENT_MODE_GIMBAL_X 300
#define CURRENT_MODE_GIMBAL_Y 300
//������׼
#define AIM_CATCH_ASS_CENTER_X 960
#define AIM_CATCH_ASS_DX 40
#define AIM_CATCH_ASS_START_Y 700
#define AIM_CATCH_ASS_END_Y 400
#define STATE_START_X 100
#define STATE_START_Y 850
#define STATE_DIFF1 60
#define STATE_DIFF2 40
//ȡ������
#define ORE_PUMP_X (STATE_START_X)
#define ORE_PUMP_Y (STATE_START_Y)
//��������
#define GND_PUMP_X (STATE_START_X)
#define GND_PUMP_Y (STATE_START_Y - STATE_DIFF2)
//��������
#define TOP_AIR_X (STATE_START_X)
#define TOP_AIR_Y (STATE_START_Y - 2 * STATE_DIFF1)
//��Ԯ����
#define AID_AIR_X (STATE_START_X)
#define AID_AIR_Y (STATE_START_Y - 2 * STATE_DIFF1 - STATE_DIFF2)
//��������
#define GANTRY_X (STATE_START_X)
#define GANTRY_Y (STATE_START_Y - 2 * STATE_DIFF1 - 2 * STATE_DIFF2)
//��צλ��
#define CLAW_X 1500
#define CLAW_Y 600
//��Ԯ��λ��
#define CARD_X 1500
#define CARD_Y 700

extern int servo_angle[7];
extern RC_ctrl_t rc_ctrl;
extern int low_speed;
extern int target_can2_205_angle;
extern uint8_t coordinates_control_flag;
//extern int target_can1_angle_07;
extern ext_game_robot_state_t robot_state;
uint16_t card_data=0;
uint16_t claw_data=0;
uint16_t send_id;
uint16_t receive_id;
extern int pump10_flag;
extern uint8_t pump_flag_left;
extern uint8_t pump_flag_right;
/*�������*/
extern UART_HandleTypeDef huart6;
uint8_t seq = 0;
void referee_data_pack_handle(uint8_t sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
    uint8_t tx_buff[MAX_SIZE];
    uint16_t frame_length = frameheader_len + cmd_len + crc_len + len;

    memset(tx_buff, 0, MAX_SIZE); //�洢���ݵ���������

    /*****֡ͷ���*****/
    tx_buff[0] = sof;                                  //����֡��ʼ�ֽ�
    memcpy(&tx_buff[1], (uint8_t *)&len, sizeof(len)); //����֡��data�ĳ���
    tx_buff[3] = seq;                                  //�����
    append_CRC8_check_sum(tx_buff, frameheader_len);   //֡ͷУ��CRC8

    /*****��������*****/
    memcpy(&tx_buff[frameheader_len], (uint8_t *)&cmd_id, cmd_len);

    /*****���ݴ��*****/
    memcpy(&tx_buff[frameheader_len + cmd_len], p_data, len);
    append_CRC16_check_sum(tx_buff, frame_length); //һ֡����У��CRC16
    if (seq == 0xff)
        seq = 0;
    else
        seq++;

    usart6_tx_dma_enable(tx_buff, frame_length);
}

void clear_all_layer(uint16_t sender_id, uint16_t receiver_id)
{

    ext_client_custom_graphic_delete_t init_struct;

    init_struct.header_data_t.data_cmd_id = 0x0100;
    init_struct.header_data_t.receiver_ID = receiver_id;
    init_struct.header_data_t.sender_ID = sender_id;

    init_struct.operate_tpye = 2;
    init_struct.layer = 0;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&init_struct, sizeof(init_struct));
}

void clear_single_layer(uint16_t sender_id, uint16_t receiver_id, uint8_t layer)
{
    ext_client_custom_graphic_delete_t init_struct;

    init_struct.header_data_t.data_cmd_id = 0x0100;
    init_struct.header_data_t.receiver_ID = receiver_id;
    init_struct.header_data_t.sender_ID = sender_id;

    init_struct.operate_tpye = 1;
    init_struct.layer = layer;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&init_struct, sizeof(init_struct));
}

void draw_init_all(uint16_t sender_id, uint16_t receiver_id)
{
    clear_all_layer(sender_id, receiver_id);
    osDelay(100);
    draw_catch_aim_assistant(sender_id, receiver_id, 1); //�Կ�����
    osDelay(100);
    draw_current_mode(sender_id, receiver_id, 1); //�ٶ�
    osDelay(100);
    draw_ore_pump(sender_id, receiver_id, 1); //������
    osDelay(100);
    draw_ground_pump(sender_id, receiver_id, 1); //������
    osDelay(100);
    draw_up_air(sender_id, receiver_id, 1); //������
    osDelay(100);
    draw_assistant_air(sender_id, receiver_id, 1); //������
    osDelay(100);
    draw_dragon_air(sender_id, receiver_id, 1); //����
    osDelay(100);
    draw_state_circle(sender_id, receiver_id, 1); //״̬
    osDelay(100);
    draw_claw_position(sender_id, receiver_id, 1); //��צλ��
    osDelay(100);
    draw_card_position(sender_id, receiver_id, 1); //��Ԯ��λ��
    osDelay(100);
    draw_claw_data(sender_id, receiver_id, 1);
    osDelay(100);
    draw_card_data(sender_id, receiver_id, 1);
    osDelay(100);
    draw_two_border(sender_id,receiver_id,1);
    osDelay(100);
}

void draw_current_mode(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_character_t draw_init;
		uint8_t end_angle_flag;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 0;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 1;
    draw_init.grapic_data_struct.operate_tpye = op_type;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 1;
    draw_init.grapic_data_struct.color = 8;

    draw_init.grapic_data_struct.start_angle = 17; //�ַ���С

    draw_init.grapic_data_struct.width = 3;
    draw_init.grapic_data_struct.start_x = CURRENT_MODE_X;
    draw_init.grapic_data_struct.start_y = CURRENT_MODE_Y;
    draw_init.grapic_data_struct.end_x = 0;
    draw_init.grapic_data_struct.end_y = 0;
    draw_init.grapic_data_struct.radius = 0;
    memset(draw_init.data, 0, sizeof(draw_init.data));

    draw_init.data[0] = 'S';
    draw_init.data[1] = 'P';
    draw_init.data[2] = 'E';
    draw_init.data[3] = 'E';
    draw_init.data[4] = 'D';
    draw_init.data[5] = ':';

    if (low_speed == 0)
    {
        end_angle_flag = 10;
        draw_init.data[6] = 'H';
        draw_init.data[7] = 'I';
        draw_init.data[8] = 'G';
        draw_init.data[9] = 'H';
    }
    else if (low_speed == 1)
    {
        end_angle_flag = 12;
        draw_init.data[6] = 'M';
        draw_init.data[7] = 'E';
        draw_init.data[8] = 'D';
        draw_init.data[9] = 'I';
        draw_init.data[10] = 'U';
        draw_init.data[11] = 'M';
    }
    else if (low_speed == 2)
    {
        end_angle_flag = 9;
        draw_init.data[6] = 'L';
        draw_init.data[7] = 'O';
        draw_init.data[8] = 'W';
    }
    else
    {
        end_angle_flag = 7;
        draw_init.data[6] = ' ';
    }
		
		//������ʾ��ǰ��̨����ģʽ�����ĸ�Ϊ���� ���㿴������ط�ʱ ��ϲ���Ѿ���һ���ɹ���RmEngineerER�� ����һ��Ȱ û��������� ��Ҫ�������
		draw_init.data[end_angle_flag++] = ' ';
		draw_init.data[end_angle_flag++] = 'G';
    draw_init.data[end_angle_flag++] = 'I';
    draw_init.data[end_angle_flag++] = 'M';
	  draw_init.data[end_angle_flag++] = ':';
		draw_init.data[end_angle_flag++]  = 'M';
    draw_init.data[end_angle_flag++]  = 'O';
    draw_init.data[end_angle_flag++]  = 'D';
		draw_init.data[end_angle_flag++]  = 'E';
		
		if(coordinates_control_flag == 0)
		{

				draw_init.data[end_angle_flag++]  = '1';
				draw_init.grapic_data_struct.end_angle = end_angle_flag;
		}
		else if(coordinates_control_flag == 1)
		{
				draw_init.data[end_angle_flag++]  = '2';
				draw_init.grapic_data_struct.end_angle = end_angle_flag;
		}
		else if(coordinates_control_flag == 2)
		{
				draw_init.data[end_angle_flag++]  = '3';
				draw_init.grapic_data_struct.end_angle = end_angle_flag;
		}
		
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_current_mode_state(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_character_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 0;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 2;
    draw_init.grapic_data_struct.operate_tpye = op_type;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 1;
    draw_init.grapic_data_struct.color = 8;

    draw_init.grapic_data_struct.start_angle = FONT_SIZE; //�ַ���С

    draw_init.grapic_data_struct.width = 2;
    draw_init.grapic_data_struct.start_x = CURRENT_MODE_X;
    draw_init.grapic_data_struct.start_y = CURRENT_MODE_Y;
    draw_init.grapic_data_struct.end_x = 0;
    draw_init.grapic_data_struct.end_y = 0;
    draw_init.grapic_data_struct.radius = 0;

    memset(draw_init.data, 0, sizeof(draw_init.data));
    draw_init.grapic_data_struct.end_angle = 10; //�ַ�����
    draw_init.data[0] = 'Z';
    draw_init.data[1] = 'E';
    draw_init.data[2] = 'R';
    draw_init.data[3] = 'O';
    draw_init.data[4] = '_';
    draw_init.data[5] = 'F';
    draw_init.data[6] = 'O';
    draw_init.data[7] = 'R';
    draw_init.data[8] = 'C';
    draw_init.data[9] = 'E';

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_catch_aim_assistant(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_graphic_double_t draw_init;

    draw_init.header_data_t.data_cmd_id = 0x0102;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct[0].graphic_name[0] = 0;
    draw_init.grapic_data_struct[0].graphic_name[1] = 0;
    draw_init.grapic_data_struct[0].graphic_name[2] = 3;

    draw_init.grapic_data_struct[0].operate_tpye = op_type;
    draw_init.grapic_data_struct[0].graphic_tpye = 0;
    draw_init.grapic_data_struct[0].layer = 1;
    draw_init.grapic_data_struct[0].color = 4;
    draw_init.grapic_data_struct[0].start_angle = 0;
    draw_init.grapic_data_struct[0].end_angle = 0;
    draw_init.grapic_data_struct[0].width = 3;
    draw_init.grapic_data_struct[0].start_x = AIM_CATCH_ASS_CENTER_X - AIM_CATCH_ASS_DX;
    draw_init.grapic_data_struct[0].start_y = AIM_CATCH_ASS_START_Y;
    draw_init.grapic_data_struct[0].end_x = AIM_CATCH_ASS_CENTER_X - AIM_CATCH_ASS_DX;
    draw_init.grapic_data_struct[0].end_y = AIM_CATCH_ASS_END_Y;
    draw_init.grapic_data_struct[0].radius = 0;

    draw_init.grapic_data_struct[1].graphic_name[0] = 0;
    draw_init.grapic_data_struct[1].graphic_name[1] = 0;
    draw_init.grapic_data_struct[1].graphic_name[2] = 4; //ͼ����
    //���������ֽڴ������ͼ����������ͼ�������������ж���
    draw_init.grapic_data_struct[1].operate_tpye = op_type; //ͼ�β�����0���ղ�����1�����ӣ�2���޸ģ�3��ɾ����
    draw_init.grapic_data_struct[1].graphic_tpye = 0;       //ͼ�����ͣ�0Ϊֱ�ߣ������Ĳ鿴�û��ֲ�
    draw_init.grapic_data_struct[1].layer = 1;              //ͼ����
    draw_init.grapic_data_struct[1].color = 4;              //��ɫ
    draw_init.grapic_data_struct[1].start_angle = 0;
    draw_init.grapic_data_struct[1].end_angle = 0;
    draw_init.grapic_data_struct[1].width = 3;
    draw_init.grapic_data_struct[1].start_x = AIM_CATCH_ASS_CENTER_X + AIM_CATCH_ASS_DX;
    draw_init.grapic_data_struct[1].start_y = AIM_CATCH_ASS_START_Y;
    draw_init.grapic_data_struct[1].end_x = AIM_CATCH_ASS_CENTER_X + AIM_CATCH_ASS_DX;
    draw_init.grapic_data_struct[1].end_y = AIM_CATCH_ASS_END_Y;
    draw_init.grapic_data_struct[1].radius = 0;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_ore_pump(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_character_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 0;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 5;
    draw_init.grapic_data_struct.operate_tpye = op_type;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 1;
    draw_init.grapic_data_struct.color = 8;

    draw_init.grapic_data_struct.start_angle = FONT_SIZE; //�ַ���С

    draw_init.grapic_data_struct.width = 2;
    draw_init.grapic_data_struct.start_x = ORE_PUMP_X;
    draw_init.grapic_data_struct.start_y = ORE_PUMP_Y;
    draw_init.grapic_data_struct.end_x = 0;
    draw_init.grapic_data_struct.end_y = 0;
    draw_init.grapic_data_struct.radius = 0;
    memset(draw_init.data, 0, sizeof(draw_init.data));

    draw_init.data[0] = 'O';
    draw_init.data[1] = 'R';
    draw_init.data[2] = 'E';
    draw_init.data[3] = ' ';
    draw_init.data[4] = 'P';
    draw_init.data[5] = 'U';
    draw_init.data[6] = 'M';
    draw_init.data[7] = 'P';
    draw_init.data[8] = ':';
    draw_init.grapic_data_struct.end_angle = 9; //�ַ�����

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_ground_pump(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type) //$5
{
    ext_client_custom_character_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 1;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 6;
    draw_init.grapic_data_struct.operate_tpye = op_type;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 1;
    draw_init.grapic_data_struct.color = 8;

    draw_init.grapic_data_struct.start_angle = FONT_SIZE; //�ַ���С

    draw_init.grapic_data_struct.width = 2;
    draw_init.grapic_data_struct.start_x = GND_PUMP_X;
    draw_init.grapic_data_struct.start_y = GND_PUMP_Y;
    draw_init.grapic_data_struct.end_x = 0;
    draw_init.grapic_data_struct.end_y = 0;
    draw_init.grapic_data_struct.radius = 0;
    memset(draw_init.data, 0, sizeof(draw_init.data));

    draw_init.data[0] = 'G';
    draw_init.data[1] = 'N';
    draw_init.data[2] = 'D';
    draw_init.data[3] = ' ';
    draw_init.data[4] = 'P';
    draw_init.data[5] = 'U';
    draw_init.data[6] = 'M';
    draw_init.data[7] = 'P';
    draw_init.data[8] = ':';
    draw_init.grapic_data_struct.end_angle = 9; //�ַ�����

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_state_circle(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_graphic_five_t draw_init;

    draw_init.header_data_t.data_cmd_id = 0x0103;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct[0].graphic_name[0] = 0;
    draw_init.grapic_data_struct[0].graphic_name[1] = 1;
    draw_init.grapic_data_struct[0].graphic_name[2] = 7;

    draw_init.grapic_data_struct[0].operate_tpye = op_type;
    draw_init.grapic_data_struct[0].graphic_tpye = 2;
    draw_init.grapic_data_struct[0].layer = 2;
    draw_init.grapic_data_struct[0].start_angle = 0;
    draw_init.grapic_data_struct[0].end_angle = 0;
    draw_init.grapic_data_struct[0].width = 3;
    draw_init.grapic_data_struct[0].start_x = ORE_PUMP_X + 200;
    draw_init.grapic_data_struct[0].start_y = ORE_PUMP_Y - 10;
    draw_init.grapic_data_struct[0].end_x = 0;
    draw_init.grapic_data_struct[0].end_y = 0;
    draw_init.grapic_data_struct[0].radius = 15;
    if (servo_angle[6] == 20000)
    {
        draw_init.grapic_data_struct[0].color = ON_COLOR;
    }
    else
    {
        draw_init.grapic_data_struct[0].color = OFF_COLOR;
    }

    draw_init.grapic_data_struct[1].graphic_name[0] = 0;
    draw_init.grapic_data_struct[1].graphic_name[1] = 1;
    draw_init.grapic_data_struct[1].graphic_name[2] = 8;

    draw_init.grapic_data_struct[1].operate_tpye = op_type;
    draw_init.grapic_data_struct[1].graphic_tpye = 2;
    draw_init.grapic_data_struct[1].layer = 2;
    draw_init.grapic_data_struct[1].start_angle = 0;
    draw_init.grapic_data_struct[1].end_angle = 0;
    draw_init.grapic_data_struct[1].width = 3;
    draw_init.grapic_data_struct[1].start_x = GND_PUMP_X + 200;
    draw_init.grapic_data_struct[1].start_y = GND_PUMP_Y - 10;
    draw_init.grapic_data_struct[1].end_x = 0;
    draw_init.grapic_data_struct[1].end_y = 0;
    draw_init.grapic_data_struct[1].radius = 15;
    if (servo_angle[3] == 20000)
    {
        draw_init.grapic_data_struct[1].color = ON_COLOR;
    }
    else
    {
        draw_init.grapic_data_struct[1].color = OFF_COLOR;
    }

    draw_init.grapic_data_struct[2].graphic_name[0] = 0;
    draw_init.grapic_data_struct[2].graphic_name[1] = 1;
    draw_init.grapic_data_struct[2].graphic_name[2] = 9;

    draw_init.grapic_data_struct[2].operate_tpye = op_type;
    draw_init.grapic_data_struct[2].graphic_tpye = 2;
    draw_init.grapic_data_struct[2].layer = 2;
    draw_init.grapic_data_struct[2].start_angle = 0;
    draw_init.grapic_data_struct[2].end_angle = 0;
    draw_init.grapic_data_struct[2].width = 3;
    draw_init.grapic_data_struct[2].start_x = TOP_AIR_X + 200;
    draw_init.grapic_data_struct[2].start_y = TOP_AIR_Y - 10;
    draw_init.grapic_data_struct[2].end_x = 0;
    draw_init.grapic_data_struct[2].end_y = 0;
    draw_init.grapic_data_struct[2].radius = 15;

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))
    {
        draw_init.grapic_data_struct[2].color = ON_COLOR;
    }
    else
    {
        draw_init.grapic_data_struct[2].color = OFF_COLOR;
    }

    draw_init.grapic_data_struct[3].graphic_name[0] = 0;
    draw_init.grapic_data_struct[3].graphic_name[1] = 1;
    draw_init.grapic_data_struct[3].graphic_name[2] = 10;

    draw_init.grapic_data_struct[3].operate_tpye = op_type;
    draw_init.grapic_data_struct[3].graphic_tpye = 2;
    draw_init.grapic_data_struct[3].layer = 2;
    draw_init.grapic_data_struct[3].start_angle = 0;
    draw_init.grapic_data_struct[3].end_angle = 0;
    draw_init.grapic_data_struct[3].width = 3;
    draw_init.grapic_data_struct[3].start_x = AID_AIR_X + 200;
    draw_init.grapic_data_struct[3].start_y = AID_AIR_Y - 10;
    draw_init.grapic_data_struct[3].end_x = 0;
    draw_init.grapic_data_struct[3].end_y = 0;
    draw_init.grapic_data_struct[3].radius = 15;

    if (servo_angle[4] == 20000)
    {
        draw_init.grapic_data_struct[3].color = ON_COLOR;
    }
    else
    {
        draw_init.grapic_data_struct[3].color = OFF_COLOR;
    }

    draw_init.grapic_data_struct[4].graphic_name[0] = 0;
    draw_init.grapic_data_struct[4].graphic_name[1] = 1;
    draw_init.grapic_data_struct[4].graphic_name[2] = 11;

    draw_init.grapic_data_struct[4].operate_tpye = op_type;
    draw_init.grapic_data_struct[4].graphic_tpye = 2;
    draw_init.grapic_data_struct[4].layer = 2;
    draw_init.grapic_data_struct[4].start_angle = 0;
    draw_init.grapic_data_struct[4].end_angle = 0;
    draw_init.grapic_data_struct[4].width = 3;
    draw_init.grapic_data_struct[4].start_x = GANTRY_X + 200;
    draw_init.grapic_data_struct[4].start_y = GANTRY_Y - 10;
    draw_init.grapic_data_struct[4].end_x = 0;
    draw_init.grapic_data_struct[4].end_y = 0;
    draw_init.grapic_data_struct[4].radius = 15;

    if (servo_angle[5] == 20000)
    {
        draw_init.grapic_data_struct[4].color = ON_COLOR;
    }
    else
    {
        draw_init.grapic_data_struct[4].color = OFF_COLOR;
    }

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_single_line_test(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_graphic_single_t draw_test;

    draw_test.header_data_t.data_cmd_id = 0x0101;
    draw_test.header_data_t.sender_ID = sender_id;
    draw_test.header_data_t.receiver_ID = receiver_id;

    draw_test.grapic_data_struct.graphic_name[0] = 1;
    draw_test.grapic_data_struct.graphic_name[1] = 0;
    draw_test.grapic_data_struct.graphic_name[2] = 1;

    draw_test.grapic_data_struct.graphic_tpye = 0;
    draw_test.grapic_data_struct.operate_tpye = op_type;
    draw_test.grapic_data_struct.layer = 9;
    draw_test.grapic_data_struct.color = 8;
    draw_test.grapic_data_struct.width = 2;
    draw_test.grapic_data_struct.start_x = 960;
    draw_test.grapic_data_struct.start_y = 500;
    draw_test.grapic_data_struct.end_x = 960 + 200;
    draw_test.grapic_data_struct.end_y = 500 + 200;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_test, sizeof(draw_test));
}


void draw_up_air(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_character_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 1;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 99;
    draw_init.grapic_data_struct.operate_tpye = op_type;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 1;
    draw_init.grapic_data_struct.color = 8;

    draw_init.grapic_data_struct.start_angle = FONT_SIZE; //�ַ���С

    draw_init.grapic_data_struct.width = 2;
    draw_init.grapic_data_struct.start_x = TOP_AIR_X;
    draw_init.grapic_data_struct.start_y = TOP_AIR_Y;
    draw_init.grapic_data_struct.end_x = 0;
    draw_init.grapic_data_struct.end_y = 0;
    draw_init.grapic_data_struct.radius = 0;
    memset(draw_init.data, 0, sizeof(draw_init.data));

    draw_init.data[0] = 'T';
    draw_init.data[1] = 'O';
    draw_init.data[2] = 'P';
    draw_init.data[3] = ' ';
    draw_init.data[4] = 'R';
    draw_init.data[5] = 'O';
    draw_init.data[6] = 'D';
    draw_init.data[7] = ':';

    draw_init.grapic_data_struct.end_angle = 8; //�ַ�����

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_assistant_air(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_character_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 1;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 90;
    draw_init.grapic_data_struct.operate_tpye = op_type;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 1;
    draw_init.grapic_data_struct.color = 8;

    draw_init.grapic_data_struct.start_angle = FONT_SIZE; //�ַ���С

    draw_init.grapic_data_struct.width = 2;
    draw_init.grapic_data_struct.start_x = AID_AIR_X;
    draw_init.grapic_data_struct.start_y = AID_AIR_Y;
    draw_init.grapic_data_struct.end_x = 0;
    draw_init.grapic_data_struct.end_y = 0;
    draw_init.grapic_data_struct.radius = 0;
    memset(draw_init.data, 0, sizeof(draw_init.data));

    draw_init.data[0] = 'A';
    draw_init.data[1] = 'I';
    draw_init.data[2] = 'D';
    draw_init.data[3] = ' ';
    draw_init.data[4] = 'R';
    draw_init.data[5] = 'O';
    draw_init.data[6] = 'D';
    draw_init.data[7] = ':';

    draw_init.grapic_data_struct.end_angle = 8; //�ַ�����

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_dragon_air(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_character_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 0;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 95;
    draw_init.grapic_data_struct.operate_tpye = op_type;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 1;
    draw_init.grapic_data_struct.color = 8;

    draw_init.grapic_data_struct.start_angle = FONT_SIZE; //�ַ���С

    draw_init.grapic_data_struct.width = 2;
    draw_init.grapic_data_struct.start_x = GANTRY_X;
    draw_init.grapic_data_struct.start_y = GANTRY_Y;
    draw_init.grapic_data_struct.end_x = 0;
    draw_init.grapic_data_struct.end_y = 0;
    draw_init.grapic_data_struct.radius = 0;
    memset(draw_init.data, 0, sizeof(draw_init.data));

    draw_init.data[0] = 'G';
    draw_init.data[1] = 'A';
    draw_init.data[2] = 'N';
    draw_init.data[3] = 'T';
    draw_init.data[4] = 'R';
    draw_init.data[5] = 'Y';
    draw_init.data[6] = ':';

    draw_init.grapic_data_struct.end_angle = 7; //�ַ�����

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_claw_position(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_character_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 0;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 18;
    draw_init.grapic_data_struct.operate_tpye = op_type;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 1;
    draw_init.grapic_data_struct.color = 8;

    draw_init.grapic_data_struct.start_angle = FONT_SIZE; //�ַ���С

    draw_init.grapic_data_struct.width = 2;
    draw_init.grapic_data_struct.start_x = CLAW_X;
    draw_init.grapic_data_struct.start_y = CLAW_Y;
    draw_init.grapic_data_struct.end_x = 0;
    draw_init.grapic_data_struct.end_y = 0;
    draw_init.grapic_data_struct.radius = 0;
    memset(draw_init.data, 0, sizeof(draw_init.data));

    draw_init.data[0] = 'C';
    draw_init.data[1] = 'L';
    draw_init.data[2] = 'A';
    draw_init.data[3] = 'W';
    draw_init.data[4] = ' ';
    draw_init.data[5] = 'P';
    draw_init.data[6] = 'O';
    draw_init.data[7] = 'S';
    draw_init.data[8] = ':';

    draw_init.grapic_data_struct.end_angle = 9; //�ַ�����

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_card_position(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_character_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 0;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 19;
    draw_init.grapic_data_struct.operate_tpye = op_type;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 1;
    draw_init.grapic_data_struct.color = 8;

    draw_init.grapic_data_struct.start_angle = FONT_SIZE; //�ַ���С

    draw_init.grapic_data_struct.width = 2;
    draw_init.grapic_data_struct.start_x = CARD_X;
    draw_init.grapic_data_struct.start_y = CARD_Y;
    draw_init.grapic_data_struct.end_x = 0;
    draw_init.grapic_data_struct.end_y = 0;
    draw_init.grapic_data_struct.radius = 0;
    memset(draw_init.data, 0, sizeof(draw_init.data));

    draw_init.data[0] = 'P';
    draw_init.data[1] = 'U';
    draw_init.data[2] = 'M';
    draw_init.data[3] = 'P';
    draw_init.data[4] = ' ';
		
		if(pump_flag_left == 1)
		{
			draw_init.data[5] = 'O';
			draw_init.data[6] = 'P';
			draw_init.data[7] = 'E';
			draw_init.data[8] = 'N';
			
		}
    else if(pump_flag_left == 0)
		{
			draw_init.data[5] = 'O';
			draw_init.data[6] = 'F';
			draw_init.data[7] = 'F';
			draw_init.data[8] = ' ';
		}

    draw_init.grapic_data_struct.end_angle = 9; //�ַ�����

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}
void draw_claw_data(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_graphic_single_t_data init_struct;

    init_struct.header_data_t.data_cmd_id = 0x0101;
    init_struct.header_data_t.sender_ID = sender_id;
    init_struct.header_data_t.receiver_ID = receiver_id;

    init_struct.grapic_data_struct.graphic_name[0] = 0;
    init_struct.grapic_data_struct.graphic_name[1] = 0;
    init_struct.grapic_data_struct.graphic_name[2] = 20;
    init_struct.grapic_data_struct.operate_tpye = op_type;
    init_struct.grapic_data_struct.graphic_tpye = 6;
    init_struct.grapic_data_struct.layer = 1;
    init_struct.grapic_data_struct.color = 4;
    init_struct.grapic_data_struct.start_x = CLAW_X + 180;
    init_struct.grapic_data_struct.start_y = CLAW_Y;

    init_struct.grapic_data_struct.start_angle = 15;
    init_struct.grapic_data_struct.end_angle = 0;
    init_struct.grapic_data_struct.width = 3;

    init_struct.grapic_data_struct.data = claw_data;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&init_struct, sizeof(init_struct));
}

void draw_card_data(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_graphic_single_t_data init_struct;

    init_struct.header_data_t.data_cmd_id = 0x0101;
    init_struct.header_data_t.sender_ID = sender_id;
    init_struct.header_data_t.receiver_ID = receiver_id;

    init_struct.grapic_data_struct.graphic_name[0] = 0;
    init_struct.grapic_data_struct.graphic_name[1] = 0;
    init_struct.grapic_data_struct.graphic_name[2] = 21;
    init_struct.grapic_data_struct.operate_tpye = op_type;
    init_struct.grapic_data_struct.graphic_tpye = 6;
    init_struct.grapic_data_struct.layer = 1;
    init_struct.grapic_data_struct.color = 4;
    init_struct.grapic_data_struct.start_x = CARD_X + 180;
    init_struct.grapic_data_struct.start_y = CARD_Y;

    init_struct.grapic_data_struct.start_angle = 15;
    init_struct.grapic_data_struct.end_angle = 0;
    init_struct.grapic_data_struct.width = 3;

    init_struct.grapic_data_struct.data = card_data;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&init_struct, sizeof(init_struct));
}

void draw_get_robot_id()
{
    switch (robot_state.robot_id)
    {
    case RED_HERO:
        send_id = 1;
        receive_id = 0x0101;
        break;
    case RED_ENGINEER:
        send_id = 2;
        receive_id = 0x0102;
        break;
    case RED_STANDARD_1:
        send_id = 3;
        receive_id = 0x0103;
        break;
    case RED_STANDARD_2:
        send_id = 4;
        receive_id = 0x0104;
        break;
    case RED_STANDARD_3:
        send_id = 5;
        receive_id = 0x0105;
        break;
    case RED_AERIAL:
        send_id = 6;
        receive_id = 0x0106;
        break;
    case BLUE_HERO:
        send_id = 101;
        receive_id = 0x0165;
        break;
    case BLUE_ENGINEER:
        send_id = 102;
        receive_id = 0x0166;
        break;
    case BLUE_STANDARD_1:
        send_id = 103;
        receive_id = 0x0167;
        break;
    case BLUE_STANDARD_2:
        send_id = 104;
        receive_id = 0x0168;
        break;
    case BLUE_STANDARD_3:
        send_id = 105;
        receive_id = 0x0169;
        break;
    case BLUE_AERIAL:
        send_id = 106;
        receive_id = 0x016A;
        break;
    default:
        send_id = 0;
        receive_id = 0;
        break;
    }
}

void draw_two_border(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    int border_mid_x=0;
    int border_dx_short=0;
    int border_dx_long=0;
    int border_start_y=0;
    int border_end_y=0;


    if (pump10_flag == 1)
    {
        border_mid_x=960;
        border_start_y=0;
        border_end_y=170;
        border_dx_short=70;
        border_dx_long=350;
    }
    else 
    {
        border_mid_x=980;
        border_start_y=0;
        border_end_y=300;
        border_dx_short=230;
        border_dx_long=480;
    }

    ext_client_custom_graphic_double_t draw_init;

    draw_init.header_data_t.data_cmd_id = 0x0102;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct[0].graphic_name[0] = 0;
    draw_init.grapic_data_struct[0].graphic_name[1] = 0;
    draw_init.grapic_data_struct[0].graphic_name[2] = 22;

    draw_init.grapic_data_struct[0].operate_tpye = op_type;
    draw_init.grapic_data_struct[0].graphic_tpye = 0;
    draw_init.grapic_data_struct[0].layer = 3;
    draw_init.grapic_data_struct[0].color = 2;
    draw_init.grapic_data_struct[0].start_angle = 0;
    draw_init.grapic_data_struct[0].end_angle = 0;
    draw_init.grapic_data_struct[0].width = 3;

    draw_init.grapic_data_struct[0].start_x = border_mid_x-border_dx_long;
    draw_init.grapic_data_struct[0].start_y = border_start_y;
    draw_init.grapic_data_struct[0].end_x = border_mid_x-border_dx_short;
    draw_init.grapic_data_struct[0].end_y = border_end_y;
    draw_init.grapic_data_struct[0].radius = 0;

    draw_init.grapic_data_struct[1].graphic_name[0] = 0;
    draw_init.grapic_data_struct[1].graphic_name[1] = 0;
    draw_init.grapic_data_struct[1].graphic_name[2] = 23; //ͼ����

    draw_init.grapic_data_struct[1].operate_tpye = op_type; //ͼ�β�����0���ղ�����1�����ӣ�2���޸ģ�3��ɾ����
    draw_init.grapic_data_struct[1].graphic_tpye = 0;       //ͼ�����ͣ�0Ϊֱ�ߣ������Ĳ鿴�û��ֲ�
    draw_init.grapic_data_struct[1].layer = 3;              //ͼ����
    draw_init.grapic_data_struct[1].color = 2;              //��ɫ
    draw_init.grapic_data_struct[1].start_angle = 0;
    draw_init.grapic_data_struct[1].end_angle = 0;
    draw_init.grapic_data_struct[1].width = 3;
    draw_init.grapic_data_struct[1].start_x = border_mid_x+border_dx_long;
    draw_init.grapic_data_struct[1].start_y = border_start_y;
    draw_init.grapic_data_struct[1].end_x = border_mid_x+border_dx_short;
    draw_init.grapic_data_struct[1].end_y = border_end_y;
    draw_init.grapic_data_struct[1].radius = 0;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_cal_card_pos()
{
//    float ratio=(float)(target_can1_angle_07)/65000;
//    card_data=(uint16_t)(400*ratio);
}

void draw_cal_claw_angle()
{
    float ratio=(float)target_can2_205_angle/5600;
    claw_data=(uint16_t)(270*ratio);
}