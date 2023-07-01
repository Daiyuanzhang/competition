#ifndef REFEREE_DISPATCHTASK_H
#define REFEREE_DISPATCHTASK_H
#include "sys.h"
#include "Referee.h"

void DispchRefereeTask(void *parmas);
uint8_t GetEenmyColor(void);
float GetRealPower(void);
float GetPowerBuffer(void);
Referee_Date *GetRefereeDataPoint(void);
void SendDataToClient(void);

uint8_t GetRobotID(void);
void Draw_grahic_char(uint16_t cmd_id, uint16_t data_id, uint16_t tx_id, uint16_t rx_id);
void Draw_grahic_status_change(uint16_t cmd_id, uint16_t data_id, uint16_t tx_id, uint16_t rx_id); //显示改变之后的图形
void Draw_grahic_status(uint16_t cmd_id, uint16_t data_id, uint16_t tx_id, uint16_t rx_id);
void Draw_grahic_char_check(uint16_t cmd_id, uint16_t data_id, uint16_t tx_id, uint16_t rx_id);
u8 *get_official_check_flag(void);

#endif
