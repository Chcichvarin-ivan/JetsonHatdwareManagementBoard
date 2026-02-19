//
// Created by chichvarinivan on 2/19/26.
//

#ifndef JETSONEXTHARDWAREMANAGER_I2C_APP_H
#define JETSONEXTHARDWAREMANAGER_I2C_APP_H
/* LED commands written by master */
#define I2C_SLAVE_LED_OFF     0xA0
#define I2C_SLAVE_LED_ON      0xA1
#define I2C_SLAVE_LED_FLASH2H 0xA2
#define I2C_SLAVE_LED_SHTDWN  0xA3
void setLedState(uint8_t in_state);
void I2C_SlaveApp_Init(I2C_HandleTypeDef *hi2c);

#endif //JETSONEXTHARDWAREMANAGER_I2C_APP_H