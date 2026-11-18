#ifndef DRIVER_GATE_SENSOR_H
#define DRIVER_GATE_SENSOR_H

#include "app_common.h"

void GateSensor_HardwareInit(void);
float HCSR04_Read_Entry(void);
float HCSR04_Read_Exit(void);
void Servo_SetAngle_Entry(uint8_t angle);
void Servo_SetAngle_Exit(uint8_t angle);

#endif /* DRIVER_GATE_SENSOR_H */