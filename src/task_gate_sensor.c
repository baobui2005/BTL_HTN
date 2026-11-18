#include "task_gate_sensor.h"
#include "driver_gate_sensor.h"

void MemberA_InitTasks(void) {
    GateSensor_HardwareInit();
    xTaskCreate(TaskSensor, "TaskSensor", 128, NULL, 4, NULL);
    xTaskCreate(TaskGate,   "TaskGate",   128, NULL, 3, NULL);
}

void TaskSensor(void *argument) {
    uint8_t entry_lock = 0, exit_lock = 0;
    ParkingEvent_t evt;

    for (;;) {
        float d_in = HCSR04_Read_Entry();
        if (d_in > 1.0f && d_in < DETECT_DISTANCE_CM) {
            if (!entry_lock) {
                entry_lock = 1;
                evt = EVENT_CAR_ENTER;
                xQueueSend(xEventQueue, &evt, 0);
            }
        } else {
            entry_lock = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));

        float d_out = HCSR04_Read_Exit();
        if (d_out > 1.0f && d_out < DETECT_DISTANCE_CM) {
            if (!exit_lock) {
                exit_lock = 1;
                evt = EVENT_CAR_EXIT;
                xQueueSend(xEventQueue, &evt, 0);
            }
        } else {
            exit_lock = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void TaskGate(void *argument) {
    GateCommand_t cmd;
    Servo_SetAngle_Entry(0);
    Servo_SetAngle_Exit(0);

    for (;;) {
        if (xQueueReceive(xGateQueue, &cmd, portMAX_DELAY) == pdPASS) {
            if (cmd == CMD_OPEN_ENTRY_GATE) {
                Servo_SetAngle_Entry(90);
                vTaskDelay(pdMS_TO_TICKS(GATE_OPEN_TIME_MS));
                Servo_SetAngle_Entry(0);
            } else if (cmd == CMD_OPEN_EXIT_GATE) {
                Servo_SetAngle_Exit(90);
                vTaskDelay(pdMS_TO_TICKS(GATE_OPEN_TIME_MS));
                Servo_SetAngle_Exit(0);
            }
        }
    }
}
