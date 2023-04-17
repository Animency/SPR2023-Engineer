#include "servo_task.h"
#include "cmsis_os.h"

void servo_task(void const *pvParameters)
{

    while (1)
    {

        vTaskDelay(20);
    }
}
