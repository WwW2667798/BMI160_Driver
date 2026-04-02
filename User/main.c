#include "bmi160_driver.h"
//#include "usart.h"
#include "Delay.h"

float pitch, roll, yaw;
float pitch1, roll1, yaw1;
int8_t result;

int main(void)
{
//    usart_init(115200);

	result = bmi160_driver_init();
    if (result != 0)
    {
//        printf("BMI160 init failed!\n");
        while(1);
    }

    Mahony_Init();
	Delay_ms(1000);
	
	while(1)
	{
		// bmi160_read_6axis(&sensor_data);

        BMI160_Complementary_Update(&pitch, &roll, &yaw, 0.01f); 
		BMI160_Mahony_Update(&pitch1, &roll1, &yaw1, 0.01f);
		Delay_ms(10);
	}
}
