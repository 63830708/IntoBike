/*该头文件引用由IntoRobot自动添加.*/
#include <IntoBike_PWM_OUT/IntoBike_PWM_OUT.h>

void setup() 
{
// put your setup code here, to run once.
   pinMode(D0, OUTPUT);
}

void loop() 
{
// put your main code here, to run repeatedly.
   outputPWM(D0, 100, 13000);
}