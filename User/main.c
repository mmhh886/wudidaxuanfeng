#include "stm32f10x.h"  // Device header
#include "OLED.h"
#include "Delay.h"
#include "Motor.h"
#include "Key.h"
#include "Encoder.h"
#include "pid.h"
#include "Timer.h"
#include "Serial.h"
#include "gray_track.h"
#include "MPU6050.h"
#include "MadgwickAHRS.h"

/*---------------------宏定义---------------------*/
#define LEFT 1
#define RIGHT 2

/*---------------------按键定义---------------------*/
#define ADDKey 2
#define DECKey 1
#define SETKey 3

uint8_t KeyNum;		//定义用于接收按键键码的变量
uint8_t KeyNumMenu=1;		//定义用于接收按键页码
uint8_t KeyNumMenuTotal=2;		//！！！！！！定义用于接收按键页码的总和！！！！！
uint8_t MenuFlag=0;        //定义菜单标志,0为主菜单,1为子菜单

uint8_t LastKeyNumMenu=1;		//定义用于上一次接收按键页码
uint8_t LastMenuFlag=0;        //上一次定义菜单标志,0为主菜单,1为子菜单

/*---------------------速度定义---------------------*/
int16_t Speed;		//定义速度变量

Encoder ecd_left;
Encoder ecd_right;

PID vec_left;
PID vec_right;

int flag = 1;
float index_yaw = 180;

void targetYaw(float target){
    float yaw = getYaw();
    float error_yaw = target - yaw;
    if(error_yaw>=270)
        error_yaw = -yaw - (360-target);
    else if(error_yaw<=-270)
        error_yaw = target + 360 - yaw;
    if(error_yaw>1){
        motor_target_set(40,0);
    }else if(error_yaw<-1){
        motor_target_set(0,40);
    } else {
        motor_target_set(100, 100);
    }
}

float changeYaw(float yaw){
    float index = getYaw()+yaw;
    if(index>=360){
        index-= 360;
    }else if(index<0){
        index += 360;
    }
    return index;
}

/*---------------------小车运行---------------------*/
int carFlag = 1;
double angle = 38.65980825;

void runX(void){
    if(carFlag == 1){
        targetYaw(180.0 + angle);
        if(D1 == 0||D2 == 0||D3 == 0||D4 == 0||D5 == 0||D6 == 0||D7 == 0||D8 == 0){
            carFlag = 2;
        }
    }else if(carFlag == 2){
        track();
        if(D1&D2&D3&D4&D5&D6&D7&D8){
            carFlag = 3;
        }
    }else if(carFlag == 3) {
        targetYaw(360 - angle);
        if (D1 == 0 || D2 == 0 || D3 == 0 || D4 == 0 || D5 == 0 || D6 == 0 || D7 == 0 || D8 == 0) {
            carFlag = 4;
        }
    }else if(carFlag == 4){
        track();
        if(D1&D2&D3&D4&D5&D6&D7&D8){
            carFlag = 1;
        }
    }
}
/*---------------------MPU6050变量定义---------------------*/
int16_t AX, AY, AZ, GX, GY, GZ;
uint64_t millis;                        //millis为当前程序运行的时间，类似arduino里millis()函数

uint8_t ID;								//定义用于存放ID号的变量

MPU6050Params mpu6050 = {
        .MPU6050dt = 10,
        .preMillis = 0,
        .MPU6050ERROE = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
};

SensorMsg msg = {
        .A = {0.0f, 0.0f, 0.0f},
        .G = {0.0f, 0.0f, 0.0f}
};
/*---------------------初始化函数---------------------*/
void myCarControlCodeInit(){
    Parameter param = {4, 28, 13, 0.065};
    initEncoder(&ecd_left,param);
    initEncoder(&ecd_right,param);
    //vec pid init
    initPID(&vec_left, 2100, 5000);
    setPIDParam(&vec_left, 7.5, 0.7,0.4);
    setPIDTarget(&vec_left, 0);

    initPID(&vec_right, 2100, 5000);
    setPIDParam(&vec_right,7.5, 0.7,0.4);
    setPIDTarget(&vec_right, 0);

}
/*---------------------主函数---------------------*/

int main(void) {
    /*模块初始化*/
    OLED_Init();
    begin(1000.0f / (float)mpu6050.MPU6050dt);

	Motor_Init();
//	Delay_ms(1000);
    MPU6050_Init();
//	Delay_ms(1000);
    Encoder_Init_TIM_All();
    Timer_Init();//定时器初始化
    TIM8_Init();

	Delay_ms(5000);
    dataGetERROR();
	Key_Init();
	Serial_Init();
	gray_init();
	
    /*变量初始化*/
    myCarControlCodeInit();
	//Delay_ms(1000);

    while (1) {
		
		Motor_SetSpeedA(vec_left.output);
        Motor_SetSpeedB(vec_right.output);
        //更新速度

        if(millis - mpu6050.preMillis >= mpu6050.MPU6050dt) {
            mpu6050.preMillis = millis;
            dataGetAndFilter();		                            //获取MPU6050的数据
            updateIMU(msg.G[0], msg.G[1], msg.G[2], msg.A[0], msg.A[1], msg.A[2]);
        }//姿态角更新

        if (LastKeyNumMenu!=KeyNumMenu||LastMenuFlag!=MenuFlag){
            OLED_Clear();
            LastKeyNumMenu=KeyNumMenu;
            LastMenuFlag=MenuFlag;
        }
		


        KeyNum = Key_GetNum();				//获取按键键码

        if (KeyNum==ADDKey && KeyNumMenu<KeyNumMenuTotal && MenuFlag==0)   KeyNumMenu++;//菜单页码按键加
        if (KeyNum==DECKey && KeyNumMenu>1 && MenuFlag==0)   KeyNumMenu--;//菜单页码按键减
        if (KeyNum==SETKey)  MenuFlag=!MenuFlag;


        /*显示菜单*/
        if (MenuFlag==0)
        {
            if (KeyNumMenu==1)
            {
                OLED_ShowString(1,1,"X Speed");//1 Speed
                OLED_ShowString(2,1,"2 Test");//2 Menu2
//                OLED_ShowString(1,5,"3 Menu3");//3 Menu3
//                OLED_ShowString(1,7,"4 Menu4");//4 Menu4
            }
            if (KeyNumMenu==2)
            {
                OLED_ShowString(1,1,"1 Speed");//1 Speed
                OLED_ShowString(2,1,"X Test");//2 Menu2
//                OLED_ShowString(1,5,"3 Menu3");//3 Menu3
//                OLED_ShowString(1,7,"4 Menu4");//4 Menu4
            }
        }

        /*菜单操作*/
        if (MenuFlag==1){

            /*  速度操作 */
            if (KeyNumMenu==1)
            {
                OLED_ShowString(1,1,"Speed");//1 Speed

                OLED_ShowSignedNum(3,1,(int32_t)ecd_left.counter.count_increment,8);//
                OLED_ShowSignedNum(4,1,(int32_t)ecd_right.counter.count_increment,8);//

                track();
//                Serial_Printf("Speed:%lld,",ecd_left.counter.count_increment);
//				Serial_Printf("%lld\n",ecd_right.counter.count_increment);

            }



            /*  Test */
            if (KeyNumMenu==2)//目前测试的进程
            {

                OLED_ShowNum(1, 1, millis, 7);
                OLED_ShowNum(2, 1, getYaw(), 3);
                runX();
            }
	    }
    }//while函数的大括号
}//main函数的大括号

/*---------------------定时器中断函数---------------------*/
void TIM1_UP_IRQHandler(void)//100ms
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        updateEncoderLoopSimpleVersion(&ecd_left, 100, TIM2);
        updateEncoderLoopSimpleVersion(&ecd_right, 100, TIM4);
		


        updatePID(&vec_left, ecd_left.counter.count_increment);
        updatePID(&vec_right, ecd_right.counter.count_increment);

        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}//速度计时器中断服务函数

void TIM8_UP_IRQHandler(void)//1ms
{
    if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)
    {
        millis++;
        // 清除中断标志位
        TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
    }
}

/*---------------------MPU6050数据获取函数---------------------*/
void dataGetERROR() {
    for(uint8_t i = 0; i < 100; ++i) {
        getMPU6050Data();
        mpu6050.MPU6050ERROE[0] += msg.A[0];
        mpu6050.MPU6050ERROE[1] += msg.A[1];
        mpu6050.MPU6050ERROE[2] += msg.A[2] - 9.8;
        mpu6050.MPU6050ERROE[3] += msg.G[0];
        mpu6050.MPU6050ERROE[4] += msg.G[1];
        mpu6050.MPU6050ERROE[5] += msg.G[2];
        Delay_ms(10);
    }
    for(uint8_t i = 0; i < 6; ++i) {
        mpu6050.MPU6050ERROE[i] /= 100.0f;
    }
}

void getMPU6050Data() {
    MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);		//获取MPU6050的数据
    msg.A[0] = (float)((float)AX / (float)32768) * 16 * 9.8;
    msg.A[1] = (float)((float)AY / (float)32768) * 16 * 9.8;
    msg.A[2] = (float)((float)AZ / (float)32768) * 16 * 9.8;
    msg.G[0] = (float)((float)GX / (float)32768) * 2000 * 3.5/1.2444444444;
    msg.G[1] = (float)((float)GY / (float)32768) * 2000 * 3.5/1.2444444444;
    msg.G[2] = (float)((float)GZ / (float)32768) * 2000 * 3.5/1.2444444444;

}

void dataGetAndFilter() {
    getMPU6050Data();
    msg.A[0] -= mpu6050.MPU6050ERROE[0];
    msg.A[1] -= mpu6050.MPU6050ERROE[1];
    msg.A[2] -= mpu6050.MPU6050ERROE[2];
    msg.G[0] -= mpu6050.MPU6050ERROE[3];
    msg.G[1] -= mpu6050.MPU6050ERROE[4];
    msg.G[2] -= mpu6050.MPU6050ERROE[5];
}




