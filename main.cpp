//include libraries in cpp file
#include "mbed.h"
#include "MPU6050.h"
#include <math.h>
#include "myPwmOut.h"

#define M_PI           3.14159265358979323846
#define SERVO_MIN_PULSEWIDTH 0.01 // Minimum pulse width for MG90S servos
#define SERVO_MAX_PULSEWIDTH 0.02 // Maximum pulse width for MG90S servos

Timer t;

//creating an object of serial class
//so that we can communicate with PC
//Serial pc(SERIAL_TX, SERIAL_RX);
//setting LED1 to give digital output
DigitalOut myled(LED1);
//creating onject of MPU6050 class
MPU6050 mpu(PB_9,PB_8);
PwmOut servoA(D11);
PwmOut servoB(D10);

float elapsedTime, currentTime, previousTime;
float kp = 1;
float ki = 1;
float kd = 1;
float dutyCycleA, dutyCycleB;
float angleX, angleY, angleZ, accAngleX, accAngleY, accelX, accelY, accelZ;;
float valueX = 110;
float valueY = 150;
float angleXSample1, angleXSample2, angleXSample3, angleXAvg;
float angleYSample1, angleYSample2, angleYSample3, angleYAvg;
float angleZSample1, angleZSample2, angleZSample3, angleZAvg;

float rad_to_deg(float rad) {
  return rad * 180.0 / M_PI;
}

void initServo()
{
    servoA.period_ms(PERIOD);

    return;
}

void runServo(float angleXAvg, float angleYAvg)
{
    //TVC Startup//
    dutyCycleA = (0.0006 * angleYAvg) + 0.075; //using equation from Angle vs. Duty Cycle graph
    dutyCycleB = (0.0006 * angleYAvg) + 0.075;
    servoB.write(dutyCycleB);
    servoA.write(dutyCycleA);
    printf("duty cycle A: %f    | duty cycle B: %f\n", dutyCycleA, dutyCycleB);
    wait_us(WAIT);
}

int main()
{
    initServo();
    t.start();

    while(1) {

        //setup, test connection with gyro and stop program if bad connection
        mpu.setSleepMode(false);
        bool test = mpu.testConnection();
        if (test == true)
        {
            printf("Connection: Good\n");
        }
        else if (test == false) {
            printf("Connection: Bad\n");
            return(0);
        }

        mpu.setGyroRange(0);
        mpu.setAcceleroRange(0);
        
        previousTime = currentTime;
        currentTime = t.read_ms();
        elapsedTime = (currentTime - previousTime) / 1000;

        float temp = mpu.getTemp();
        printf("temprature = %0.2f ^C\r\n",temp);

        float gyro[3];
        //Collecting gyro angles three times and averaging them for more accurate results
        for (int i = 0; i < 2; i++)
        {
            mpu.getGyro(gyro);
            angleX = rad_to_deg(gyro[0]) * elapsedTime;
            angleY = rad_to_deg(gyro[1]) * elapsedTime;
            angleZ = rad_to_deg(gyro[2]) * elapsedTime;

            if (i == 0)
            {
                angleXSample1 = angleX;
                angleYSample1 = angleY;
                angleZSample1 = angleZ;
            }
            else if (i == 1)
            {
                angleXSample2 = angleX;
                angleYSample2 = angleY;
                angleZSample2 = angleZ;
            }
            else if (i == 2)
            {
                angleXSample3 = angleX;
                angleYSample3 = angleY;
                angleZSample3 = angleZ;
            }
        }

        angleXAvg = (angleXSample1 + angleXSample2 + angleXSample3) / 3;
        angleYAvg = (angleYSample1 + angleYSample2 + angleYSample3) / 3;
        angleZAvg = (angleZSample1 + angleZSample2 + angleZSample3) / 3;
        
        printf("Angle X: %f     | Angle Y: %f   | Angle Z: %f\n", angleXAvg, angleYAvg, angleZAvg);
        runServo(angleXAvg, angleYAvg);

        //accelerometer data
        float acce[3];
        mpu.getAccelero(acce);
        accelX = rad_to_deg(acce[0]);
        accelY = rad_to_deg(acce[1]);
        accelZ = rad_to_deg(acce[2]);
        printf("AccelX=%f,  AccelY=%f,  AccelZ=%f  (m/s^2)\r\n",acce[0],acce[1],acce[2]);
        
        wait_us(2e6); //wait 1000ms
    }
}