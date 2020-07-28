
#include "mbed.h"
#include "MPU6050.h"
#include "ledControl.h"

/* */
//Timer timer;
/* Defined in the MPU6050.cpp file  */
// I2C i2c(p9,p10);         // setup i2c (SDA,SCL)  

Serial pc(USBTX,USBRX);    // default baud rate: 9600
MPU6050 mpu6050;           // class: MPU6050, object: mpu6050 
Ticker toggler1;
Ticker filter;           

//Serial pc1(p28,p27);
//DigitalOut myled(LED4);

DigitalOut m1d1(p5);
DigitalOut m1d2(p6);
DigitalOut m2d1(p7);
DigitalOut m2d2(p8);
PwmOut pm1(p21);
PwmOut pm2(p22);   

void toggle_led1();
void toggle_led2();
void compFilter();
float Err = 0;
float Ierr = 0;
float pitchAngle = 0;
float rollAngle = 0;
float P=0;
float kp=0.25;
float I=0;
float output=0;
float sum=0;
float sp = -2.5;
float ki=0;
float kd=0;
float error_prior=0;
float D=0;
float level=0;
float avg=0;
int main() 
{
    pc.baud(9600);
    //pc1.baud(9600);                              // baud rate: 9600
    mpu6050.whoAmI();                           // Communication test: WHO_AM_I register reading 
    wait(1);
    mpu6050.calibrate(accelBias,gyroBias);      // Calibrate MPU6050 and load biases into bias registers
    pc.printf("Calibration is completed. \r\n");
    wait(0.5);
    mpu6050.init();                             // Initialize the sensor
    //wait(1);
    //timer.reset();                      //Reset the timer.
    //timer.start(); 
    pc.printf("MPU6050 is initialized for operation.. \r\n\r\n");
    //wait_ms(500);
    /*for(int i =0; i < 10000;i++)
    {
        filter.attach(&compFilter, 0.005);
        avg = avg + pitchAngle;
    }
    level = avg/10000;
    wait(10);
    */
    while(1) 
    {
     
     /* Uncomment below if you want to see accel and gyro data */
          
//        pc.printf(" _____________________________________________________________  \r\n");
//        pc.printf("| Accelerometer(g) | ax=%.3f | ay=%.3f | az=%.3f                \r\n",ax,ay,az);
//        pc.printf("| Gyroscope(deg/s) | gx=%.3f | gy=%.3f | gz=%.3f                \r\n",gx,gy,gz);
//        pc.printf("|_____________________________________________________________  \r\n\r\n");
//        
//        wait(2.5);
        //char c=pc1.getc();
        //if(c=='a') ledToggle(3);
        //else if (c=='d') ledToggle(3);        
        filter.attach(&compFilter, 0.005);    // Call the complementaryFilter func. every 5 ms (200 Hz sampling period)
        
        //pc.printf(" _______________\r\n");
        pc.printf("| Pitch: %.3f   \r\n",pitchAngle);
        
        //wait(1);
        //pc.printf("| output: %.3f   \r\n",output);
        //pc.printf("| Err: %.3f   \r\n",Err);
        //pc.printf("| Roll:  %.3f   \r\n",rollAngle);
        //pc.printf("|_______________\r\n\r\n");
        //float DeltaT = timer.read();              //Read and store the timer values in variable called 't' ehich will be used for calculating PID outputs. 
        //timer.reset();
        /*
        if(2.5 > pitchAngle > 0)
        {
            Ierr = abs(pitchAngle + sp);
        }
        if(pitchAngle > 2.5)
        {
            Ierr = pitchAngle + sp;
        }
        if(0 > pitchAngle > -2.5)
        {
            Ierr = abs(abs(pitchAngle) + sp);
        }
        if(pitchAngle < -2.5)
        {
            Ierr = pitchAngle + sp;
        }
        */ 
        if(2.5 > pitchAngle > 0)
        {
            Err = abs(pitchAngle - sp);
        }
        if(pitchAngle > 2.5)
        {
            Err = pitchAngle - sp ;
        }    
        if(0 > pitchAngle > -2.5)
        {
            Err = abs(abs(pitchAngle)+sp);
        }
        if(pitchAngle < -2.5)
        {
            Err = pitchAngle - sp;
        }       
        P = kp * Err;
        sum = (sum +(Err)); 
        I = ((ki) * sum);
        D = (kd)*((Err - error_prior));    //calculating differential.
        error_prior = Err;
        output = P+D+I;
        
        if(output > 0.6)
        {
            output = 0.6;
        }
        /*
        if(output > 0.2)
        {
            output = -0.2;
        }
        */
        /*
        if(pitchAngle > 0)
        {
        pm1.write(kp*pitchAngle);
        pm2.write(kp*pitchAngle);
        m1d1 = 1;
        m1d2 = 0;
        m2d1 = 1;
        m2d2 = 0;
        }
        if(pitchAngle < 0)
        {
        pm1.write(kp*abs(pitchAngle));
        pm2.write(kp*abs(pitchAngle)); 
        m1d1 = 0;
        m1d2 = 1;
        m2d1 = 0;
        m2d2 = 1;   
        }
        */
        
        if(pitchAngle > sp);
        {
        pm1.write(output);
        pm2.write(output);
        m1d1 = 1;
        m1d2 = 0;
        m2d1 = 1;
        m2d2 = 0;
        }
        
        if(pitchAngle < sp)
        {
        //rollAngle = -1 *  rollAngle;
        pm1.write(abs(output));
        pm2.write(abs(output)); 
        m1d1 = 0;
        m1d2 = 1;
        m2d1 = 0;
        m2d2 = 1;
        }
        if(pitchAngle == sp)
        {
        pm1.write(output);
        pm2.write(output); 
        m1d1 = 0;
        m1d2 = 0;
        m2d1 = 0;
        m2d2 = 0;      
        }
        /*
        if(pitchAngle > (-3) && pitchAngle < (-1))
        {
        pm1.write(0);
        pm2.write(0); 
        m1d1 = 0;
        m1d2 = 0;
        m2d1 = 0;
        m2d2 = 0;
        }
        */   
    }
}

void toggle_led1() {ledToggle(1);}
void toggle_led2() {ledToggle(2);}

/* This function is created to avoid address error that caused from Ticker.attach func */ 
void compFilter() {mpu6050.complementaryFilter(&pitchAngle, &rollAngle);}
