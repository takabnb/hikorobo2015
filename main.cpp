#include "mbed.h"
#include "MPU6050.h"
#include "ledControl.h"

#define chan    5         // the numbers of channels

Serial pc(USBTX,USBRX);    // default baud rate: 9600

DigitalOut ex_led(p29);

Ticker toggler1;

PwmOut esc1(p21);
PwmOut esc2(p22);
PwmOut esc3(p23);
PwmOut esc4(p24);

void toggle_led1();
void toggle_led2();

Timer t_main;

//////////////////////////////////////////////////
// MPU6050
//////////////////////////////////////////////////

MPU6050 mpu6050;           // class: MPU6050, object: mpu6050 

void compFilter();

double pitch = 0;
double roll  = 0;

//////////////////////////////////////////////////
// Read reciever
//////////////////////////////////////////////////

InterruptIn ch1(p15);       // yaw
InterruptIn ch2(p16);       // pitch
InterruptIn ch3(p17);       // throttle
InterruptIn ch4(p18);       // roll
InterruptIn ch5(p13);       // aux1
//InterruptIn ch6(p15);       // aux2

void ch1_rise();    void ch1_fall();
void ch2_rise();    void ch2_fall();
void ch3_rise();    void ch3_fall();
void ch4_rise();    void ch4_fall();
void ch5_rise();    void ch5_fall();
//void ch6_rise();    void ch6_fall();

Timer t_ch1;
Timer t_ch2;
Timer t_ch3;
Timer t_ch4;
Timer t_ch5;
//Timer t_ch6;

double pwm_in_ch1      =   0;
double pwm_in_ch2      =   0;
double pwm_in_ch3      =   0;
double pwm_in_ch4      =   0;
double pwm_in_ch5      =   0;
//double pwm_in_ch6      =   0;

int main()
{
    
    int flag_mode   =   0;  // 0: disarmed, 1: armed, 2: auto
    
    int count_print  =   0;
    
    double pwm_in_mod_ch1   =   0;
    double pwm_in_mod_ch2   =   0;
    double pwm_in_mod_ch3   =   0;
    double pwm_in_mod_ch4   =   0;
    
    //double pwm_in_cen_ch1   =   0;
    //double pwm_in_cen_ch2   =   0;
    double pwm_in_low_ch3   =   0;
    //double pwm_in_cen_ch4   =   0;
    
    //double pwm_in_trim_ch1;
    //double pwm_in_trim_ch2;
    double pwm_in_trim_ch3;
    //double pwm_in_trim_ch4;
    
    double input_phi    =   0;
    double input_the    =   0;
    double input_ome    =   0;
    
    // Gains
    double GAIN_P   =   0.0000003;
    double GAIN_Q   =   0.0000003;
    double GAIN_R   =   0.0000026;
    
    double GAIN_PHI   =   0.00000213;
    double GAIN_THE   =   0.00000213;
    
    double GAIN_PWM2ANGLE  =   25 / 0.0004;    // Max input -> 30 [deg] (0.0011~0.0019 [s], center: 0.0015 [s])
    
    double GAIN_OME =   1.2;
    
    //
    double PWM_OUT_MIN  =   0.0011;
    double pwm_out_ch1  =   PWM_OUT_MIN;
    double pwm_out_ch2  =   PWM_OUT_MIN;
    double pwm_out_ch3  =   PWM_OUT_MIN;
    double pwm_out_ch4  =   PWM_OUT_MIN;
    
    // Rocking wings
    double pwm_in_mod_ch3_prev  =   pwm_out_ch3;
    
    int count_rw     =   0;
    int flag_rw_on   =   0;
    int flag_rw_off  =   0;
    
    double phi_target_rw    =   0;
    double roll_target_rw   =   0;
    
    double DT_RW_1    =   0.3;     // For 1st tilt
    double DT_RW_2    =   0.3;     // For stable
    double DT_RW_3    =   0.3;     // For 2nd tilt
    
    int COUNT_1_RW    =   DT_RW_1 * 200;              // 200 Hz
    int COUNT_2_RW    =   COUNT_1_RW + DT_RW_2 * 200;
    int COUNT_3_RW    =   COUNT_2_RW + DT_RW_3 * 200;
    int COUNT_4_RW    =   COUNT_3_RW + DT_RW_2 * 200;
    
    double TARGET_ANGLE_RW_1 =   15;    // For 1st tilt
    double TARGET_ANGLE_RW_2 =   5;     // for 2nd tilt
    
    // Set baud rate for PC
    pc.baud(115200);                            // baud rate: 115200
    
    // Set reciever function
    ch1.rise(&ch1_rise);    ch1.fall(&ch1_fall);
    ch2.rise(&ch2_rise);    ch2.fall(&ch2_fall);
    ch3.rise(&ch3_rise);    ch3.fall(&ch3_fall);
    ch4.rise(&ch4_rise);    ch4.fall(&ch4_fall);
    ch5.rise(&ch5_rise);    ch5.fall(&ch5_fall);
    //ch6.rise(&ch6_rise);   ch6.fall(&ch6_fall);
    
    // Set ESC period
    esc1.period(0.020);         // servo requires a 20ms period
    esc2.period(0.020);         // servo requires a 20ms period
    esc3.period(0.020);         // servo requires a 20ms period
    esc4.period(0.020);         // servo requires a 20ms period
    wait(1);
    
    led4    =   1;
    esc1.pulsewidth(0);
    esc2.pulsewidth(0);
    esc3.pulsewidth(0);
    esc4.pulsewidth(0);
    wait(3);
    led4    =   0;
    
    // Set MPU6050
    i2c.frequency(400000);                      // fast i2c: 400 kHz
    wait(1);
    mpu6050.calibrate(accelBias,gyroBias);      // Calibrate MPU6050 and load biases into bias registers
    pc.printf("\r\nCalibration is completed. \r\n");
    wait(0.5);
    mpu6050.init();                             // Initialize the sensor
    wait(1);
    pc.printf("MPU6050 is initialized for operation.. \r\n\r\n");
    wait_ms(500);
    
    //////////////////////////////////////////////////
    // Preprocessing
    //////////////////////////////////////////////////
    // Get trim reciever
    for(int i_trim_rc=0; i_trim_rc<10; i_trim_rc++)
    {
        wait(.1);
        //pwm_in_cen_ch1  +=   pwm_in_ch1;
        //pwm_in_cen_ch2  +=   pwm_in_ch2;
        pwm_in_low_ch3  +=   pwm_in_ch3;
        //pwm_in_cen_ch4  +=   pwm_in_ch4;
        //pc.printf("%f %f %f %f\r\n", pwm_in_ch1, pwm_in_ch2, pwm_in_ch3, pwm_in_ch4);
    }
    //pwm_in_cen_ch1  =   pwm_in_cen_ch1 / 10;
    //pwm_in_cen_ch2  =   pwm_in_cen_ch2 / 10;
    pwm_in_low_ch3  =   pwm_in_low_ch3 / 10;
    //pwm_in_cen_ch4  =   pwm_in_cen_ch4 / 10;
    
    //pwm_in_trim_ch1    =   pwm_in_cen_ch1 - 0.0015;
    //pwm_in_trim_ch2    =   pwm_in_cen_ch2 - 0.0015;
    pwm_in_trim_ch3    =   pwm_in_low_ch3 - 0.0011;
    //pwm_in_trim_ch4    =   pwm_in_cen_ch4 - 0.0015;
    
    //pc.printf("%f %f %f %f\r\n", pwm_in_cen_ch1, pwm_in_cen_ch2, pwm_in_low_ch3, pwm_in_cen_ch4);
    
    // Prepare ESC
    led3    =   1;
    esc1.pulsewidth(PWM_OUT_MIN);
    esc2.pulsewidth(PWM_OUT_MIN);
    esc3.pulsewidth(PWM_OUT_MIN);
    esc4.pulsewidth(PWM_OUT_MIN);
    wait(5);
    led3    =   0;
    
    // LED for auto
    ex_led  =   0;
    
    //double dt_check =   0;
    //t_main.start();
    
    // Main loop
    while(1)
    {
        
        if((0.00165<pwm_in_ch5 || flag_rw_on==1) && flag_rw_off==0)
        {
            flag_mode   =   2;
            flag_rw_on  =   1;
            ex_led      =   1;
        }
        else if(0.00165<pwm_in_ch5 && flag_rw_off==1) { flag_mode   =   1;  }
        else if(0.00135<pwm_in_ch5 && pwm_in_ch5<0.00165)
        {
            flag_mode   =   1;
            flag_rw_off =   0;
        }
        else { flag_mode   =   0;  }
        
        // Update sensors
        mpu6050.complementaryFilter(&pitch, &roll);
        
        //
        pwm_in_mod_ch1  =   pwm_in_ch1;
        pwm_in_mod_ch2  =   pwm_in_ch2;
        pwm_in_mod_ch3  =   pwm_in_ch3 - pwm_in_trim_ch3;
        pwm_in_mod_ch4  =   pwm_in_ch4;
        
        switch(flag_mode)
        {
            case 0:
                
                esc1.pulsewidth(PWM_OUT_MIN);
                esc2.pulsewidth(PWM_OUT_MIN);
                esc3.pulsewidth(PWM_OUT_MIN);
                esc4.pulsewidth(PWM_OUT_MIN);
                
                wait(0.004468);
                
                break;
                
            case 1:
                
                // Control law (ch1: yaw, ch2: pitch, ch3: thr, ch4: roll)
                input_phi =   - GAIN_P * gx + GAIN_PHI * (GAIN_PWM2ANGLE*(pwm_in_mod_ch2-0.0015) - pitch);
                input_the =   - GAIN_Q * gy + GAIN_THE * (GAIN_PWM2ANGLE*(pwm_in_mod_ch4-0.0015) + roll);
                input_ome =   - GAIN_R * gz + GAIN_OME * (pwm_in_mod_ch1-0.0015);
                
                // ch1: left_front, ch3: right_front, ch4: left_rear, ch2: right_rear 
                pwm_out_ch1  =   pwm_in_mod_ch3 + input_phi + input_the + input_ome * 0.95;
                pwm_out_ch3  =   pwm_in_mod_ch3 + input_phi - input_the - input_ome;
                pwm_out_ch4  =   pwm_in_mod_ch3 - input_phi + input_the - input_ome;
                pwm_out_ch2  =   pwm_in_mod_ch3 - input_phi - input_the + input_ome * 0.95;
                
                // Limit pwm out
                if(pwm_out_ch1<0.00121){ pwm_out_ch1 =   0.00121; }
                if(pwm_out_ch2<0.00121){ pwm_out_ch2 =   0.00121; }
                if(pwm_out_ch3<0.00121){ pwm_out_ch3 =   0.00121; }
                if(pwm_out_ch4<0.00121){ pwm_out_ch4 =   0.00121; }
                
                if(0.0019<pwm_out_ch1){ pwm_out_ch1 =   0.0019; }
                if(0.0019<pwm_out_ch2){ pwm_out_ch2 =   0.0019; }
                if(0.0019<pwm_out_ch3){ pwm_out_ch3 =   0.0019; }
                if(0.0019<pwm_out_ch4){ pwm_out_ch4 =   0.0019; }
                
                // Set pwm out (1-2ms)
                esc1.pulsewidth(pwm_out_ch1);
                esc2.pulsewidth(pwm_out_ch2);
                esc3.pulsewidth(pwm_out_ch3);
                esc4.pulsewidth(pwm_out_ch4);
                
                wait(0.004436);
                
                break;
                
            case 2:
                
                // Inputs
                if(count_rw<COUNT_1_RW) {  roll_target_rw  =   - TARGET_ANGLE_RW_1;   }
                else if(COUNT_1_RW<count_rw && count_rw<COUNT_2_RW) {  roll_target_rw  =   0; }
                else if(COUNT_2_RW<=count_rw && count_rw<COUNT_3_RW) {  roll_target_rw  =   TARGET_ANGLE_RW_1;  }
                else if(COUNT_3_RW<=count_rw && count_rw<COUNT_4_RW) {  roll_target_rw  =   -TARGET_ANGLE_RW_2; }
                
                // Control law (ch1: yaw, ch2: pitch, ch3: thr, ch4: roll)
                input_phi =   - GAIN_P * gx + GAIN_PHI * (phi_target_rw - pitch);
                input_the =   - GAIN_Q * gy + GAIN_THE * (roll_target_rw + roll);
                input_ome =   - GAIN_R * gz + GAIN_OME * (pwm_in_mod_ch1-0.0015);
                
                // ch1: left_front, ch3: right_front, ch4: left_rear, ch2: right_rear 
                pwm_out_ch1  =   pwm_in_mod_ch3_prev + input_phi + input_the + input_ome * 0.95;
                pwm_out_ch3  =   pwm_in_mod_ch3_prev + input_phi - input_the - input_ome;
                pwm_out_ch4  =   pwm_in_mod_ch3_prev - input_phi + input_the - input_ome;           
                pwm_out_ch2  =   pwm_in_mod_ch3_prev - input_phi - input_the + input_ome * 0.95;
                
                // Limit pwm out
                if(pwm_out_ch1<0.00121){ pwm_out_ch1 =   0.00121; }
                if(pwm_out_ch2<0.00121){ pwm_out_ch2 =   0.00121; }
                if(pwm_out_ch3<0.00121){ pwm_out_ch3 =   0.00121; }
                if(pwm_out_ch4<0.00121){ pwm_out_ch4 =   0.00121; }
                
                if(0.0019<pwm_out_ch1){ pwm_out_ch1 =   0.0019; }
                if(0.0019<pwm_out_ch2){ pwm_out_ch2 =   0.0019; }
                if(0.0019<pwm_out_ch3){ pwm_out_ch3 =   0.0019; }
                if(0.0019<pwm_out_ch4){ pwm_out_ch4 =   0.0019; }
                
                // Set pwm out (1-2ms)
                esc1.pulsewidth(pwm_out_ch1);
                esc2.pulsewidth(pwm_out_ch2);
                esc3.pulsewidth(pwm_out_ch3);
                esc4.pulsewidth(pwm_out_ch4);
                
                count_rw++;
                
                wait(0.004445);
                
                if(COUNT_3_RW<=count_rw)
                {
                    flag_rw_on  =   0;
                    flag_rw_off =   1;
                    count_rw    =   0;
                    ex_led      =   0;
                }
                
                break;
                
            default:
                
                led1    =   1;                  led2    =   1;                  led3    =   1;                  led4    =   1;
                esc1.pulsewidth(PWM_OUT_MIN);   esc2.pulsewidth(PWM_OUT_MIN);   esc3.pulsewidth(PWM_OUT_MIN);   esc4.pulsewidth(PWM_OUT_MIN);
                
            }
            
            pwm_in_mod_ch3_prev  =   pwm_in_mod_ch3;
            /*
            if(200<count_print)
            {
                //pc.printf("%9.6f, %9.6f, %9.6f, %9.6f\r\n", pwm_in_mod_ch1, pwm_in_mod_ch2, pwm_in_mod_ch3, pwm_in_mod_ch4);
                //pc.printf("%7.3f, %7.3f, %7.3f, %7.3f, %7.3f\r\n", gx, gy, gz, pitch, roll);
                //pc.printf("%d, %3.0f, %7.3f, %7.3f, %13.10f, %13.10f\r\n", flag_mode, roll_target_rw, pitch, roll, input_phi, input_the);
                pc.printf("%d, %3.0f, %4.0f, %4.0f, %4.0f, %4.0f\r\n", flag_mode, roll_target_rw, pwm_out_ch1*1000000, pwm_out_ch2*1000000, pwm_out_ch3*1000000, pwm_out_ch4*1000000);
                
                //t_main.stop();
                //dt_check    =   t_main.read();
                //pc.printf("%8.6f\r\n", dt_check);
                //t_main.reset();
                //t_main.start();
                
                count_print =   0;
            }
            count_print++;
            */
    }
}

void toggle_led1() {ledToggle(1);}
void toggle_led2() {ledToggle(2);}

//////////////////////////////////////////////////
// Functions for read ESC
//////////////////////////////////////////////////

void ch1_rise() {    t_ch1.start();   }
void ch2_rise() {    t_ch2.start();   }
void ch3_rise() {    t_ch3.start();   }
void ch4_rise() {    t_ch4.start();   }
void ch5_rise() {    t_ch5.start();   }
//void ch6_rise() {    t_ch6.start();   }

void ch1_fall(){
    t_ch1.stop();
    pwm_in_ch1    =   t_ch1.read();
    t_ch1.reset();
}

void ch2_fall(){
    t_ch2.stop();
    pwm_in_ch2    =   t_ch2.read();
    t_ch2.reset();
}

void ch3_fall(){
    t_ch3.stop();
    pwm_in_ch3    =   t_ch3.read();
    t_ch3.reset();
    //pc.printf("%f\r\n", pwm_in_ch3);
}

void ch4_fall(){
    t_ch4.stop();
    pwm_in_ch4    =   t_ch4.read();
    t_ch4.reset();
}

void ch5_fall()
{
    t_ch5.stop();
    pwm_in_ch5    =   t_ch5.read();
    t_ch5.reset();
}
/*
void ch6_fall()
{
    t_ch6.stop();
    pwm_in_ch6    =   t_ch6.read();
    t_ch6.reset();
}
*/
