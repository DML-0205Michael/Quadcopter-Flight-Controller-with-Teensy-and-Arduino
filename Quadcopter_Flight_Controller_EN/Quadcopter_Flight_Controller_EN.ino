// ================================================ 0.0 Teensy 4.0 Restart/Reboot ================================================ //
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);
// ================================================ 0.0 Teensy 4.0 Restart/Reboot ================================================ //
// ================================================ 1.0 Read Sensor Data Start ================================================ //
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
Adafruit_MPU6050 mpu;
#include <EEPROM.h>
// #define EEPROM_size 24 // ax, ay, az error, gx,gy,gz error

// accel
float ax_float, ay_float, az_float;
float ax_error, ay_error, az_error; // error of linear acceleration

// gyro
float wx, wy, wz; // rad/s
float wx_error, wy_error, wz_error;// rad/s
Kalman kalmanX; // roll
Kalman kalmanY; // pitch
double p_kf=0, r_kf;
uint32_t timer_kf;
// ================================================ 1.0 Read Sensor Data End ================================================ //
// ================================================ 2.0 iBus and BT Control Start ================================================ //
#include <IBusBM.h>
IBusBM ibus;
bool receive_flag=1;

int update_interval=100; // time interval in ms for updating panel indicators
unsigned long last_time=0; // time of last update
char data_in; // data received from serial link
// ================================================ 2.0 iBus and BT Control End================================================ //
// ================================================ 3.0 PID Control Start ================================================ //
// 3.1 Actual value, from Sensor
float actual[4][6]={
 //current, previous, change/speed
  {0,0,0,0,0,0}, // roll, rad, 100*rad/s
  {0,0,0,0,0,0}, // pitch, rad
  {0,0,0,0,0,0}, // height, m
  {0,0,0,0,0,0} // yaw, rad, rad/s
};

// 3.2 Target value, from transimetter
int throttle_input;
float target[5]={0,0,0,0,0}; //r,p,t,y,armed
float target_angular_speed=0; // deg/s

// 3.3 Errors
float error[5][6]={
  //       angle                   angle v
  // current, sum, previous; current, sum, previous
     {0,       0,     0,        0,     0,      0}, // roll
  {0,0,0,0,0,0}, // pitch
  {0,0,0,0,0,0}, // height
  {0,0,0,0,0,0}, // yaw ang vel
};
 
// 3.4 KP, KI, KD
// float KP_angle=1.6, KI_angle=0.005, KD_angle=100; 
float KP_angle=2.75,KI_angle=0.001,KD_angle=0.63; 
float KP_angle_v=1.23,KI_angle_v=0.00113,KD_angle_v=8.0; 
float KP_yaw_v=-5,KI_yaw_v=0;

float K[4][6]={
  {KP_angle,KI_angle,KD_angle,KP_angle_v,KI_angle_v,KD_angle_v}, // roll, KP,KI,KD,KP_angular_v,KI_angular_v,KD_angular_v
  {KP_angle,KI_angle,KD_angle,KP_angle_v,KI_angle_v,KD_angle_v}, // pitch, KP,KI,KD,KP_angular_v,KI_angular_v,KD_angular_v
  {0,0,0,0,0,0}, // throttle, KP,KI,KD,KP_vertical_v,KI_vertical_v,KD_vertical_v
  {0,0,0,KP_yaw_v,KI_yaw_v,0} // yaw, KP,KI,KD,0,0,0
};

bool KI_flag;
float integral_limit[4][5]{ //    
  //             b                         b+1      ,0,      b                b+1
  // angle steady state error (deg), angle int limit,0, angle v ss error, angle v int limit
  {7,500,0,100,1000}, // roll: 
  {7,500,0,100,1000}, // pitch
  {200,0,0,0,1000},
  {200,0,0,0,1000} // yaw
};

// 3.5 Output in roll, pitch, throttle, yaw
float PID_output[4]={0,0,0,0};
// ================================================ 3.0 PID Control End ================================================ //
// ================================================ 4.0 Output Control Start ================================================ //
#define take_off_value 1600// of throttle

#define M1PIN 6
#define M2PIN 5
#define M3PIN 4
#define M4PIN 3
int motor_pin[4]={M1PIN,M2PIN,M3PIN,M4PIN};
bool M1_state=1, M2_state=1, M3_state=1, M4_state=1; 
int PWM[4]={0,0,0,0}; // length of high, microseconds
int pre_armed_state=0; // armed=1, not armed=0
// ================================================ 4.0 Output Control End ================================================ //
// ================================================ 5.0 LED Indicator Start ================================================ //
#define red_LED_pin 15
#define green_LED_pin 14
#define orange_LED_pin 13
// ================================================ 5.0 LED Indicator End ================================================ //

void setup() {
  motor_output_setup1();

  digitalWrite(green_LED_pin,HIGH); delay(200);digitalWrite(green_LED_pin,LOW);delay(200);
  delay(500);

  communication_setup();
  digitalWrite(green_LED_pin,HIGH); delay(200);digitalWrite(green_LED_pin,LOW);delay(200);
  digitalWrite(green_LED_pin,HIGH); delay(200);digitalWrite(green_LED_pin,LOW);delay(200);
  delay(500);

  mpu6050_setup();
  digitalWrite(green_LED_pin,HIGH); delay(200);digitalWrite(green_LED_pin,LOW);delay(200);
  digitalWrite(green_LED_pin,HIGH); delay(200);digitalWrite(green_LED_pin,LOW);delay(200);
  digitalWrite(green_LED_pin,HIGH); delay(200);digitalWrite(green_LED_pin,LOW);delay(200);
  delay(500);
  
  motor_output_setup2();
  digitalWrite(red_LED_pin,LOW); // finish calibration
  digitalWrite(green_LED_pin,HIGH); delay(100);digitalWrite(green_LED_pin,LOW);delay(100);// ready to go
  digitalWrite(green_LED_pin,HIGH); delay(100);digitalWrite(green_LED_pin,LOW);delay(100);
  digitalWrite(green_LED_pin,HIGH); delay(100);digitalWrite(green_LED_pin,LOW);delay(100);
  digitalWrite(green_LED_pin,HIGH); delay(100);digitalWrite(green_LED_pin,LOW);delay(100);
}

void loop() {
  read_bluetooth_data_loop();
  read_iBus_loop();

  send_bluetooth_data_loop();

  loop_time_holder1(); // next loop start point

  read_mpu6050_angle_loop();
  PID_control_attitude_loop();
  motor_output_loop(); // 1-2 ms high time

  Serial.println();

}
// ================================================ 0.0 Time Start ================================================ //
unsigned long loop_start_time=0,loop2_start_time=0;
const int loop_time=4000; // micro seconds
void loop_time_holder1(){
  while ((micros()- loop_start_time)<loop_time){} 
  loop_start_time = micros(); 
}

unsigned long previous_time=0;
void loop_time_holder2(){
  // previous_time=loop2_start_time;
  while ((micros()- loop2_start_time)<loop_time){} 
  loop2_start_time = micros(); 
  // Serial.print("dt: ");
  // unsigned long dt=loop2_start_time-previous_time;
  // Serial.print(dt);
}
// ================================================ 0.0 Time End ================================================ //
// ================================================ 1.0 Read Sensor Data Start ================================================ //
void mpu6050_setup(){ 
  Serial.flush();

  delay(1000);

  mpu6050_start();

  delay(1000);

  IMU_calibration(); 

  delay(2000);

  read_accel_gyro_raw();
  ax_float-=ax_error;
  ay_float-=ay_error;
  az_float-=az_error;

  double pitch=atan(ay_float/sqrt(sq(ax_float)+sq(az_float))); 
  double roll=atan(-1*ax_float/sqrt(sq(ay_float)+sq(az_float)));
  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  timer_kf = micros();
}

void mpu6050_start(){
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

void read_accel_gyro_raw(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax_float=a.acceleration.x;
  ay_float=a.acceleration.y;
  az_float=a.acceleration.z;

  wx=g.gyro.x;
  wy=g.gyro.y;
  wz=g.gyro.z;
}

void IMU_calibration() {
  digitalWrite(red_LED_pin,HIGH);
  Serial.println("Start IMU calibration. ");
  // acceleration error
  int num_of_loop=1000; // number of sample
  int i=0;
  while (i < num_of_loop) {
    read_accel_gyro_raw();
    ax_error=ax_error+ax_float; // unit: m/s^2
    ay_error=ay_error+ay_float;
    az_error=az_error+az_float;
    i++;
  }
  ax_error=ax_error/num_of_loop; // unit: raw data
  ay_error=ay_error/num_of_loop;
  az_error=(az_error/num_of_loop-9.81); // should measure 1g when stationary. chip face is -z
  
  // angular velocity error
  i=0;
  while (i < num_of_loop) {
    read_accel_gyro_raw();
    wx_error=wx_error+wx; // unit: rad/sec
    wy_error=wy_error+wy;
    wz_error=wz_error+wz;
    i++;
  }
  wx_error=wx_error/num_of_loop; // rad/sec
  wy_error=wy_error/num_of_loop;
  wz_error=wz_error/num_of_loop;

  delay(500);


  Serial.println("Minus these errors when converting from int16_t to float. ");
  Serial.println("Acceleration error (m/s^2): ");
  Serial.print("ax_error: ");
  Serial.println(ax_error);
  Serial.print("ay_error: ");
  Serial.println(ay_error);
  Serial.print("az_error: ");
  Serial.println(az_error);
  Serial.println();

  Serial.println("Angular velocity error (rad/s): ");
  Serial.print("wx_error: ");
  Serial.println(wx_error);
  Serial.print("wy_error: ");
  Serial.println(wy_error);
  Serial.print("wz_error: ");
  Serial.println(wz_error);

  Serial.println("Accel and gyro Calibration finished.");
  Serial.println();
  digitalWrite(red_LED_pin,LOW);
}

void read_mpu6050_angle_loop(){
  read_accel_gyro_raw();
  ax_float-=ax_error;
  ay_float-=ay_error;
  az_float-=az_error;
  wx-=wx_error;
  wy-=wy_error;
  wz-=wz_error;

  double dt = (double)(micros() - timer_kf) / 1000000; // Calculate delta time
  timer_kf = micros();
  
  double pitch=atan(ay_float/sqrt(sq(ax_float)+sq(az_float))); // *180/M_PI; 
  double roll=atan(-1*ax_float/sqrt(sq(ay_float)+sq(az_float))); // *180/M_PI;

  r_kf = kalmanX.getAngle(roll, wy, dt); // roll, rad
  p_kf = kalmanY.getAngle(pitch, wx, dt); // pitch, rad

  float yaw_v = (wz - wx * sin(p_kf) + wy * cos(p_kf) * sin(r_kf)) / (cos(r_kf) * cos(p_kf));

  //      angle                        angle v
  // previous=current
  actual[0][1]=actual[0][0];   actual[0][4]=actual[0][3]; // ch1
  actual[1][1]=actual[1][0];   actual[1][4]=actual[1][3]; // ch2
                               actual[3][4]=actual[3][3]; // ch4
  // current=new value
  actual[0][0]=r_kf*180/M_PI;  actual[0][3]=wy*100; // ch1
  actual[1][0]=p_kf*180/M_PI;  actual[1][3]=wx*100; // ch2
                               actual[3][3]=yaw_v*100; // ch4

  // change=current-pre=ang v
  actual[0][2]=wy*100;         actual[0][5]=actual[0][3]-actual[0][4]; // ch1
  actual[1][2]=wx*100;         actual[1][5]=actual[1][3]-actual[1][4]; // ch2
                               actual[3][5]=actual[3][3]-actual[3][4]; // ch4
  
  // Serial.print("p_kf:");Serial.print(actual[1][0]); Serial.print("\t");
  // Serial.print("r_kf:");Serial.print(actual[0][0]); Serial.print("\t");
  // Serial.print("wx:");Serial.print(actual[1][3]); Serial.print("\t");
  // Serial.print("wy:");Serial.print(actual[0][3]); Serial.print("\t");
  Serial.print("Yaw rate:");Serial.print(actual[3][3]); Serial.print("\t");
  // Serial.print("p ang v:");Serial.print(pitch_ang_v*180/M_PI); Serial.print("\t");
}
// ================================================ 1.0 Read Sensor Data End ================================================ //
// ================================================ 2.0 iBus and BT Control Start ================================================ //
void communication_setup(){
  Serial.begin(460800);
  ibus.begin(Serial1);
  delay(2000);
  Serial2.begin(9600);
  Serial2.write("AT+BAUDA"); // it won't save the setting. set to 460800

  build_panel();

  // Check if Armed
  ibus.loop();
  target[4]=read_channel(4,-100,100,0);
  while (target[4] == 100){
    ibus.loop();
    digitalWrite(red_LED_pin, HIGH);
    delay(500);
    digitalWrite(red_LED_pin, LOW);
    delay(500);
    target[4]=read_channel(4,-100,100,0);
  }
}

void read_iBus_loop(){
  if (receive_flag){
  ibus.loop();
  target[0]=read_channel(0,-30,30,0); // CH1, roll, angle, deg
  target[1]=-read_channel(1,-30,30,0); // CH2, pitch, angle, deg
  target[2]=read_channel(2,1000,1700,0); // CH3, throttle, vertical speed (m/s)
  throttle_input=read_channel(2,1000,1700,0);
  target[3]=-read_channel(3,-200,200,0); // CH4, yaw, angular velocity, 100*rad/s
  target[4]=read_channel(4,-100,100,0); // CH5, armed

  // if throttle not spinning, disable roll pitch yaw manual control
  if (throttle_input<=1200){
    target[0]=0;
    target[1]=0;
    target[3]=0;
  }
  } else {
    for (byte i=0; i<4;i++){
        target[i]=0;
    }
    target[4]=-100;
  }
}

void build_panel(){}

void read_bluetooth_data_loop(){
  if (Serial.available()){
    data_in=Serial.read();  //Get next character
    if(data_in=='S'){receive_flag=0;}
    if(data_in=='R'){CPU_RESTART;}
    if(data_in=='A'){KP_angle=float(Serial.parseInt())/1000;} // in:0~30, out: 0~500
    if(data_in=='B'){KI_angle=float(Serial.parseInt())/1000;}
    if(data_in=='C'){KD_angle=float(Serial.parseInt())/1000;}
    if(data_in=='D'){KP_angle_v=float(Serial.parseInt())/1000;} // in: 0~500, out: 0~1000
    if(data_in=='E'){KI_angle_v=float(Serial.parseInt())/1000000;}
    if(data_in=='F'){KD_angle_v=float(Serial.parseInt())/1000;}
  }

  if (Serial2.available()){
    data_in=Serial2.read();  //Get next character
    if(data_in=='S'){receive_flag=0;}
    if(data_in=='R'){CPU_RESTART;}
    if(data_in=='A'){KP_angle=float(Serial2.parseInt())/1000;} // in:0~30, out: 0~500
    if(data_in=='B'){KI_angle=float(Serial2.parseInt())/1000;}
    if(data_in=='C'){KD_angle=float(Serial2.parseInt())/1000;}
    if(data_in=='D'){KP_angle_v=float(Serial2.parseInt())/1000;} // in: 0~500, out: 0~1000
    if(data_in=='E'){KI_angle_v=float(Serial2.parseInt())/1000000;}
    if(data_in=='F'){KD_angle_v=float(Serial2.parseInt())/1000;}
    if(data_in=='G'){KP_yaw_v=float(Serial2.parseInt());}
    if(data_in=='H'){KI_yaw_v=float(Serial2.parseInt())/1000;}
  }

  K[0][0]=KP_angle; K[0][1]=KI_angle; K[0][2]=KD_angle; 
  K[0][3]=KP_angle_v; K[0][4]=KI_angle_v; K[0][5]=KD_angle_v;

  K[1][0]=KP_angle; K[1][1]=KI_angle; K[1][2]=KD_angle; 
  K[1][3]=KP_angle_v; K[1][4]=KI_angle_v; K[1][5]=KD_angle_v;

  K[3][3]=KP_yaw_v; K[3][4]=KI_yaw_v;
}

void send_bluetooth_data_loop(){
  unsigned long t=millis();
  if ((t-last_time)>update_interval){
    String temp;
    last_time=t;

    Serial2.print("1:"); Serial2.print(PID_output[0]); Serial2.print("\t");
    Serial2.print("2:"); Serial2.print(PID_output[1]); Serial2.print("\t");
    Serial2.print("3:"); Serial2.print(PID_output[2]); Serial2.print("\t");
    Serial2.print("4:"); Serial2.print(PID_output[3]); Serial2.print("\t");


    // Serial2.print("1:"); Serial2.print(PWM[0]); Serial2.print("\t");
    // Serial2.print("2:"); Serial2.print(PWM[1]); Serial2.print("\t");
    // Serial2.print("3:"); Serial2.print(PWM[2]); Serial2.print("\t");
    // Serial2.print("4:"); Serial2.print(PWM[3]); Serial2.print("\t");
    Serial2.println();
  }
}

int read_channel(byte channel, int min, int max, int default_value){
  int ch=ibus.readChannel(channel); //0, 1, 2, 3, 4
  if (ch<100) return default_value; // if noise
  return map(ch, 1000, 2000, min, max);
}
// ================================================ 2.0 iBus and BT Control End ================================================ //
// ================================================ 3.0 PID Control Start ================================================ //
void PID_control_attitude_loop(){
  // actual[a][b]
  // a+1=channel number
  // b=0~2=angle, b=3~5=ang v
  
  float temp_ch1_out=PID(0,0,target[0],actual[0][0]); // ch1, angle, target roll ang, actual
  float temp_ch2_out=PID(1,0,target[1],actual[1][0]); // ch2, angle, target pitch ang, actual

  PID_output[0]=PID(0,3,temp_ch1_out,actual[0][3]);
  PID_output[1]=PID(1,3,temp_ch2_out,actual[1][3]);

  PID_output[3]=PID(3,3,target[3],actual[3][3]);

}

float PID(int a, int b, float target_PID, float actual_PID){
  // a+1=channel

  // error[a][b]
  // b: current (error)
  // b+1: sum
  // b+2: previous error

  // K[a][b]
  // b:Kp
  // b+1: Ki
  // b+2: Kd
  error[a][b]=target_PID-actual_PID; // current error of channel a, b=0~2 is angle, b=3~5 is ang vel
  //  // Serial.print("\t");
  
  if ((target[4]==100) && (throttle_input>take_off_value)){ // prevent large error sum before take off
    //if ((actual_PID > integral_limit) || (actual_PID < -integral_limit)){ // if not reaching steady state/still have large overshoot
    if (abs(error[a][b]) > integral_limit[a][b]){ // if outside of ss error
      KI_flag=0;
      error[a][b+1]=0; // clear sum
    } else if (abs(error[a][b]) < integral_limit[a][b]){ // if within ss error
      KI_flag=1;
      error[a][b+1]+=error[a][b]; // sum +=current
    } 
  } else {error[a][b+1]=0;} // if not armed or not take off
  
  float KP_out=0, KI_out=0, KD_out=0;
  KP_out=K[a][b]*error[a][b];
  KI_out=KI_flag*K[a][b+1]*error[a][b+1];
  if (KI_out>integral_limit[a][b+1]) KI_out=integral_limit[a][b+1];
  if (KI_out<-integral_limit[a][b+1]) KI_out=-integral_limit[a][b+1];
  if (b==3) KD_out=K[a][b+2]*(error[a][b]-error[a][b+2]); // if ang v
  else KD_out=K[a][b+2]*actual[a][b+2]; // if ang
  float output=KP_out+KI_out+KD_out;
  // PID_output[a]=K[a][b]*error[a][b]+KI_flag*K[a][b+1]*error[a][b+1]+K[a][b+2]*(error[a][b]-error[a][b+2]);
  error[a][b+2]=error[a][b];
  return output=constrain(output,-600,600);

  /*
  if (b==3){ // angular v
    Serial.print("KPv: "); Serial.print(K[a][b]); Serial.print("\t"); // CH2, pitch, KP
    Serial.print("KDv: "); Serial.print(K[a][b+2]); Serial.print("\t"); // CH2, pitch, KD
    // Serial.print("Err: "); Serial.print(error[a][b]); Serial.print("\t");
    // Serial.print("KP_out: "); Serial.print(KP_out); Serial.print("\t");
    // Serial.print("KD_out: "); Serial.print(KD_out); Serial.print("\t");
    // Serial.print("Tot out: "); Serial.print(PID_output[a]); Serial.print("\t");
  } else if (b==0){ // angle
    Serial.print("KP: "); Serial.print(K[a][b]); Serial.print("\t"); // CH2, pitch, KP
    Serial.print("KI: "); Serial.print(K[a][b+1],3); Serial.print("\t"); // CH2, pitch, KI
    Serial.print("KD: "); Serial.print(K[a][b+2]); Serial.print("\t"); // CH2, pitch, KD
    Serial.print("Err: "); Serial.print(error[a][b]); Serial.print("\t");
    // Serial.print("KP_out: "); Serial.print(KP_out); Serial.print("\t");
    Serial.print("KI_out: "); Serial.print(KI_out); Serial.print("\t");
    // Serial.print("KD_out: "); Serial.print(KD_out); Serial.print("\t");
    Serial.print("Tot out: "); Serial.print(PID_output[a]); Serial.print("\t");
  }
  */
  
}
// ================================================ 3.0 PID Control End ================================================ //
// ================================================ 4.0 Output Control Start ================================================ //
void motor_output_setup1(){
  pinMode(red_LED_pin, OUTPUT);
  pinMode(green_LED_pin, OUTPUT);
  pinMode(orange_LED_pin, OUTPUT);
  digitalWrite(red_LED_pin,HIGH);
  DDRD = B01111000; // set digital pins 3,4,5,6 to output
  for (int i=0; i<1250; i++){                           //Wait 5 seconds 
    PORTD |= B01111000;
    delayMicroseconds(1000);
    PORTD &= B10000111; 
    delayMicroseconds(3000); 
  }

  digitalWrite(red_LED_pin,LOW);
}

void motor_output_setup2(){
  pinMode(M1PIN,OUTPUT); digitalWriteFast(M1PIN,LOW);
  pinMode(M2PIN,OUTPUT); digitalWriteFast(M2PIN,LOW);
  pinMode(M3PIN,OUTPUT); digitalWriteFast(M3PIN,LOW);
  pinMode(M4PIN,OUTPUT); digitalWriteFast(M4PIN,LOW);
}

void motor_output_loop(){
  if (throttle_input>1200){ // start balancing after started spinning
    PWM[0]=throttle_input+PID_output[0]+PID_output[1]-PID_output[3]; // M1
    PWM[1]=throttle_input-PID_output[0]+PID_output[1]+PID_output[3]; // M2
    PWM[2]=throttle_input-PID_output[0]-PID_output[1]-PID_output[3]; // M3
    PWM[3]=throttle_input+PID_output[0]-PID_output[1]+PID_output[3]; // M4
  } else {
    PWM[0]=throttle_input;
    PWM[1]=throttle_input;
    PWM[2]=throttle_input;
    PWM[3]=throttle_input;
  }

  if (target[4]==100 && pre_armed_state==-100 && throttle_input>1002){
    while(throttle_input>1003 || target[4]==100){
      digitalWrite(red_LED_pin, HIGH); delay(300); digitalWrite(red_LED_pin, LOW); delay(300);
      ibus.loop();
      throttle_input=read_channel(2,1000,1700,0);
      target[4]=read_channel(4,-100,100,0);
    }
  }
  pre_armed_state=target[4];

  if (target[4]==100){               // if armed, green on
    digitalWriteFast(red_LED_pin,LOW);
    digitalWriteFast(green_LED_pin,HIGH);
    for (byte i=0; i<4;i++){         // if armed, limit PWM signal
      if (PWM[i]<1000) PWM[i]=1000;
      else if (PWM[i]>2000) PWM[i]=2000;
    }
  } else {                           // if not armed, red on
    digitalWriteFast(red_LED_pin,HIGH);
    digitalWriteFast(green_LED_pin,LOW);
    for (byte i=0; i<4;i++){         // if not armed, 0 PWM signal
        PWM[i]=1000;
    }
  }
 
  loop_time_holder2();
  digitalWriteFast(M1PIN, HIGH);
  digitalWriteFast(M2PIN, HIGH);
  digitalWriteFast(M3PIN, HIGH);
  digitalWriteFast(M4PIN, HIGH);
  unsigned long M1_end_time=loop2_start_time+PWM[0];
  unsigned long M2_end_time=loop2_start_time+PWM[1];
  unsigned long M3_end_time=loop2_start_time+PWM[2];
  unsigned long M4_end_time=loop2_start_time+PWM[3];
  M1_state=1; M2_state=1; M3_state=1; M4_state=1; 

  while(M1_state==1 || M2_state==1 || M3_state==1 || M4_state==1) {
    // Serial.println("While loop");
    unsigned long pwm_one_period_time=micros();
    if (M1_end_time<=pwm_one_period_time) {
      digitalWriteFast(M1PIN, LOW); // pin 6 to low
      M1_state=0;
    } else M1_state=1;
    
    if (M2_end_time<=pwm_one_period_time){
      digitalWriteFast(M2PIN, LOW); // pin 5 to low
      M2_state=0;
    } else M2_state=1;
    
    if (M3_end_time<=pwm_one_period_time){
      digitalWriteFast(M3PIN, LOW); // pin 4 to low
      M3_state=0;
    } else M3_state=1;

    if (M4_end_time<=pwm_one_period_time){
      digitalWriteFast(M4PIN, LOW); // pin 3 to low
      M4_state=0;
    } else M4_state=1;
  }
}

// ================================================ 4.0 Output Control End ================================================ //


