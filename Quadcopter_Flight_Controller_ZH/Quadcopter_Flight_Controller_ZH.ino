
/* ======================================== 免责声明 ========================================
1. 本人学的机械，不是计算机或其它，代码渣勿喷（你猜为什么所有的函数都在一个文件里），不会花里胡哨的定时器中断
2. HC-06蓝牙接收指令的时候可能会卡，导致一个控制循环远远超过4ms（可能），所以建议不要飞机在空中的时候调PID。落地了再调
3. 本代码只适合Teensy 4.0，Teensy的其他板子可以试试，再有其他的板子大概会编译不成功
4. 保留了一些调试代码
5. 引脚号根据自己的需求改
6. 已经尽量做了注释，看不懂的B站问我。B站id：游笙0205Michael
7. 我讨厌小写开头然后中间一个大写命名变量和函数等，比如actualValue, readChannel。
8. 虽然说函数都在一个文件里，但你就说飞没飞起来吧
*/

// ================================================ 0.0 Teensy 4.0 Restart/Reboot ================================================ //
// 串口调试的时候输入R重启Teensy 4.0
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);
// Source: By KrisKasprzak in the thread of https://forum.pjrc.com/index.php?threads/soft-reboot-on-teensy4-0.57810/
// ================================================ 0.0 Teensy 4.0 Restart/Reboot ================================================ //
// ================================================ 1.0 Read Sensor Data Start ================================================ //
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
Adafruit_MPU6050 mpu;

// accel
float ax_float, ay_float, az_float; // 浮点型的加速度值，m/s^2
float ax_error, ay_error, az_error; // 加速度值的误差

// gyro
float wx, wy, wz; // 角速度，单位rad/s
float wx_error, wy_error, wz_error;// 陀螺仪误差，rad/s
Kalman kalmanX; // roll横滚
Kalman kalmanY; // pitch俯仰
double p_kf=0, r_kf; // 卡尔曼滤波后的俯仰和横滚
uint32_t timer_kf;
// ================================================ 1.0 Read Sensor Data End ================================================ //
// ================================================ 2.0 iBus and BT Control Start ================================================ //
#include <IBusBM.h>
IBusBM ibus;
bool receive_flag=1; // 串口里输入S进行紧急停机，需要重启

int update_interval=100; // 更新平板上蓝牙信息的更新间隔，ms
unsigned long last_time=0; // time of last update
char data_in; // data received from serial link
// ================================================ 2.0 iBus and BT Control End================================================ //
// ================================================ 3.0 PID Control Start ================================================ //
// 3.1 Actual value, from Sensor
float actual[4][6]={ // 实际值
  // 当前角度， 上一个角度， 角度的变化/速度（100倍角速度，rad/s），当前角速度（100倍角速度，rad/s），上一个角速度，角加速度
  //current, previous, change/speed
  {0,0,0,0,0,0}, // roll, rad, 100*rad/s
  {0,0,0,0,0,0}, // pitch, rad
  {0,0,0,0,0,0}, // height, m
  {0,0,0,0,0,0} // yaw, rad, rad/s
};

// 目标值，来自遥控器
// 3.2 Target value, from transimetter
int throttle_input; // 油门
float target[5]={0,0,0,0,0}; // 横滚，俯仰，油门，航向，锁定

// 误差值
// 3.3 Errors
float error[5][6]={
  // 当前角度误差，角度误差和，上一个角度误差，当前角速度误差，角速度误差和，上一个角速度误差
  //       angle                   angle v
  // current, sum, previous; current, sum, previous
     {0,       0,     0,        0,     0,      0}, // roll
  {0,0,0,0,0,0}, // pitch
  {0,0,0,0,0,0}, // height
  {0,0,0,0,0,0}, // yaw ang vel
};

// 3.4 KP, KI, KD
float KP_angle=2.75,KI_angle=0.001,KD_angle=0.63; // 角度环KP, KI, KD
float KP_angle_v=1.23,KI_angle_v=0.00113,KD_angle_v=8.0; // 角速度环KP, KI, KD
float KP_yaw_v=-5,KI_yaw_v=0; // 航向角KP

float K[4][6]={
  {KP_angle,KI_angle,KD_angle,KP_angle_v,KI_angle_v,KD_angle_v}, // roll, KP,KI,KD,KP_angular_v,KI_angular_v,KD_angular_v
  {KP_angle,KI_angle,KD_angle,KP_angle_v,KI_angle_v,KD_angle_v}, // pitch, KP,KI,KD,KP_angular_v,KI_angular_v,KD_angular_v
  {0,0,0,0,0,0}, // throttle, KP,KI,KD,KP_vertical_v,KI_vertical_v,KD_vertical_v
  {0,0,0,KP_yaw_v,KI_yaw_v,0} // yaw, KP,KI,KD,0,0,0
};

bool KI_flag; // 积分分离标志位
float integral_limit[4][5]{ // 稳态误差区间和积分限幅
  // 稳态误差角度，积分限幅，0，稳态误差角速度，积分限幅
  //             b                         b+1      ,0,      b                b+1
  // angle steady state error (deg), angle int limit,0, angle v ss error, angle v int limit
  {7,500,0,100,1000}, // roll: 
  {7,500,0,100,1000}, // pitch
  {200,0,0,0,1000},
  {200,0,0,0,1000} // yaw
};

// 3.5 Output in roll, pitch, throttle, yaw
float PID_output[4]={0,0,0,0}; // 最终四个通道的PID输出
// ================================================ 3.0 PID Control End ================================================ //
// ================================================ 4.0 Output Control Start ================================================ //
#define take_off_value 1600 // 起飞离地时的PWM（我随便写的，没测过，大概给了个数）

#define M1PIN 6
#define M2PIN 5
#define M3PIN 4
#define M4PIN 3
int motor_pin[4]={M1PIN,M2PIN,M3PIN,M4PIN};
bool M1_state=1, M2_state=1, M3_state=1, M4_state=1; // 点机输出标志位
int PWM[4]={0,0,0,0}; // PWM信号，1000到2000微秒
int pre_armed_state=0; // 上一次的解锁状态。解锁armed=1, 没解锁not armed=0
// ================================================ 4.0 Output Control End ================================================ //
// ================================================ 5.0 LED Indicator Start ================================================ //
#define red_LED_pin 15
#define green_LED_pin 14
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
  while ((micros()- loop_start_time)<loop_time){} // 等到过了4000微秒在开始下一个循环
  loop_start_time = micros(); 
}

unsigned long previous_time=0;
void loop_time_holder2(){
  // previous_time=loop2_start_time;
  while ((micros()- loop2_start_time)<loop_time){} // 等到过了4000微秒在开始下一个循环
  loop2_start_time = micros(); 
  // Serial.print("dt: ");
  // unsigned long dt=loop2_start_time-previous_time;
  // Serial.print(dt);
}
// ================================================ 0.0 Time End ================================================ //
// ================================================ 1.0 Read Sensor Data Start ================================================ //
// 初始化mpu6050
void mpu6050_setup(){ 
  Serial.flush();

  delay(1000);

  mpu6050_start();

  delay(1000);

  IMU_calibration(); // IMU校准，需要放地上不动

  delay(2000);

  // 卡尔曼滤波初始化
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
    ax_error=ax_error+ax_float; // 单位: m/s^2
    ay_error=ay_error+ay_float;
    az_error=az_error+az_float;
    i++;
  }
  ax_error=ax_error/num_of_loop; 
  ay_error=ay_error/num_of_loop;
  az_error=(az_error/num_of_loop-9.81); // 当平放静止不动的时候，这个值应该是9.81（还是-9.81)。反正减掉误差后的ax_float, ay_float, az_float都得是0
  
  // angular velocity error
  i=0;
  while (i < num_of_loop) {
    read_accel_gyro_raw();
    wx_error=wx_error+wx; // 单位: rad/sec
    wy_error=wy_error+wy;
    wz_error=wz_error+wz;
    i++;
  }
  wx_error=wx_error/num_of_loop; // 单位：rad/sec
  wy_error=wy_error/num_of_loop;
  wz_error=wz_error/num_of_loop;

  delay(500);

  // 输出加速度计和陀螺仪误差到串口监视器，可要可不要，反正看了也不会自己算
  /* 
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
  */
  digitalWrite(red_LED_pin,LOW);
}

void read_mpu6050_angle_loop(){
  // 获取三轴加速度和三轴角速度信息并且消除误差
  read_accel_gyro_raw();
  ax_float-=ax_error; // 单位：m/s^2
  ay_float-=ay_error;
  az_float-=az_error;
  wx-=wx_error; // 单位：rad/s
  wy-=wy_error;
  wz-=wz_error;
  
  double dt = (double)(micros() - timer_kf) / 1000000; 
  timer_kf = micros();
  
  // 只用加速度计解算出姿态
  double pitch=atan(ay_float/sqrt(sq(ax_float)+sq(az_float))); // 单位：rad/s
  double roll=atan(-1*ax_float/sqrt(sq(ay_float)+sq(az_float))); // 单位：rad/s

  // 卡尔曼滤波开始算。还是有个库更香
  r_kf = kalmanX.getAngle(roll, wy, dt); // 横滚, rad
  p_kf = kalmanY.getAngle(pitch, wx, dt); // 俯仰, rad

  // 惯性系下的航向角速度，也就是不论俯仰横滚角是多少，机头投影到地面上的航向角速度
  float yaw_v =(wz-wx*sin(p_kf)+wy*cos(p_kf)*sin(r_kf))/(cos(r_kf)*cos(p_kf));

  // 给实际值的数组赋值
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
  // Serial.print("Yaw rate:");Serial.print(actual[3][3]); Serial.print("\t");
  // Serial.print("p ang v:");Serial.print(pitch_ang_v*180/M_PI); Serial.print("\t");
}
// ================================================ 1.0 Read Sensor Data End ================================================ //
// ================================================ 2.0 iBus and BT Control Start ================================================ //
void communication_setup(){
  Serial.begin(460800); // 连电脑的串口监视器
  ibus.begin(Serial1); // 连接收机的ibus
  delay(2000);
  Serial2.begin(9600); // 连HC06
  Serial2.write("AT+BAUDA"); // 然后把HC06波特率设置成460800

  build_panel(); // Bluetooth Electronic这个软件生成的代码其中一个就是这个，大概就是初始化一下平板上的控制界面

  // Check if Armed
  ibus.loop(); // 如果是用Teensy 4.0和ibus一起用，必须要这行代码，不然读不出来
  target[4]=read_channel(4,-100,100,0); // 如果是解除锁定的状态，闪灯，直到锁定
  while (target[4] == 100){
    ibus.loop();
    digitalWrite(red_LED_pin, HIGH);
    delay(500);
    digitalWrite(red_LED_pin, LOW);
    delay(500);
    target[4]=read_channel(4,-100,100,0);
  }
}

void read_iBus_loop(){ // 读取接收机信号
  if (receive_flag){
    ibus.loop();
    // Read target value
    target[0]=read_channel(0,-30,30,0); // CH1, roll, angle, deg
    target[1]=-read_channel(1,-30,30,0); // CH2, pitch, angle, deg
    target[2]=read_channel(2,1000,1700,0); // CH3, throttle, vertical speed (m/s)
    throttle_input=read_channel(2,1000,1700,0);
    target[3]=-read_channel(3,-200,200,0); // CH4, yaw, angular velocity, 100*rad/s
    target[4]=read_channel(4,-100,100,0); // CH5, armed

    // 如果油门没起来，禁止其他三个通道的控制
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
  if (Serial.available()){ // 如果是从电脑上的串口监视器
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

  if (Serial2.available()){ // 如果是从蓝牙
    data_in=Serial2.read();  
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

  // 重新赋值
  K[0][0]=KP_angle; K[0][1]=KI_angle; K[0][2]=KD_angle; 
  K[0][3]=KP_angle_v; K[0][4]=KI_angle_v; K[0][5]=KD_angle_v;

  K[1][0]=KP_angle; K[1][1]=KI_angle; K[1][2]=KD_angle; 
  K[1][3]=KP_angle_v; K[1][4]=KI_angle_v; K[1][5]=KD_angle_v;

  K[3][3]=KP_yaw_v; K[3][4]=KI_yaw_v;
}

void send_bluetooth_data_loop(){ // 把调试信息通过蓝牙发送
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

int read_channel(byte channel, int min, int max, int default_val){
  int ch_val=ibus.readChannel(channel); //通道1: channel=0, 通道2: channel=1
  if (ch_val<100) return default_val; 
  return map(ch_val, 1000, 2000, min, max);
}
// ================================================ 2.0 iBus and BT Control End ================================================ //
// ================================================ 3.0 PID Control Start ================================================ //
void PID_control_attitude_loop(){
  // actual[a][b]
  // a+1=通道。a=0=通道1，a=1=通道2
  // b=0~2=角度, b=3~5=角速度

  // 3.1 Roll, pitch
  // 3.1.1 angle
  float temp_ch1_out=PID(0,0,target[0],actual[0][0]); // 通道1，0表示角度，通道1目标值，通道1角度实际值
  float temp_ch2_out=PID(1,0,target[1],actual[1][0]); // ch2, angle, target pitch ang, actual

  // 3.1.2 anglular velocity
  PID_output[0]=PID(0,3,temp_ch1_out,actual[0][3]); // 通道1，3表示角速度，角度环输出值，通道1角速度实际值
  PID_output[1]=PID(1,3,temp_ch2_out,actual[1][3]);

  // 3.3 yaw
  PID_output[3]=PID(3,3,target[3],actual[3][3]); // 通道4，3表示角速度，通道3目标值，通道4角速度实际值
}

float PID(int a, int b, float target_PID, float actual_PID){
  // a+1=通道。a=0=通道1，a=1=通道2

  // error[a][b] 
  // 如果是误差数组
  // 如果出现 |  则代表
  // -----------------------
  // b:      | 当前误差
  // b+1:    | 误差和
  // b+2:    | 前一个误差

  // K[a][b] KP, KI, KD
  // b:Kp
  // b+1: Ki
  // b+2: Kd
  error[a][b]=target_PID-actual_PID; 
  
  if ((target[4]==100) && (throttle_input>take_off_value)){ // 防止起飞时积分已经累计很大
    if (abs(error[a][b]) > integral_limit[a][b]){ // 如果离稳态误差还很远
      KI_flag=0;
      error[a][b+1]=0; // 清除误差和
    } else if (abs(error[a][b]) < integral_limit[a][b]){ // 如果离稳态误差很近
      KI_flag=1;
      error[a][b+1]+=error[a][b]; // 误差和
    } 
  } else {error[a][b+1]=0;} // 如果没有解除锁定或起飞
  
  // output = KP*  current err + KI_flag*KI     * error sum   +KD       *(current err - previous err)
  float KP_out=0, KI_out=0, KD_out=0;
  KP_out=K[a][b]*error[a][b];
  KI_out=KI_flag*K[a][b+1]*error[a][b+1];
  if (KI_out>integral_limit[a][b+1]) KI_out=integral_limit[a][b+1]; // 积分限幅
  if (KI_out<-integral_limit[a][b+1]) KI_out=-integral_limit[a][b+1]; // 积分限幅
  if (b==3) KD_out=K[a][b+2]*(error[a][b]-error[a][b+2]); // 如果是角速度，误差的差就是角加速度
  else KD_out=K[a][b+2]*actual[a][b+2]; // 如果是角度，误差的差就是角速度
  float output=KP_out+KI_out+KD_out;
  error[a][b+2]=error[a][b]; // 上次误差=这次误差
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
  digitalWrite(red_LED_pin,HIGH);
  // 输出250Hz的PWM信号，高电平1ms，低电平3ms，防止ESC进入油门行程校准模式
  DDRD = B01111000; // 把Teensy 4.0上端口D的3,4,5,6号引脚设置为输出模式。1为输出
  for (int i=0; i<1250; i++){ // 4毫秒一个周期，等待1250个周期, 一共5秒
    PORTD |= B01111000; delayMicroseconds(1000);
    PORTD &= B10000111; delayMicroseconds(3000); 
  }

  digitalWrite(red_LED_pin,LOW);
}

void motor_output_setup2(){ // 把电机输出引脚设置为低。我也不知道是不是必须的，但我还是写了
  pinMode(M1PIN,OUTPUT); digitalWriteFast(M1PIN,LOW);
  pinMode(M2PIN,OUTPUT); digitalWriteFast(M2PIN,LOW);
  pinMode(M3PIN,OUTPUT); digitalWriteFast(M3PIN,LOW);
  pinMode(M4PIN,OUTPUT); digitalWriteFast(M4PIN,LOW);
}

void motor_output_loop(){
  if (throttle_input>1200){ // 当油门大于1200时才进行自平衡
    // 这个加减要自己推导一下试一下，以免起飞就翻了
    // 我的飞机是这样的：
    // M1（对应PWM[0])：左前
    // M2（对应PWM[1])：又前
    // M3（对应PWM[2])：右后
    // M4（对应PWM[3])：左后
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

  // 新加的功能，视频里没有演示。如果在解锁电机的时候油门不是0，闪红灯，直到油门归零且锁定电机
  if (target[4]==100 && pre_armed_state==-100 && throttle_input>1002){
    while(throttle_input>1003 || target[4]==100){
      digitalWrite(red_LED_pin, HIGH); delay(300); digitalWrite(red_LED_pin, LOW); delay(300);
      ibus.loop();
      throttle_input=read_channel(2,1000,1700,0);
      target[4]=read_channel(4,-100,100,0);
    }
  }
  pre_armed_state=target[4];

  if (target[4]==100){               // 如果解除锁定，绿灯亮
    digitalWriteFast(red_LED_pin,LOW);
    digitalWriteFast(green_LED_pin,HIGH);
    for (byte i=0; i<4;i++){         // PWM限幅
      if (PWM[i]<1000) PWM[i]=1000;
      else if (PWM[i]>2000) PWM[i]=2000;
    }
  } else {                           // 如果没解除锁定，红灯亮
    digitalWriteFast(red_LED_pin,HIGH);
    digitalWriteFast(green_LED_pin,LOW);
    for (byte i=0; i<4;i++){         // PWM设置为最低，电机不转
        PWM[i]=1000;
    }
  }

  loop_time_holder2(); // 等待到4毫秒的周期
  // 反正PWM信号一定有1ms是高电平，先给高电平再接着计算。
  digitalWriteFast(M1PIN, HIGH); 
  digitalWriteFast(M2PIN, HIGH);
  digitalWriteFast(M3PIN, HIGH);
  digitalWriteFast(M4PIN, HIGH);

  // 计算每个电机的这一个PWM信号结束的时间。PWM信号是1000微秒到2000微秒
  // 最后输出的波形应该是示波器上显示的那样（图1）
  unsigned long M1_end_time=loop2_start_time+PWM[0]; 
  unsigned long M2_end_time=loop2_start_time+PWM[1];
  unsigned long M3_end_time=loop2_start_time+PWM[2];
  unsigned long M4_end_time=loop2_start_time+PWM[3];
  M1_state=1; M2_state=1; M3_state=1; M4_state=1; // 重置电机输出标志位

  while(M1_state==1 || M2_state==1 || M3_state==1 || M4_state==1) { // 如果有任意一路电机没到PWM结束的时间，就继续while循环
    unsigned long pwm_one_period_time=micros();                     // 当前时间
    if (M1_end_time<=pwm_one_period_time) {                         // 如果这个电机到时间了
      digitalWriteFast(M1PIN, LOW);                                 // 就拉低
      M1_state=0;                                                   // 更新标志位
    } else M1_state=1;                                              // 否则继续while。不加这行会有bug，我也不知道为什么，反正就是运行不起来
    
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