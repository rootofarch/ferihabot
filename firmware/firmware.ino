/*
   rosserial Planar Odometry Example
*/
//#define USE_USBCON //only for serial connection
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <Wire.h>
#include <QuadratureEncoder.h>
#include <Odometer.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Scheduler.h>

#include <Arduino.h>
#include <SPI.h>
#include <WiFi101.h>

//--------tcp-ip-setup---------------
IPAddress server(192, 168, 1, 105);//ros master ip
WiFiClient client;

class WiFiHardware {

  public:
  WiFiHardware() {};

  void init() {
    // do your initialization here. this probably includes TCP server/client setup
    client.connect(server, 11411);
  }

  // read a byte from the serial port. -1 = failure
  int read() {
    // implement this method so that it reads a byte from the TCP connection and returns it
    return client.read();         
  }

  // write data to the connection to ROS
  void write(uint8_t* data, int length) {
    // implement this so that it takes the arguments and writes or prints them to the TCP connection
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }

  // returns milliseconds since start of program
  unsigned long time() {
     return millis();
  }
};

char ssid[] = "YOUR_SSID";     //  your network SSID (name)
char pass[] = "YOUR_SECRET_PASSPHRASE";  // your network password


/*
   ------------general variables--
*/

//servo pin setup
int leftForwardPin = 0;
int leftBackwardPin = 1;
int rightForwardPin = 2;
int rightBackwardPin = 3;
int leftThrust = 90;
int rightThrust = 90;
Servo leftMotor;
Servo rightMotor;

//localizaton varialbles
float head = 0;
float xVector = 0;
float yVector = 0;
float dWay = 0;
float ddWay = 0;
float hold = 0;

//bno055 setup
Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> gyro;
int bnoRstPin = 8;


//encoderPinSetup
int encoderLA = 4;
int encoderLB = 5;
int encoderRA = 7;
int encoderRB = 6;


//object definitions

Encoder leftEncoder(LEFT);
Encoder rightEncoder(RIGHT);
Odometer odom;

//clock definitions
unsigned long cycles = 0;
unsigned long start;

/*
   ------------general variables--
*/
double radius_of_whells = 6; //r
double wide_length = 9.7;     //l

/*
    ---------function declerations--
*/

//function definitions
void rightHandlerA();   //functions for encoder interrupt handlers
void rightHandlerB();
void leftHandlerA();
void leftHandlerB();

inline double degToRad(double deg);

float map_func(float value, float inMin, float inMax, float outMin, float outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
/*
   ---------function variable definitions--
*/
float fwd_message;
float turn_message;
float right_vel;
float left_vel;
int mapped_right;
int mapped_left;

/*
    ------------- ROS ------------
*/


//ros::NodeHandle  nh;  //for serail connections
ros::NodeHandle_<WiFiHardware> nh;

geometry_msgs::Pose2D pose2d;

double x = 1.0;
double y = 0.0;
double theta = 1.57;

ros::Publisher ros_odom("/feriha/pose2d", &pose2d);



/*
    reads cmd_vel topic then turn the motors
*/

void cmd_vel_handle( const geometry_msgs::Twist& msg) {
  fwd_message = msg.linear.x;
  turn_message = msg.angular.z;
  right_vel = int(((2.0 * fwd_message)+(turn_message * wide_length)) / (2.0 * radius_of_whells));
  left_vel = int(((2.0 * fwd_message)-(turn_message * wide_length)) / (2.0 * radius_of_whells));
  mapped_right = int(map_func(right_vel , -5 , 5 , 0 , 180));
  mapped_left = int(map_func(left_vel , -5 , 5 , 0 , 180));

  
  if (right_vel >= 0){
    digitalWrite(rightForwardPin,HIGH);
    digitalWrite(rightBackwardPin,LOW);
  }
  else{
    digitalWrite(rightForwardPin,LOW);
    digitalWrite(rightBackwardPin,HIGH);
  }
  if (left_vel >= 0){
    digitalWrite(leftForwardPin,HIGH);
    digitalWrite(leftBackwardPin,LOW);
  }
  else{
    digitalWrite(leftForwardPin,LOW);
    digitalWrite(leftBackwardPin,HIGH);
  }

  if(mapped_right > 180){
    rightMotor.write(180);
  }
  else{
    rightMotor.write(int(map_func(right_vel , -5 , 5 , 0 , 180)));
  }
  if(mapped_left > 180){
    leftMotor.write(180);
  }
  else{
    leftMotor.write(int(map_func(left_vel , -5 , 5 , 0 , 180)));
  }

}

ros::Subscriber<geometry_msgs::Twist> sub("/feriha/cmd_vel", &cmd_vel_handle );

/*
    ------------- ROS ------------
*/

void setup()
{
  WiFi.begin(ssid, pass); //wifi connection

  //rosnode initialize
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(ros_odom);

  //encoder setup for odometer
  rightEncoder.attach(encoderRA, encoderRB);
  leftEncoder.attach(encoderLA, encoderLB);
  leftEncoder.initialize();
  rightEncoder.initialize();
  attachInterrupt(digitalPinToInterrupt(leftEncoder.greenCablePin), leftHandlerA , CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoder.yellowCablePin), leftHandlerB , CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoder.greenCablePin), rightHandlerA , CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoder.yellowCablePin), rightHandlerB , CHANGE);


  //reset bno055
  pinMode(bnoRstPin, OUTPUT);
  digitalWrite(bnoRstPin, LOW);
  delay(500);
  digitalWrite(bnoRstPin, HIGH);
  delay(500);

  //initialize bno055 absolute orientation sensor
  while (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    nh.logerror("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    

    /*
      your solution here
      give rst pin ground and 3.3 volts
    */
    
    delay(100);
    
  }
  bno.setExtCrystalUse(true);
  delay(100);

  //PWM motor controller setup
  leftMotor.attach(A5);
  rightMotor.attach(A6);

  rightMotor.write(90);
  leftMotor.write(90);

  start = millis();

}

void loop()
{
  cycles =  millis() - start;
  start = millis();

  //get values from encoders for odometer
  hold = odom.getWay();
  dWay = hold - ddWay;
  ddWay = hold;
  sensors_event_t event;
  bno.getEvent(&event);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  yVector = yVector - dWay * sin(degToRad(event.orientation.x));
  xVector = xVector + dWay * cos(degToRad(event.orientation.x));

  //correct the orientation
  pose2d.x = xVector / 10.0;
  pose2d.y = yVector / 10.0;
  pose2d.theta = degToRad(map_func(event.orientation.x , 0 , 360 , 360 , 0));
  ros_odom.publish( &pose2d );

  nh.spinOnce();

  delay(2);
}


/* user defined functions bodies*/

void rightHandlerA() {
  rightEncoder.handleInterruptGreen();
  odom.rightEncoderTick = rightEncoder.encoderTicks;
}
void rightHandlerB() {
  rightEncoder.handleInterruptYellow();
  odom.rightEncoderTick = rightEncoder.encoderTicks;
}

void leftHandlerA() {
  leftEncoder.handleInterruptGreen();
  odom.leftEncoderTick = leftEncoder.encoderTicks;
}
void leftHandlerB() {
  leftEncoder.handleInterruptYellow();
  odom.leftEncoderTick = leftEncoder.encoderTicks;
}

inline double degToRad(double deg) {
  return deg * M_PI / 180.0;
}
