//Defining car address
#define CAR_ADDR 9
//Defining threshold to start motors
#define threshold 780

#define max_pwm 255
#define default_pwm 100

#define INPUT_SIZE 2
#define BUFFER_SIZE 50

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <stdlib.h>

Adafruit_MPU6050 mpu;

//Defining left motors
int in1 = 2;
int in2 = 4;
int enA = 3;

//Defining right motors
int in3 = 5;
int in4 = 7;
int enB = 6;

char *token;
const char *delimiter =",";
char *values[INPUT_SIZE];

char buffer[BUFFER_SIZE];

void setup() {
  // put your setup code here, to run once:

  //Setting up ouput pins for left motors
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  //Setting up ouput pins for right motors
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //Setting up built-in LED output
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  delay(100);
}


void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  char accel_x[10], accel_y[10], accel_z[10], gyro_x[10], gyro_y[10], gyro_z[10], temperature[10];

  dtostrf(a.acceleration.x,4,2,accel_x);
  dtostrf(a.acceleration.y,4,2,accel_y);
  dtostrf(a.acceleration.z,4,2,accel_z);
  dtostrf(g.gyro.x,4,2,gyro_x);
  dtostrf(g.gyro.y,4,2,gyro_y);
  dtostrf(g.gyro.z,4,2,gyro_z);
  dtostrf(temp.temperature,4,2,temperature);

  sprintf(buffer,"%s,%s,%s,%s,%s,%s,%s",
                  accel_x,accel_y,accel_z,
                  gyro_x,gyro_y,gyro_z,
                  temperature);
  Serial.println(buffer);

  // put your main code here, to run repeatedly:
  if (Serial.available() > 0){
    String message = Serial.readStringUntil('\n');
    char message_array[message.length()+1];
    message.toCharArray(message_array,message.length()+1);

    int idx = 0;
    token = strtok(message_array, delimiter);
    while (token != NULL) {
      if( idx < INPUT_SIZE ){
        values[idx++] = token;
      }
      token=strtok(NULL, delimiter);
    } 

    float linear = atof(values[0]);
    float angular = atof(values[1]);

    move(linear,angular);

  }
  delay(100);
}

void move(float linear,float angular){
  if (linear > 0){
    moveForward(linear,angular);
  }
  else if (linear < 0){
    moveBackward(linear,angular);
  }
  else{
    stop();
  }
}

void moveForward(float linear,float angular){
  //Left motors rotate forwards
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  //Right motors rotate forwards
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);

  float left_encoder_speed = default_pwm*abs(linear);
  float right_encoder_speed = default_pwm*abs(linear);

  if (left_encoder_speed < max_pwm){
    digitalWrite(enA,left_encoder_speed);
  }
  else{
    digitalWrite(enA,default_pwm);
  }
  if (right_encoder_speed < max_pwm){
    digitalWrite(enB,right_encoder_speed);
  }
  else{
    digitalWrite(enB,default_pwm);
  }

}

void moveBackward(float linear,float angular){
  //Left motors rotate backwards
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  //Right motors rotate backwards
  digitalWrite(in3,LOW);  
  digitalWrite(in4,HIGH);

  float left_encoder_speed = default_pwm*abs(linear);
  float right_encoder_speed = default_pwm*abs(linear);

  if (left_encoder_speed < max_pwm){
    digitalWrite(enA,left_encoder_speed);
  }
  else{
    digitalWrite(enA,default_pwm);
  }
  if (right_encoder_speed < max_pwm){
    digitalWrite(enB,right_encoder_speed);
  }
  else{
    digitalWrite(enB,default_pwm);
  }

}


void stop(){
  //Stops left motors
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(enA,default_pwm);

  //Stops right motors
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
  digitalWrite(enB,default_pwm);
}