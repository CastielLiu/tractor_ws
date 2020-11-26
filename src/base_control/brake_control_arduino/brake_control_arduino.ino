#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>

int EN1 = 11;    //　clutch   GREEN      (left)
int DIR1 = 12;   //          YELLOW
int PUL1 = 13;   //          RED

int EN2 = 8;     //  brake   RED      (right)
int DIR2 = 7;    //          YELLOW
int PUL2 = 6;    //          GREEN
int ADC_L_PIN = 0 ;   //left sensor adc pin
int ADC_R_PIN = 1 ;   //right sensor adc pin

int BRAKE_DIR_PRESS   = LOW;
int BRAKE_DIR_RELEASE = HIGH;

int CLUTCH_DIR_PRESS  = LOW;
int CLUTCH_DIR_RELEASE  = HIGH;

int ENABLE = LOW;
int DISABLE = HIGH;

int g_brake_tatol_grade = 5;
int g_expect_brake_grade = 0;
int g_adc_l, g_adc_r;
int g_adc_l_min = 110, g_adc_l_max = 150;
int g_adc_r_min = 95, g_adc_r_max = 125;
std_msgs::UInt8   g_brake_state_feedback;
std_msgs::UInt16  g_brake_sensor_adc_l_feedback;
std_msgs::UInt16  g_brake_sensor_adc_r_feedback;

void cmd_callback(const std_msgs::UInt8& cmd_msg);
void requestSensorVal_callback(const std_msgs::Empty& msg);

ros::NodeHandle g_nh;
//calibrate brake position
ros::Subscriber<std_msgs::Empty> g_sub_request_sensor_val("/request_sensor_val", &requestSensorVal_callback); 
ros::Publisher g_pub_adc_l("/brake_sensor_adc_left", &g_brake_sensor_adc_l_feedback);
ros::Publisher g_pub_adc_r("/brake_sensor_adc_right", &g_brake_sensor_adc_r_feedback);

ros::Publisher g_pub_state("/brake_state", &g_brake_state_feedback);
ros::Subscriber <std_msgs::UInt8>  g_sub_cmd("/brake_cmd", &cmd_callback );

void requestSensorVal_callback(const std_msgs::Empty& msg)
{
  static boolean first = true;
  if(first)
  {
    first = false;
    g_nh.advertise(g_pub_adc_l);
    g_nh.advertise(g_pub_adc_r);  
    return;
  }
  g_pub_adc_l.publish(&g_brake_sensor_adc_l_feedback);
  g_pub_adc_r.publish(&g_brake_sensor_adc_r_feedback);
}

void rotateClutch(int dir, int pwm_duration_us, int rotate_duration_us)
{
  int pwm_duration_half_us = pwm_duration_us/2;
  int for_cnt = rotate_duration_us/pwm_duration_us;
  digitalWrite(EN1,LOW);
  digitalWrite(DIR1,dir);
  
  for(int i=1; i<for_cnt; i++){ //circles of spanning,4000=1 circle
      digitalWrite(PUL1,HIGH);  
      delayMicroseconds(pwm_duration_half_us); //speed of spanning //us
      digitalWrite(PUL1,LOW);  
      delayMicroseconds(pwm_duration_half_us);  
    }
}

void rotateBrake(int dir, int pwm_duration_us, int rotate_duration_us)
{
  int pwm_duration_half_us = pwm_duration_us/2;
  int for_cnt = rotate_duration_us/pwm_duration_us;
  digitalWrite(EN2,LOW); 
  digitalWrite(DIR2,dir);
  
  for(int i=1; i<for_cnt; i++){ //circles of spanning,4000=1 circle
    digitalWrite(PUL2,HIGH);  
    delayMicroseconds(pwm_duration_half_us); //speed of spanning 
    digitalWrite(PUL2,LOW);  
    delayMicroseconds(pwm_duration_half_us);  
  } 
}

void cmd_callback(const std_msgs::UInt8& cmd_msg)
{
  static int rotate_duration_us = 1000000/20/2; //1000000us / 20hz /2motors
  static int pwm_duration_us    = 500; //us
  static int brake_up_limited = 0;
  static int brake_down_limited = 0;
  static int clutch_up_limited = 0;
  static int clutch_down_limited = 0;
  
  
  g_expect_brake_grade = cmd_msg.data;
  g_adc_l = analogRead(ADC_L_PIN);
  g_adc_r = analogRead(ADC_R_PIN);
  g_brake_sensor_adc_l_feedback.data = g_adc_l;
  g_brake_sensor_adc_r_feedback.data = g_adc_r;
  
  if(g_expect_brake_grade!= 0) // brake
  {
    if ( g_adc_l > g_adc_l_min) //left clutch
    {
      rotateClutch(CLUTCH_DIR_PRESS, pwm_duration_us, rotate_duration_us);
    }
      
    if ( g_adc_r > g_adc_r_min) //right brake
    {
        rotateBrake(BRAKE_DIR_PRESS, pwm_duration_us, rotate_duration_us);
    }
    
    //received pressed cmd the release limit reset
    brake_up_limited = 0;
    clutch_up_limited = 0;
  }
  else //  go back
  {
    if (g_adc_l < g_adc_l_max && clutch_up_limited == 0) 
      rotateClutch(CLUTCH_DIR_RELEASE, pwm_duration_us, rotate_duration_us);
    else
      clutch_up_limited = 1;
      
    if (g_adc_r < g_adc_r_max && brake_up_limited == 0)  
      rotateBrake(BRAKE_DIR_RELEASE, pwm_duration_us, rotate_duration_us) ;
    else
      brake_up_limited = 1;
  }
  
  if(g_brake_state_feedback.data > g_brake_tatol_grade)
    g_brake_state_feedback.data = g_brake_tatol_grade;
    
  g_pub_state.publish(&g_brake_state_feedback);
}


void setup()
{
   Serial.begin(57600);
   pinMode(EN1,OUTPUT);
   pinMode(DIR1,OUTPUT);
   pinMode(PUL1,OUTPUT); 
   pinMode(EN2,OUTPUT);
   pinMode(DIR2,OUTPUT);
   pinMode(PUL2,OUTPUT); 
  
  g_nh.initNode();
  g_nh.subscribe(g_sub_cmd);
  g_nh.subscribe(g_sub_request_sensor_val);
  g_nh.advertise(g_pub_state);
}

void loop()
{
  g_adc_l = analogRead(ADC_L_PIN);
  g_adc_r = analogRead(ADC_R_PIN);
 // Serial.println(g_adc_l);
  //Serial.println(g_adc_r);
  /*
  if ( g_adc_l < g_adc_l_min || g_adc_l > g_adc_l_max) // the clutch shaft reach the up or down limit
  {
    digitalWrite(EN1,LOW); 
    digitalWrite(DIR1, LOW);
    analogWrite(PUL1,0);
  }
  
  if ( g_adc_r < g_adc_r_min || g_adc_r > g_adc_r_max) // the brake shaft reach the up or down limit
  {
    digitalWrite(EN2,LOW); 
    digitalWrite(DIR2, LOW);
    analogWrite(PUL2,0);
  }
  */
  
  /*
  if(g_adc_r <= g_adc_r_min )
    g_brake_state_feedback.data = g_brake_tatol_grade;
  else if(g_adc_r >= g_adc_r_max)
    g_brake_state_feedback.data = 0;
  else
    g_brake_state_feedback.data = g_brake_tatol_grade - 1.0*(g_adc_r - g_adc_r_min)/(g_adc_r_max - g_adc_r_min) * g_brake_tatol_grade;
    */
    
  if(g_adc_l <= g_adc_l_min )
    g_brake_state_feedback.data = g_brake_tatol_grade;
  else if(g_adc_l >= g_adc_l_max)
    g_brake_state_feedback.data = 0;
  else
    g_brake_state_feedback.data = g_brake_tatol_grade - 1.0*(g_adc_l - g_adc_l_min)/(g_adc_l_max - g_adc_l_min) * g_brake_tatol_grade;
    
    
  g_brake_sensor_adc_l_feedback.data = g_adc_l;
  g_brake_sensor_adc_r_feedback.data = g_adc_r;
  
   g_nh.spinOnce();
  
}
