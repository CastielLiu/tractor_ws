/* 
 * rosserial ADC Example
 * 
 * This is a poor man's Oscilloscope.  It does not have the sampling 
 * rate or accuracy of a commerical scope, but it is great to get
 * an analog value into ROS in a pinch.
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <rosserial_arduino/Adc.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include<SoftwareSerial.h>

ros::NodeHandle nh;

rosserial_arduino::Adc adc_msg;
std_msgs::String string_msg;
std_msgs::Char char_msg;

ros::Publisher pub_str("str",&string_msg);
ros::Publisher p("adc", &adc_msg);
ros::Publisher pub_char("char", &char_msg);

SoftwareSerial myserial(2,3);

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  //Serial.begin(115200);
  myserial.begin(9600);

  nh.advertise(p);
  nh.advertise(pub_str);
  nh.advertise(pub_char);
  
}

//We average the analog reading to elminate some of the noise
int averageAnalog(int pin)
{
  int v=0;
  for(int i=0; i<4; i++) 
    v+= analogRead(pin);
  return v/4;
}

String str("liushuaipeng\n");

void loop()
{
  adc_msg.adc0 = averageAnalog(0);
  p.publish(&adc_msg);
  
  for(int i=0; i<str.length(); ++i)
  {
    myserial.write(str[i]);
  }
  readMySerial();
  delay(20);
  nh.spinOnce();
}

void readMySerial() 
{
  while (myserial.available()) 
  {
    char inChar = (char)myserial.read(); 
    string_msg.data += inChar;
     char_msg.data = inChar;
    pub_char.publish(&char_msg);
    if (inChar == '\n') 
    {
      pub_str.publish(&string_msg);
    } 
  }
}



