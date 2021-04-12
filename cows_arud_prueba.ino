/*
 * prueba con rosserial
 * envia una cadena
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("cows_found", &str_msg);


char cowinfo[50] = "cow1,1.10,2.20,3.30";



void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{ 
  
  str_msg.data = cowinfo;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
