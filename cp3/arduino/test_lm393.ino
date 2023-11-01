#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;
std_msgs::Int32 input_msg;
ros::Publisher output_pub("lm393_data", &input_msg);
int lm393_pin = A0;

void setup(){
  Serial.begin(9600);
  nh.initNode();
  nh.advertise(output_pub);
}
void loop(){
    lm393_info = analogRead(lm393_pin);
    input_msg.data = int(lm393_info);
    output_pub.publish(&input_msg);
    nh.spinOnce();
}