#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

#define GEAR_1 13.5
#define GEAR_2 27
#define GEAR_3 13.5
#define GEAR_4 14.4
#define GEAR_5 14.4
#define PPR 4000

float ros_F;
float t_total;

ros::NodeHandle  nh;


void delay_3_micros()
{
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
}

void messageCallback(const std_msgs::Int16MultiArray& msg)
{
  
  //[pulses[0,1,2,3,4],dir[5],enable[6]
  unsigned long tttm;
  
  tttm = micros();
  if(msg.data[6]==1)
  {
    int n = msg.data[0];
    int L = (int)(t_total/n -3.4)+1;
    PORTC = (byte)(msg.data[5]);  //directions
    delayMicroseconds(6);
    int i = 0;
    while(i<n)
    {
      PORTA = B0000001;
      delay_3_micros();
      PORTA = B0000000;
      delayMicroseconds(L);
      i = i+1;
    }
  }
  else
  {
    PORTB = 0xFF; //(off)
    delay(100);
  }
  
    tttm = micros()-tttm;
    char result[8];
    String str=String(tttm);
    str.toCharArray(result,8); 
    nh.loginfo(result);
}


ros::Subscriber<std_msgs::Int16MultiArray> sub("joint_angles", &messageCallback );

void setup() {
  // put your setup code here, to run once:
DDRC = B11111111;  //direction pins (30,31,32,33,34,35,36,37)
DDRB = B11111111;  //enable pins (13,12,11,10,50,51,52,53)
DDRA = B11111111;  //step pins (29,28,27,26,25,24,23,22)


PORTC = B00000000;  //setting direction pins to low
PORTB = B00011111;  //setting enable pins to high (off)
PORTA = B00000000;  //setting step pins to low

ros_F = 110;
t_total = (1000000.0/ros_F);
//pinMode(13,OUTPUT);
//nh.getHardware()->setBaud(115200);
nh.initNode();
nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly
  nh.spinOnce();
  delay(1);
}
