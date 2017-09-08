
#define interruptPin 2

//#define ROS_INTERFACE

//commands
#define SEND_DATA 1
#define RESET_COUNTER 2
#define ENABLE_COUNTER 3
#define DISABLE_COUNTER 4

volatile unsigned long count = 0;
volatile byte enable = 0;

void wirteLongINT(unsigned long L)
{
  #ifdef ROS_INTERFACE
    byte b1 = L&0xFF;
    byte b2 = L>>8&0xFF;
    byte b3 = L>>16&0xFF;
    byte b4 = L>>24&0xFF;
    Serial.write(b1);
    Serial.write(b2);
    Serial.write(b3);
    Serial.write(b4);
  #else
    Serial.println(L);
  #endif
}

void setup() {
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), pulseCount, FALLING);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available()>0)
  {
    #ifdef ROS_INTERFACE
      int cmd = Serial.read();
    #else
      int cmd = Serial.read()-48;
    #endif
    
    unsigned long L = count;
    switch(cmd)
    {
      case SEND_DATA:
        wirteLongINT(L);
        break;
      case RESET_COUNTER:
        count = 0;
        break;
      case ENABLE_COUNTER:
        enable = 1;
        break;
      case DISABLE_COUNTER:
        enable = 0;
        break;
     default:
        #ifdef ROS_INTERFACE
          Serial.write('e');
        #else
          Serial.print('e');
        #endif
    }
  }
}

void pulseCount()
{
  if(enable)
    count = count + 1;
}

