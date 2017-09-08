

#define DATA_SIZE 11 //in bytes
#define T_TOTAL 100000.0  //in microseconds

byte data[11];
int ind = 0;
void setup()
{
  //port setup (IO pins)
  DDRB = DDRB | B00100011;  //pins (_,_,13,12,11,10,9,8)
  PORTB = B00000000;
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
  data[8] = 0;
  data[9] = 0;
  data[10] = 0;
  
//  timer1 setup
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;            // preload timer 65536-16MHz/256/2Hz
  TCCR1B |= (1 << CS10);    // 1 prescaler 
  TIMSK1 |= (1 << TOIE1);
  interrupts();  
  Serial.begin(115200);
}



void loop()
{ 
  if(Serial.available()>=DATA_SIZE)
  {
    Serial.readBytes(data,DATA_SIZE);
    int p = data[0]<<8|data[1];
    int enable = (data[10]>>5)&(B00000001);
    byte dir = (byte)((data[10]<<1)&(B00000010));
    if(dir!=(PORTB&B00000010))
    {
      PORTB = dir;
      delayMicroseconds(8);
    }
    int n = p;
    if(p == 75 || p == 74)
    {
      PORTB = PORTB|B00100000;
    }
    
    unsigned int L = (int)(T_TOTAL/(2*n))-1;
    int i = 0;
    while(i<n)
    {
      PORTB = PORTB^B00000001;
      delayMicroseconds(L);
      PORTB = PORTB^B00000001;
      delayMicroseconds(L);
      i = i+1;
    }
  }
}
