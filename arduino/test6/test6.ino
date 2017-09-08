
#define DATA_SIZE 11 //in bytes
#define T_TOTAL 80000.0  //in microseconds

long n = 0;
long x = 0;
unsigned int L = 0;
int ind = 0;
byte dataReaded = 0;
byte data[11];



ISR(TIMER1_OVF_vect)          // timer compare interrupt service routine
{
  PORTB = PORTB^B00000001;
  if((PORTB&B00000001)==1)
  {
    n = n-1;
    x = x-1;
  }
  TCNT1 = L;
}


void setup()
{
  //port setup (IO pins)
  DDRB = DDRB | B00000011;  //pins (_,_,13,12,11,10,9,8)
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
  //TIMSK1 |= (1 << TOIE1);
  interrupts();  
  Serial.begin(115200);
}


void serialEvent()
{
  while(Serial.available())
  {
    data[ind] = Serial.read();
    ind = ind +1;
    if(ind==11)
    {
      ind = 0;
      dataReaded = 1;
    }
  }
}

void sendData(unsigned long tt)
{
    byte t1 = tt&0xFF;
    byte t2 = tt>>8&0xFF;
    byte t3 = tt>>16&0xFF;
    byte t4 = tt>>24&0xFF;
    Serial.write(t1);
    Serial.write(t2);
    Serial.write(t3);
    Serial.write(t4);
}

void loop()
{ 
  if(dataReaded)
  {
    unsigned long data1 = n;
    unsigned long data2 = x;
    sendData(data1);
    sendData(data2);
    n = n+7;
    x= x+7;
    if(n>0)
    {
      L = (int)(T_TOTAL/n);
      if(L>65535)
        L = 0;
      else
        L = 65535-L;
      
      TIMSK1 |= (1 << TOIE1);
    }
    else
    {
      
      TIMSK1 &= ~(1 << TOIE1);
    }
    dataReaded = 0;
  }
}
