
#define DATA_SIZE 11 //in bytes
#define T_TOTAL 80000.0  //in microseconds

volatile int n = 0;
volatile int p = 0;
volatile unsigned int L = 0;
volatile int ind = 0;
volatile byte dataReaded = 0;
volatile byte data[11];

void delay_6_micros()
{
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
}


ISR(TIMER1_OVF_vect)          // timer compare interrupt service routine
{
  PORTB = PORTB^B00000001;
  if((PORTB&B00000001)==1)
  {
    n = n-1;
  }
  TCNT1 = L;
}


void setup()
{
  //port setup (IO pins)
  DDRB = DDRB | B00000111;  //pins (_,_,13,12,11,10,9,8)
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
    noInterrupts();
    PORTB = PORTB|B00000100;
    p = data[0]<<8|data[1];
    int enable = (data[10]>>5)&(B00000001);
    byte dir = (byte)((data[10]<<1)&(B00000010));
    
    //if(dir!=(PORTB&B00000010))
    //{
    //  PORTB = dir;
    //  delay_6_micros();
    //}
    
    
    sendData(n);
    sendData(L);
    n = n+p;
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
    PORTB = PORTB&(~B00000100);
   interrupts();
  }
}
