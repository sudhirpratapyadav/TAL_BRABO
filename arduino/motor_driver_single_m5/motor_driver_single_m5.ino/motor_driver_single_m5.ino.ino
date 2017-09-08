
#define DATA_SIZE 11 //in bytes
#define T_TOTAL 80000  //in microseconds
#define DE_ACC 500
#define MIN_L 1000

volatile int n = 0;
volatile int p = 0;
volatile unsigned int L = 0;
volatile int ind = 0;
volatile byte dataReaded = 0;
volatile byte data[11];
volatile byte STOP_MOTION = 0;
volatile int t2c = 0; 


void delay_6_micros()
{
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
}


ISR(TIMER2_OVF_vect)          // timer 2 overflow interrupt service routine
{
  t2c = t2c+1;
  if(t2c==5)
    STOP_MOTION = 1;
}

ISR(TIMER1_OVF_vect)          // timer 1 overfolw interrupt service routine
{
  if(STOP_MOTION)
  {
    PORTB = PORTB|B00100000;
    PORTB = PORTB^B00000001;
    if(L<MIN_L)
    {
      TIMSK1 &= ~(1 << TOIE1);
      TCCR1B = 0x00;
    }
    else
    {
      L = L -DE_ACC;
      TCNT1 = L;
    }
  }
  else
  {
    PORTB = PORTB^B00000001;
    if((PORTB&B00000001)==1)
    {
      n = n-1;
    }
    TCNT1 = L;
  }
}


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
  

  noInterrupts();           // disable all interrupts

  //TIMER 1 SETUP
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;            // preload timer 65536-16MHz/256/2Hz
  TCCR1B |= (1 << CS10);    // 1 prescaler 
  //TIMSK1 |= (1 << TOIE1);


  //TIMER 2 SETUP
  TCCR2B = 0x00;        //Disbale Timer2 while we set it up
  TCNT2  = 0;           //Reset Timer Count to 0 out of 255
  TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK2 = 0x00;        //Timer2 INT Reg: Timer2 Overflow Interrupt Disable
  TCCR2A = 0x00;        //Timer2 Control Reg A: Wave Gen Mode normal
  TCCR2B = 0x07;        //Timer2 Control Reg B: Timer Prescaler set to 1024
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
    TCNT2  = 0;           //Reset Timer 2 Count to 0 out of 255
    TIMSK2 = 0x01;
    t2c = 0;
    p = data[8]<<8|data[9];
    int enable = (data[10]>>5)&(B00000001);
    byte dir = (byte)((data[10]>>3)&(B00000010));
    
    if(dir!=(PORTB&B00000010))
    {
      
      if(n-p>0)
      {
        //No change in direction
        n = n - p;
      }
      else
      {
        //Flip Direction
        PORTB = dir;
        delay_6_micros();
        n = p - n;
      }
    }
    else
    {
      n = n+p;
    }
    if(n>0)
    {
      L = T_TOTAL/n;
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
    if(STOP_MOTION)
    {
      TCNT1 = L;
      TCCR1B |= (1 << CS10);    // 1 prescaler 
      STOP_MOTION = 0;
      PORTB = PORTB&(B11011111);
    }
    dataReaded = 0;
   interrupts();
  }
}
