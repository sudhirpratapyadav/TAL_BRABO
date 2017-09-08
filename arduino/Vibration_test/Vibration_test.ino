
#define DELAY_T 667

void setup() {
DDRB = DDRB | B00100111;  //direction pins (_,_,13,12,11,10,9,8)
PORTB = B00000000;
}

void loop() {
  int n = 20;
  int i = 0;
  int j = 0;
  PORTB = B00000000;
  delayMicroseconds(6);
  while(j<54)
  {
  i = 0;
  while(i<n)
  {
    PORTB = B00000001;
    delayMicroseconds(DELAY_T);
    PORTB = B00000000;
    delayMicroseconds(DELAY_T);
    i = i+1;
  }
  delayMicroseconds(DELAY_T);
  i = 0;
  while(i<n-1)
  {
    PORTB = B00000001;
    delayMicroseconds(DELAY_T);
    PORTB = B00000000;
    delayMicroseconds(DELAY_T);
    i = i+1;
  }
  PORTB = B00000001;
  delayMicroseconds(DELAY_T);
  delayMicroseconds(DELAY_T);
  PORTB = B00000000;
  delayMicroseconds(DELAY_T);
  j = j+1;
  }


  PORTB = B00000010;
  delayMicroseconds(6);
  j = 0;
  while(j<54)
  {
  i = 0;
  while(i<n)
  {
    PORTB = B00000011;
    delayMicroseconds(DELAY_T);
    PORTB = B00000010;
    delayMicroseconds(DELAY_T);
    i = i+1;
  }
  delayMicroseconds(DELAY_T);
  i = 0;
  while(i<n)
  {
    PORTB = B00000011;
    delayMicroseconds(DELAY_T);
    PORTB = B00000010;
    delayMicroseconds(DELAY_T);
    i = i+1;
  }
  PORTB = B00000011;
  delayMicroseconds(DELAY_T);
  delayMicroseconds(DELAY_T);
  PORTB = B00000010;
  delayMicroseconds(DELAY_T);
  j = j+1;
  }
}

