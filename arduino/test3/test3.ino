
int i = 0;
void setup() {
  // put your setup code here, to run once:
DDRB = B00000001;
PORTB = B00000001;
}

void loop() {
  // put your main code here, to run repeatedly:

if(i<3000)
{
  i = i+1;
  PORTB = B00000001;
  delayMicroseconds(5);
  PORTB = B00000000;
  delayMicroseconds(5);
}
}
