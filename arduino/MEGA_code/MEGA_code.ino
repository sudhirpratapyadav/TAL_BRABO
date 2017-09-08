
#define DATA_SIZE 11 //in bytes
byte SENSOR_VAL;
byte data[11];
int ind = 0;
byte dataReaded = 0;

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  Serial1.begin(115200);
  Serial.begin(115200);
  DDRC = B00000000;  //pins (30,31,32,33,34,35,36,37)
  SENSOR_VAL = 0;
  pinMode(13,OUTPUT);

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
  digitalWrite(13,HIGH);
  delay(500);
  digitalWrite(13,LOW);
}


void serialEvent()
{
  while(Serial.available())
  {
    data[ind] = Serial.read();
    ind = ind +1;
    if(ind==DATA_SIZE)
    {
      ind = 0;
      dataReaded = 1;
    }
  }
}

void loop() {
  if (dataReaded)
  {
    noInterrupts();
    Serial1.write(data,DATA_SIZE);
    SENSOR_VAL = PINC;
    Serial.write((SENSOR_VAL&B00011111)^B00010100);
    interrupts();
    dataReaded = 0;
  }
}
