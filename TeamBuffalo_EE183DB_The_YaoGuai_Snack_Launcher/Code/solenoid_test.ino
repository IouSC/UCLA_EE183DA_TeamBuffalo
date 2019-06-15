

void setup() 
{ 
  pinMode(40,OUTPUT);
  pinMode(41,OUTPUT);
  digitalWrite(40,LOW);
  digitalWrite(41,LOW);
} 

void loop() 
{
  digitalWrite(40,HIGH);
  delay(200);
  digitalWrite(40,LOW);
  delay(1000);
  } 
