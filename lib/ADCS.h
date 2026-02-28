#define  Trig A4 
#define  Echo A5

const int offset = 13;

float GetDistance()
{
   digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);

  int duration = pulseIn(Echo, HIGH);
  int distance = (duration*.0343)/2;
  if (distance < 0)
  {
   distance = 400;
  }
	return distance;
}

//Motor 1 output definition
void motorL(int steep1, int steep2) 
{
   analogWrite(5,steep1);
   analogWrite(6,steep2);
}

//Motor 2 output definition
void motorR(int steep1, int steep2)
{
   analogWrite(9,steep1);
   analogWrite(10,steep2);
}
