#include <Servo.h>

Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo
Servo myservo3;  // create servo object to control a servo
Servo myservo4;  // create servo object to control a servo

Servo myServos[4];

int x=90;
int oldx=90;
int fingers[5]={90,90,90,90,90};
int pins[4]={11,10,9,3};
int current[4]={90,90,90,90};


void setup() {
  Serial.begin(115200);

  for (int nServo=0;nServo<4;nServo++)
  {
    myServos[nServo].attach(pins[nServo]);
  }
  
  // Serial.begin(9600);
  Serial.setTimeout(2);
}

void  loop() {
  //while (!Serial.available());
  if(Serial.available())
  {
    // x = Serial.readString().toInt();
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Split the string into finger values
    
    int index = 0;

    for (int i = 0; i < input.length(); i++) {
      int commaIndex = input.indexOf(',', i);
      if (commaIndex == -1) commaIndex = input.length();
      String value = input.substring(i, commaIndex);
      int thisValue= value.toInt();
      
      
      /*if((thisValue>=90) && (thisValue<=160))
      {
        fingers[index] = thisValue; // Direction 1
        //fingers[index] = 180 - thisValue; // Direction 2
      }*/

      if((thisValue>=20) && (thisValue<=160))
      {
        fingers[index] = thisValue; // Direction 1
        //fingers[index] = 180 - thisValue; // Direction 2
      }
      
      index++;

      i = commaIndex;
      if (index >= 5) break;
    }
  }
    for (int nServo=0;nServo<4;nServo++)
    {
      int dif=fingers[nServo]-current[nServo];
      int jump=1;
      if(dif>jump)
      {
        current[nServo]=current[nServo]+jump;
      }
      else if(dif<-jump)
      {
        current[nServo]=current[nServo]-jump;
      }
      else
      {
        current[nServo]=current[nServo]+dif;
      }
      myServos[nServo].write(current[nServo]);
    }
    delay(5);
}