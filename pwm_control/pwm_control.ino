#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x3C
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN 110 //125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 495 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;
int motor_angle[6] = {90,0,45,90,90,90}; 
bool invert[6] = {0,0,1,0,0,0};
//Serial data
boolean newData = false;
const byte numChars = 32;
char receivedChars[numChars]; 

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50); 
}

// the code inside loop() has been updated by Robojax
void loop() {
   recvWithEndMarker();
   showNewData(); 
   if (newData){
      parseData();   
      send_all_PWM(); 
   }
}

void send_all_PWM(){
   int pulse; 
   for(int m = 0; m<6; m++){
      if(invert[m]){
        pulse = map(180-motor_angle[m],0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
      }else{
        pulse = map(motor_angle[m],0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max         
      }
      pwm.setPWM(m, 0, pulse);
      delay(5);
      
      Serial.print(" Motor: ");Serial.print(m);
      Serial.print(" Angle: ");Serial.print(motor_angle[m]);
      Serial.print(" pulse: ");Serial.println(pulse);
   }
}

void recvWithEndMarker()
{
   static byte ndx = 0;
   char endMarker = '\n';
   char rc;

   while (Serial.available() > 0 && newData == false)
   {
      rc = Serial.read();

      if (rc != endMarker)
      {
         receivedChars[ndx] = rc;
         ndx++;
         if (ndx >= numChars)
         {
            ndx = numChars - 1;
         }
      }
      else
      {
         receivedChars[ndx] = '\0'; // terminate the string
         ndx = 0;
         newData = true;
      }
   }
}
void showNewData()
{
   if (newData == true)
   {
      Serial.print("This just in ... ");
      Serial.println(receivedChars);
      //newData = false;
   }
}

void parseData()
{
   char *strings[8]; // an array of pointers to the pieces of the above array after strtok()
   char *ptr = NULL; byte index = 0;
   ptr = strtok(receivedChars, ",");  // delimiters, semicolon
   while (ptr != NULL)
   {
      strings[index] = ptr;
      index++;
      ptr = strtok(NULL, ",");
   }
   
   for(int m = 0; m<6; m++){
      if(m<=index){
        int val = atoi(strings[m]);
        if(val>180){
          motor_angle[m] = 180; 
        }else if(val<0){
          motor_angle[m] = 0; 
        }else{
        }
          motor_angle[m] = val; 
        }
    }
   // Inverse motor 3. 
   int min_q2 = max(motor_angle[1]-45,0);
   int max_q2 = min(motor_angle[1]+35, 180);
   motor_angle[2] = max(min_q2,min(motor_angle[2],max_q2));   
   newData = false;
}
