#include <LineFollowerPID.h>
#include <mDriver.h>

String message = "";

// Motor pins
#define AIN1  4
#define BIN1  6
#define AIN2 5
#define BIN2 7
#define PWMA 9
#define PWMB 10


// IR pins
#define extremeLeft A4
#define centerLeft A3
#define center A2
#define centerRight A1
#define extremeRight A0

// Correction factors for easily modifying motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int correctionA =  1;
const int correctionB =  1;

// Initializing motors.  The library will allow you to initialize as many motors as you have memory for.
Motor leftMotor = Motor(AIN1, AIN2, PWMA, correctionA);
Motor rightMotor = Motor(BIN1, BIN2, PWMB, correctionB);

// Motor speeds
int lsp, rsp;
int lfspeed = 255; // standard speed can be modified later

// PID constants
float Kp = 0;
float Kd = 0;
float Ki = 0;

// set-point variable
int sp = 0; 
PID carPID(Kp, Ki, Kd); // PID object

// color map values
int minValues[5], maxValues[5], threshold[5], sensors[5];

void setup()
{
  Serial.begin(9600);
  Serial.println("Car Started! Ready to pair...");
  
  carPID.setSpeeds(lfspeed);
  carPID.setConstrains(0, 255);

  Kp = 0.1257;
  Kd = 0.1;
  Ki = 0.001;
  carPID.setConstants(Kp, Ki, Kd);
  /***********************************/
  pinMode(LED_BUILTIN, OUTPUT);
  
  sensors[0] = extremeLeft;
  sensors[1] = centerLeft;
  sensors[2] = center;
  sensors[3] = centerRight;
  sensors[4] = extremeRight;

  for(int i = 0; i < 5; i++)
  {
    pinMode(sensors[i], INPUT);
  }
  
carPID.calibrate(leftMotor, rightMotor, minValues, maxValues, threshold, sensors); // calibration mode
delay(2000) ;
}


void loop()
{ 



    // Extreme left turn when extremeLeft sensor detects dark region while extremeRight sensor detects white region
    if (analogRead(extremeLeft) > threshold[0] && analogRead(extremeRight) < threshold[4] )
    {
      //Serial.println("left");
      lsp = 0; rsp = lfspeed;
      leftMotor.drive(0);
      rightMotor.drive(rsp);
      //Serial.println("left");
      //left(leftMotor, rightMotor, lfspeed);
    }

    // Extreme right turn when extremeRight sensor detects dark region while extremeLeft sensor detects white region
    else if (analogRead(extremeRight) > threshold[4] && analogRead(extremeLeft) < threshold[0])
    { 
      //Serial.println("right");
      lsp = lfspeed; rsp = 0;
      leftMotor.drive(lsp);
      rightMotor.drive(0);
      //Serial.println("right");
      //right(leftMotor, rightMotor, lfspeed);
    }
    else if (analogRead(center) > threshold[2])
    {
      // arbitrary PID constans will be tuned later
      // forward(leftMotor, rightMotor);

      // Serial.print("center\t");
      // Serial.println(Kp);
      // carPID.setConstants(Kp, Ki, Kd);
      sp = (analogRead(centerLeft) - analogRead(centerRight));
      //Serial.print("error\t");
      //Serial.println(sp);
      carPID.setSetpoint(sp);
      carPID.linefollow(leftMotor, rightMotor, lsp, rsp);
    }
}
