  /*
  Motor Controller software
  This software is due to interface the computer  and the motor controller, receiving
  the signals from the encoders and doing the system control.
 
 
 
  //the content below is to be deleted when ...
  To test the loop function, that is, the PID itself, just set desired values for
  Target speed and Target position at the beggining of the loop.
 
  to test the serial communication, adapt the loop code to blink a led, and you can use
  your own computer o test it at first
  */
 
 
 
unsigned long currentTime_1;
unsigned long loopTime=0;
const int pin_G_A = 2;  
const int pin_G_B = 4;
const int pin_D_A = 3;
const int pin_D_B = 1;  
unsigned int encoder_A;
unsigned int encoder_G_B;
unsigned int encoder_D_B;
unsigned int encoder_A_prev=0;
int compteur = 0;
int compteur_prev =0;

  // associating input pins constants - depending on the microcontroller (ATMega328; check Arduino ATMega328 pinout for details)
  #define Left_INT 0 // correspond à la voie 3
  #define Right_INT 1 // correspond à la voie 2
  #define Left_A A4
  #define Left_B A5
  #define Right_A A3 //c'etait A2
  #define Right_B A2 //c'etait A1
 
  // associating output pins constants - depending on the microcontroller (ATMega328; check Arduino ATMega328 pinout for details)
  #define Status_Blue A3
  #define Status_Red A2
  #define EN_L 7
  #define PWM_L 5
  #define EN_R 8
  #define PWM_R 6
  #define Serial_Send 0 //mod
 
  // communication constants
  #define UART_BAUD 19200
 
  // motor control constants
  #define FORWARD 0
  #define REVERSE 1
  #define STOP 2
  #define LEFT 0
  #define RIGHT 1
 
  // PID mode constants
  #define Coord_PD 0
  #define Speed_PD 1
  #define Coord_PID 2
  #define Speed_PID 3
 
  // other constants
  #define m_pi 3.14159
 
  // robot parameters variables
  int trackWidth, wheelDiameter, cpr, motorWheelDiameter;
  int leftWheelDiameter, rightWheelDiameter;
 
  // ### UART communication variables
  // receive
  unsigned char CARD_TYPE = 128;
  unsigned char CARD_INDEX = 128;
  unsigned char cardType, cardIndex, cardCommand, cardArgCount;
  unsigned char cardArg[16];
  // send
  unsigned char replyCommand, replyArgCount;
  unsigned char replyArg[16];
 
  bool doingSerial = false;
 
  // current location variables
  volatile long leftClicks, rightClicks;
  long deltaClicks;
  long lastLeftClicks, lastRightClicks;
  long deltaLeftClicks, deltaRightClicks;
  float deltaAvgClicks;
  float angle, tempAngle, lastAngle, avgAngle, tempAngleLeft, tempAngleRight;
  long intAngle;
  float xClicks, yClicks;
  float x, y;
  long xInt, yInt;
  long deltaForwardLeft, deltaForwardRight, deltaForward;
  long intDeltaAngle;
  float deltaAngle, deltaAngleLeft, deltaAngleRight;
 
  bool hasArrived;
 
  // ### PID variables ###
  int PIDmode;
  boolean PIDautoswitch;
  long lastPWM;
  long errorThresholdSlow, errorThresholdStop, errorThresholdHalf; //j'ajoute errorThresholdHalf
 
  // coordinate PID
  long leftTarget, rightTarget;
  long leftError, rightError;
  long lastLeftError, lastRightError;
  long leftDer, rightDer;
  long newLeftTarget, newRightTarget;
 
  const int kPcoord = 30; // WARNING: the value is divided by 1024 //c'etait 5
  const int kDcoord = 1000; // WARNING: the value is divided by 1024 //c'etait 600
 
  // speed PID
  long leftSpeed, rightSpeed;
  long leftTargetSpeed, rightTargetSpeed;
  long leftSpeedError, rightSpeedError;
  long lastLeftSpeedError, lastRightSpeedError;
  long leftSpeedDer, rightSpeedDer;
  long leftSpeedInt, rightSpeedInt;


  unsigned long currentTime, lastTime, deltaTime;
 
  const int kPspeed = 20; // WARNING: the value is divided by 1024 //c'etait 25
  const int kDspeed = 2500; // WARNING: the value is divided by 1024 //c'etait 1000
  const int kIspeed = 1; // WARNING: the value is divided by 1024
 
  const long maxSpeedInt = 1024 / kIspeed + 1;
  const long maxIntError = 200;
 
  // robot behaviour global variables
  long leftPWM, rightPWM;
  long leftBigPWM, rightBigPWM;
  long maxLeftPWM, maxRightPWM;
  int maxSpeedC, maxAccC;
 
  // temporary variables
  bool statusBlue = false;
  unsigned char i;
  float tempPWM;
  float tempDelta;
  int tempPWMsign;
  long tempSpeed, intSpeed, extSpeed;
  long tempRadius, intRadius, extRadius;
  long intClicks, extClicks;
  long tempAngleCircleInt;
  float tempAngleCircle;
  bool tempRotationMode;
  float intDistance, extDistance;

  void incr_right(){
 encoder_G_B = digitalRead(pin_G_B);
 if(encoder_G_B) {
        // B is high so clockwise
        leftClicks ++;
     
        
      }   
      else {
        // B is low so counter-clockwise      
     
        leftClicks--;    }}


  void incr_left(){
 encoder_D_B = digitalRead(pin_D_B);
 if(encoder_D_B) {
        // -B is high so cclockwise
        rightClicks --;
      }   
      else {
        // -B is low so clockwise      
        rightClicks ++;
      }
  }
        
  // ##### The initialization function, runs once at startup. #####
    
  void setup()
  {
    //capteurs
     Serial.begin(9600);

    pinMode(pin_G_A,INPUT);
    pinMode(pin_G_B,INPUT);
    pinMode(pin_D_A,INPUT);
    pinMode(pin_D_B,INPUT);
    
    
    // Robot construction values
    cpr = 10000; // number of counts per revolution of the encoder //c'etait 5000
    wheelDiameter = 72; // ENCODER wheel diameter in milimeters
    leftWheelDiameter = 72; // LEFT ENCODER wheel diameter in milimeters
    rightWheelDiameter = 72; // RIGHT ENCODER wheel diameter in milimeters
    trackWidth = 338.75; // the distance between the wheels in milimeters //c'etait 250
    motorWheelDiameter = 100; // MOTOR wheel diameter in milimeters
 
    // configuring I/O digital ports
    //pinMode(Left_A, INPUT);  // encoder A left input
    //pinMode(Left_B, INPUT);  // encoder B left input
    //pinMode(Right_A, INPUT);  // encoder A right input
    //pinMode(Right_B, INPUT);  // encoder B right input
 
    pinMode(Status_Red, OUTPUT); // UART Status LED output
    pinMode(Status_Blue, OUTPUT); // Loop Status LED output
    pinMode(Serial_Send, OUTPUT); // UART Send Enable output
 
    pinMode(PWM_L, OUTPUT);  // PWM left output
    pinMode(PWM_R, OUTPUT);  // PWM right output
    pinMode(EN_L, OUTPUT);  // direction left output
    pinMode(EN_R, OUTPUT);  // direction right output
 
    // overriding the PWM frequency; we have to use a demultiplication factor of 1 in order to achieve an ultrasonic frequency and avoid the annoying buzzing
    // !!! WARNING !!! the default prescaling factor was 64 and the use of this function influences the delay(), millis(), micros() functions
    // the delay() will have to be 64x higher, while the actual values of millis() and micros() are 64x lower
    // setPwmFreq(PWM_L, 1);
    // setPwmFreq(PWM_R, 1);
 
    // PID parameters - default values - DO NOT TOUCH
    leftError = 0;
    rightError = 0;
    lastLeftError = 0;
    lastRightError = 0;
    leftDer = 0;
    rightDer = 0;
    leftPWM = 0;
    rightPWM = 0;
    leftClicks = 0;
    rightClicks = 0;
    leftTarget = 0;
    rightTarget = 0;
 
    // attaching the interrupt functions for the wheel encoders
    attachInterrupt(Right_INT, incr_right, FALLING);
    attachInterrupt(Left_INT, incr_left, FALLING);
    
    // beginning the UART communication
    Serial.begin(UART_BAUD);
    Serial.setTimeout(1000);

    PIDmode = Coord_PD;
 
    errorThresholdSlow = (long) (cpr / 2);//by default the thresholder is half of a circle - you can chang it
    errorThresholdStop = 500;//by default the thresholder is one fifth of a circle //c'etait 200
    PIDautoswitch = false;
 
    // setting maximum PWM values
    if (PIDmode == Speed_PD || PIDmode == Speed_PID)
    {
      maxLeftPWM = 128 + 36 * (leftTargetSpeed + rightTargetSpeed) / 40;
      maxRightPWM = maxLeftPWM;
    }
    else
    {
      maxLeftPWM = 160; //c'etait 180, j'ai mis 160 pour qu'il n'aille pas trop vite
      maxRightPWM = 160; //c'etait 180, j'ai mis 160 pour qu'il n'aille pas trop vite
    }

 
/*  long turnss = 2;
  leftTarget  = turnss*cpr;
  rightTarget = turnss*cpr;
 
 
  Serial.print("leftTarget: ");
  Serial.println(leftTarget);
  Serial.println();
 
  leftTargetSpeed  = 30;
  rightTargetSpeed = 30;
 
    PIDmode = Coord_PD;
    PIDmode   = Speed_PD;*/
  }
 
  void leftUpdate() // update the encoder values for the LEFT motor
  {
      //Serial.println("leftupdate");
      if(PIND & B00000100)//Thats equivalent to digitalRead, only we read it inside the registers, so as it's much faster. Check Port Manipulation out
      {
          
          if (PIND & B00010000) leftClicks++;
          else leftClicks--;
      }
      else
      {
          if (PIND & B00010000) leftClicks--;
          else leftClicks++;
      }
 
  }

  void rightUpdate() // update the encoder values for the RIGHT motor
  { 
    
    if(PIND & B00001000)//left A//if ((PINC>>PINC0) & 1) // equivalent to "if (PINC.0 == 1)"
    {
      
        if (PIND & B00000010) rightClicks--; // left B // equivalent to "if (PINC.1 == 1)"
        else rightClicks++;
    }
    else
    {
        
        
        if (PIND & B00000010) {
          rightClicks++;
          
        } // Left B//equivalent to "if (PINC.1 == 1)"
        else{
         rightClicks--;
        
        }
    }
  }
 
  void setMotorDir(int motor, int dir) // set the motor direction -- REVIEWED
  {
    if (motor == LEFT)
    {
      if (dir == 0) digitalWrite(EN_L, LOW);//      FORWARD
      else if (dir == 1) digitalWrite(EN_L, HIGH);//REVERSE
      else if (dir == 2) digitalWrite(EN_L, LOW);//STOP
    }
    else if (motor == RIGHT)
    {
      if (dir == 0) digitalWrite(EN_R, HIGH);
      else if (dir == 1) digitalWrite(EN_R, LOW);
      else if (dir == 2) digitalWrite(EN_R, LOW);
    }
  }
 
  void setMotor(int motor, int dir, int speed) // set the motor parameters -- REVIEWED
  {
    setMotorDir(motor, dir);
    if (speed < 0) speed = 0;
    if (speed > 255) speed = 255;
    if (motor == LEFT) analogWrite(PWM_L, speed);
    else if (motor == RIGHT) analogWrite(PWM_R, speed);
  }
 /*
  void setPwmFreq(int pin, int divisor) // function used to override the PWM frequency setting used by default by Arduino
  {
    byte mode;
    if (pin == 5 || pin == 6 || pin == 9 || pin == 10)
    {
      switch (divisor)
      {
        case 1: mode = 0x01; break;
        case 8: mode = 0x02; break;
        case 64: mode = 0x03; break;
        case 256: mode = 0x04; break;
        case 1024: mode = 0x05; break;
        default: return;
      }
 
      if (pin == 5 || pin == 6) TCCR0B = TCCR0B & 0b11111000 | mode;
      else TCCR1B = TCCR1B & 0b11111000 | mode;
    }
    else if (pin == 3 || pin == 11)
    {
      switch (divisor)
      {
        case 1: mode = 0x01; break;
        case 8: mode = 0x02; break;
        case 32: mode = 0x03; break;
        case 64: mode = 0x04; break;
        case 128: mode = 0x05; break;
        case 256: mode = 0x06; break;
        case 1024: mode = 0x7; break;
        default: return;
      }
      TCCR2B = TCCR2B & 0b11111000 | mode;
    }
  }
  */

  //J'ajoute la fonction GetCurrentCoord()
  //en fait cette fonction est écrit explicitement dans le loop(), mais je veux ecrire
  //dans une fonction pour qu'il soit propre.
  void GetCurrentCoord()
  {
    float derive_x, derive_y;
    tempAngleLeft = (float)leftClicks * (float)m_pi * (float)wheelDiameter;
    tempAngleLeft = tempAngleLeft / (float)cpr;
    float L = leftClicks;
    tempAngleLeft = tempAngleLeft / (float) trackWidth;
    tempAngleRight = (float)rightClicks * (float)m_pi * (float)wheelDiameter;
    tempAngleRight = tempAngleRight / (float)cpr;
    tempAngleRight = tempAngleRight / (float) trackWidth;

    angle = tempAngleRight - tempAngleLeft;
    while (angle < -m_pi) angle = angle + 2 * m_pi;
    while (angle > m_pi) angle = angle - 2 * m_pi;
    L = L + ((trackWidth/2) * cpr) / ((wheelDiameter * m_pi) * tan(angle)); //exprime en Clicks
    //L = L + (trackWidth/2)/tan(angle)
    deltaAngle = angle - lastAngle;
    //***A little filter to decrease "noise angles" ( take the average )***
    avgAngle = (angle + lastAngle) / 2.0; //en radians
    lastAngle = angle;

    //***calculating delta clicks, since last loop iteration***//
    deltaLeftClicks = leftClicks - lastLeftClicks;
    deltaRightClicks = rightClicks - lastRightClicks;
    lastLeftClicks = leftClicks;
    lastRightClicks = rightClicks;

    deltaAvgClicks = ((float)deltaLeftClicks + (float)deltaRightClicks)/2;

    float K;
    //I added this condition so that xClicks and yClicks don't increase when deltaAvgClicks = 0
    //deltaAvgClicks = 0 means that the robot standstills
    if (deltaAvgClicks == 0) {
      K = 0;
    }
    else if(deltaAngle = 0){
      K = 1;
    }
    else {
      K = sin(deltaAngle/2) / (deltaAngle/2);
    }

    //calcul dx et dy
    dx = K * L * cos(avgAngle);
    dy = k * L * sin(avgAngle);
  
    //J'ajoute la correction centrefuge pour corriger de manniere plus precis
    //Quand le robot se deplace sur une courbe, il y a la force centrefuge qui
    //joue un rôle dans l'error de deplacement. En supposant que le deplacement
    //latéral est proportionnel à cette force, on peut ecrire delta_epsilon = Kc*v*delta_angle
    //ou Kc est le coefficient à determiné experimentalement, v la vitesse correspond à L
    //delta_epsilon est porté par u_theta et égale à dérive_x + dérive_y
  
    float Kc = 0; // À déterminé expérimentalement. Il dépend de masse aussi
    derive_x = Kc * deltaAngle * dy; //c'est dy car derive_x = Kc * L * sin(avgAngle)
    derive_y = -Kc * deltaAngle * dx; //c'est dx car derive_y = Kc * L * cos(avgAngle

    xClicks = xClicks + dx + derive_x;
    yClicks = yClicks + dy + derive_y;

    Serial.print("Coordination : ");
    Serial.print(xClicks); Serial.print(" ,");
    Serial.println(yClicks);
    Serial.print("Orientation : "); Serial.println(avgAngle);
  }

  //J'ajoute cette fonction pour la meme raison que GetCurrentCoord()
  void PD_Coord()
  {
    //computing the errors for each motor
    leftError = leftTarget - leftClicks;
    rightError = rightTarget - rightClicks;
    //Why the robot turn when I push it, even though the target is zero?
    //because when I push it, the leftClicks and rightClicks change;
    //when they change leftError and rightError also change.
    //so the code works as we want it to be.

    // computing the derivatives for each motor
    leftDer = leftError - lastLeftError;
    rightDer = rightError - lastRightError;

    // updating the last error
    lastLeftError = leftError;
    lastRightError = rightError;

    // actual PID calculus
    leftPWM = (kPcoord * leftError + kDcoord * leftDer) / 1024;
    rightPWM = (kPcoord * rightError + kDcoord * rightDer) / 1024;

    // compensating the non-linear dependency speed = f(PWM_Value)
    tempPWM = (float) abs(leftPWM) / 255.0;
    tempPWMsign = leftPWM / abs(leftPWM);
    tempPWM = pow(tempPWM, 0.2);
    tempPWM = 255.0 * tempPWM;
    leftPWM = (int) tempPWM * tempPWMsign;

    tempPWM = (float) abs(rightPWM) / 255.0;
    tempPWMsign = rightPWM / abs(rightPWM);
    tempPWM = pow(tempPWM, 0.2);
    tempPWM = 255.0 * tempPWM;
    rightPWM = (int) tempPWM * tempPWMsign;
}

  //j'ajoute ces fonctions pour la meme raison que GetCurrentCoord()
  void arrived()
  {
    if((leftError < errorThresholdStop) && (rightError < errorThresholdStop)) hasArrived = true;
    else hasArrived =false;
  }

void PD_Speed()
{
  // computing the time interval
  currentTime = micros();
  deltaTime = currentTime - lastTime;
  lastTime = currentTime;
  deltaTime = deltaTime / 64;

  // computing the current speed in clicks / millisecond
  leftSpeed = deltaLeftClicks * 1000 / (long) deltaTime;
  rightSpeed = deltaRightClicks * 1000 / (long) deltaTime;

  // computing the speed error
  leftSpeedError = leftTargetSpeed - leftSpeed;
  rightSpeedError = rightTargetSpeed - rightSpeed;

  // computing the speed error derivative
  leftSpeedDer = leftSpeedError - lastLeftSpeedError;
  rightSpeedDer = rightSpeedError - lastRightSpeedError;

  // updating the last error value
  lastLeftSpeedError = leftSpeedError;
  lastRightSpeedError = rightSpeedError;

  // computing the errors for each motor -- useful for the autoswitch
  lastLeftError = leftTarget - leftClicks;
  lastRightError = rightTarget - rightClicks;

  // actual PID calculus
  leftBigPWM = leftBigPWM + (kPspeed * leftSpeedError + kDspeed * leftSpeedDer) / 16;
  rightBigPWM = rightBigPWM + (kPspeed * rightSpeedError + kDspeed * rightSpeedDer) / 16;

  leftPWM = leftBigPWM / 64;
  rightPWM = rightBigPWM / 64;

  lastPWM = (abs(leftPWM) + abs(rightPWM)) / 2;

}

void PID_Speed()
{
  // computing the time interval
  currentTime = micros();
  deltaTime = currentTime - lastTime;
  lastTime = currentTime;
  deltaTime = deltaTime / 64;

  // computing the current speed in clicks / millisecond
  leftSpeed = deltaLeftClicks * 1000 / (long) deltaTime;
  rightSpeed = deltaRightClicks * 1000 / (long) deltaTime;

  // computing the speed error
  leftSpeedError = leftTargetSpeed - leftSpeed;
  rightSpeedError = rightTargetSpeed - rightSpeed;

  // computing the speed error derivative
  leftSpeedDer = leftSpeedError - lastLeftSpeedError;
  rightSpeedDer = rightSpeedError - lastRightSpeedError;

  // updating the last error value
  lastLeftSpeedError = leftSpeedError;
  lastRightSpeedError = rightSpeedError;

  // limiting the error that will be added to the integral error
  if ((leftSpeedError > -maxIntError) && (leftSpeedError < maxIntError)) leftSpeedInt = leftSpeedInt + leftSpeedError;
  if ((rightSpeedError > -maxIntError) && (rightSpeedError < maxIntError)) rightSpeedInt = rightSpeedInt + rightSpeedError;

  // limiting the integral error
  if (leftSpeedInt > maxSpeedInt) leftSpeedInt = maxSpeedInt;
  else if (leftSpeedInt < - maxSpeedInt) leftSpeedInt = - maxSpeedInt;
  else leftSpeedInt = 0;
  if (rightSpeedInt > maxSpeedInt) rightSpeedInt = maxSpeedInt;
  else if (rightSpeedInt < - maxSpeedInt) rightSpeedInt = -maxSpeedInt;
  else rightSpeedInt = 0;

  // actual PID calculus
  leftBigPWM = leftBigPWM + (kPspeed * leftSpeedError + kDspeed * leftSpeedDer + kIspeed * leftSpeedInt) / 16;
  rightBigPWM = rightBigPWM + (kPspeed * rightSpeedError + kDspeed * rightSpeedDer + kIspeed * rightSpeedInt) / 16;
  leftPWM = leftBigPWM / 64;
  rightPWM = rightBigPWM / 64;

  if (leftPWM > 255.0) leftPWM = 255.0;
  else if (leftPWM < -255.0) leftPWM = -255.0;
  if (rightPWM > 255.0) rightPWM = 255.0;
  else if (rightPWM < -255.0) rightPWM = -255.0;

  lastPWM = (abs(leftPWM) + abs(rightPWM)) / 2;

}

  //***implementing the PID autoswitch***
void PIDautoSwitch()
  {
    if ((leftError < errorThresholdStop) && (rightError < errorThresholdStop))
    {
      PIDmode = Coord_PD;
      maxLeftPWM = 160; //c'etait 180
      maxRightPWM = 160; //c'etait 180
      lastLeftSpeedError = 0;
      lastRightSpeedError = 0;
      PIDautoswitch = false;
    }
    else if ((leftError < errorThresholdHalf) && (rightError < errorThresholdHalf) && PIDautoslow)
    {
      if (leftTargetSpeed > 0)
      {
        if (leftTargetSpeed >= 18) leftTargetSpeed = leftTargetSpeed * 3 / 4;
        else leftTargetSpeed = 12;
      }
      else
      {
        if (leftTargetSpeed <= -18) leftTargetSpeed = leftTargetSpeed * 3 / 4;
        else leftTargetSpeed = -12;
      }

      if (rightTargetSpeed > 0)
      {
        if (rightTargetSpeed >= 18) rightTargetSpeed = rightTargetSpeed * 3 / 4;
        else rightTargetSpeed = 12;
      }
      else
        {
          if (rightTargetSpeed <= -18) rightTargetSpeed = rightTargetSpeed * 3 / 4;
          else rightTargetSpeed = -12;
      }
      PIDautoslow = false;
    }
    else if ((leftError < errorThresholdSlow) && (rightError < errorThresholdSlow) && PIDautopreswitch)
    {
      PIDmode = Coord_PD;
      if (lastPWM >= 128)
      {
        maxLeftPWM = lastPWM;
        maxRightPWM = lastPWM;
      }
      else
      {
        maxLeftPWM = 128;
        maxRightPWM = 128;
      }
      lastLeftSpeedError = 0;
      lastRightSpeedError = 0;
      lastLeftError = 0;
      lastRightError = 0;
      PIDautopreswitch = false;
    }
 }

// va tout droit avec la distance en millimètre
// et la vitesse en PWM entre 0-255
void GoStraight(int distance, int Speed) //#136
{
  tempDelta = (float) distance;
  tempDelta = tempDelta * (float) cpr / ((float)m_pi * (float) wheelDiameter);
  deltaForward = (long) tempDelta;

  leftTarget = leftClicks + deltaForward;
  rightTarget = rightClicks + deltaForward;

  leftTargetSpeed = Speed * (long) cpr;
  leftTargetSpeed = leftTargetSpeed * (long) motorWheelDiameter;
  leftTargetSpeed = leftTargetSpeed / 60000;
  leftTargetSpeed = leftTargetSpeed / (long) wheelDiameter;
  rightTargetSpeed = leftTargetSpeed;

  leftBigPWM = 0;
  rightBigPWM = 0;

  maxLeftPWM = 128 + 36 * abs(leftTargetSpeed + rightTargetSpeed) / 40;
  maxRightPWM = maxLeftPWM;
  lastPWM = maxLeftPWM;

  if (abs(deltaForward) > 10000)
  {
    errorThresholdSlow = (long) cpr / 2;
    errorThresholdHalf = (long) cpr;
  }
  else
  {
    errorThresholdSlow = abs(deltaForward) / 2;
    errorThresholdHalf = abs(deltaForward) * 3 / 4;
  }
  
  //PIDmode = Coord_PD;
  PIDmode = Speed_PD; //c'etait commenté
  PIDautoswitch = true; //c'etait commenté
  PIDautoslow = true; //c'etait commenté
  PIDautopreswitch = true;
}

//tourner autour soi-meme avec l'angle en radians
//et avec la vitesse en PWM entre 0-255
//il reste à le tester
void TurnRad(double angle, int Speed) //#137 angle in radians
{
 
 deltaAngle = (float)angle;
 deltaAngle *= (float)trackWidth * (float)cpr;
 deltaAngle /= (float)2*m_pi * (float)wheelDiameter;
 deltaForward = (long) deltaAngle;

 deltaForward = (long)deltaAngle;

 leftTarget = leftClicks - deltaForward;
 rightTarget = rightClicks - deltaForward;

 leftTargetSpeed = (long)Speed;
 leftTargetSpeed *= (long) cpr * (long)motorWheelDiameter;
 leftTargetSpeed /= (60000 * (long)wheelDiameter);

 if (deltaForward >= 0)
 {
   rightTargetSpeed = leftTargetSpeed;
   leftTargetSpeed = -leftTargetSpeed;  
 }
 else {rightTargetSpeed = -leftTargetSpeed;}
                    
 leftBigPWM = 0;
 rightBigPWM = 0;
                    
 maxLeftPWM = 128 + 36 * (abs(leftTargetSpeed) + abs(rightTargetSpeed)) / 40;
 maxRightPWM = maxLeftPWM;
 lastPWM = maxLeftPWM;
                    
 //if (abs(deltaForward) > 20000) errorThresholdSlow = (long) cpr;
 //else errorThresholdSlow = abs(deltaForward) / 2;

 if (abs(deltaForward) > 20000)
 {
   errorThresholdSlow = (long) cpr / 2;
   errorThresholdHalf = (long) cpr;
 }
 else
 {
   errorThresholdSlow = abs(deltaForward) / 2;
   errorThresholdHalf = abs(deltaForward) * 3 / 4;
 }

 PIDmode = Speed_PD;
 PIDautoswitch = true;
 PIDautoslow = false; // we will do whatever we usually do when we turn!
 PIDautopreswitch = true;                   
}

//À tester
void SetTargetSpeed(int leftRPM, int rightRPM) //#141
{
  leftTargetSpeed = leftRPM;
  rightTargetSpeed = rightRPM;
  leftTargetSpeed *= (long)cpr * (long)motorWheelDiameter;
  rightTargetSpeed *= (long)cpr * (long)motorWheelDiameter;
  leftTargetSpeed /= (60000 * (long)wheelDiameter);
  rightTargetSpeed /= (60000 * (long)wheelDiameter);
  leftBigPWM = 0;
  rightBigPWM = 0;
}

//À tester
void TurnCircle(int radius, float angle, int Speed) //#145
{
  while(angle < - m_pi) {angle += 2 * m_pi;}
  while(angle > m_pi) {angle -= 2 * m_pi;
  tempRadius = radius;
  tempSpeed = Speed;
  tempAngleCircle = angle;
  //tempRadius = 400;
  //tempSpeed = 60;
  //tempAngleCircle = (float) -M_PI / 4;

  // storing the rotation direction (false = left, true = right)
  if (tempRadius < 0)
  {
    tempRotationMode = false;
    tempRadius = 0-tempRadius;
  }
  else tempRotationMode = true;

  // computing the internal and external turning radius
  intRadius = tempRadius - (long) trackWidth/2;
  extRadius = tempRadius + (long) trackWidth/2;

  // computing the internal and external movement distances
  intDistance = (float) intRadius * tempAngleCircle;
  extDistance = (float) extRadius * tempAngleCircle;

  // computing the internal and external click counts
  tempDelta = (float) intDistance;
  tempDelta = tempDelta * (float) cpr / ((float) M_PI * (float) wheelDiameter);
  intClicks = (long) tempDelta;

  tempDelta = (float) extDistance;
  tempDelta = tempDelta * (float) cpr / ((float) M_PI * (float) wheelDiameter);
  extClicks = (long) tempDelta;

  // computing the average speed in clicks / second
  tempSpeed = tempSpeed * (long) cpr;
  tempSpeed = tempSpeed * (long) motorWheelDiameter;
  tempSpeed = tempSpeed / 60000;
  tempSpeed = tempSpeed / (long) wheelDiameter;

  // computing the internal and external speed
  intSpeed = tempSpeed * intRadius / tempRadius;
  extSpeed = tempSpeed * extRadius / tempRadius;

  // setting the movement to left or right
  if (tempRotationMode == true)
  {
    leftTarget = leftClicks + extClicks;
    rightTarget = rightClicks + intClicks;

    // setting the movement backwards or forwards
    if (tempAngleCircle > 0)
    {
      leftTargetSpeed = extSpeed;
      rightTargetSpeed = intSpeed;
    }
    else
    {
      leftTargetSpeed = -extSpeed;
      rightTargetSpeed = -intSpeed;
    }
  }
  else
  {
    leftTarget = leftClicks + intClicks;
    rightTarget = rightClicks + extClicks;

    // setting the movement backwards or forwards
    if (tempAngleCircle > 0)
    {
        leftTargetSpeed = intSpeed;
        rightTargetSpeed = extSpeed;
    }
    else
    {
        leftTargetSpeed = -intSpeed;
        rightTargetSpeed = -extSpeed;
    }
   }        

   // PID start
   errorThresholdSlow = (long) (cpr/2);

   PIDmode = Speed_PD;
   PIDautoswitch = true;
   
  } 
}

//cette fonction était écrite explicitement dans le loop()
//je l'écris ici pour qu'il soit propre et facile à corriger.
//il faut tester cette fonction, car je réécris de façons plus compact avec les appels
//aux fonctions que j'ai ajoutées en haut (en inspirant des codes de 2014 biensur :) )
//cette fonction permet au robot de rester immobile
//quand on le pousse à l'avance, le robot reculera
//même réaction quand nous le faisons tourner
void asservit()
{
  GetCurrentCoord();

  //**computing the errors for each motor**//
  leftError = abs(leftTarget - leftClicks);
  rightError = abs(rightTarget - rightClicks);
  
  arrived();//check if we have arrived

  // implementing the PID autoswitch
  if (PIDautoswitch == true) PIDautoSwitch();

  // ### PID computing ###
  switch (PIDmode)
  {
    case Coord_PD: // PD algorithm, target COORDINATES
      {
        PD_Coord();
        break;
      }
    case Coord_PID: // PID algorithm, target COORDINATES
      {
        break;
      }
    case Speed_PD: // PD algorithm, target SPEED
      {
        PD_Speed();
        break;
      }
    case Speed_PID: // PID algorithm, target SPEED -- NOT PROPERLY TUNED, might abandon
      {
        PID_Speed();
        break;
      }
  }

  // speed limiting (in order to avoid PWM values higher than the maximally set values)
  if (leftPWM < -maxLeftPWM) leftPWM = -maxLeftPWM;
  if (leftPWM > maxLeftPWM) leftPWM = maxLeftPWM;
  if (rightPWM < -maxRightPWM) rightPWM = -maxRightPWM;
  if (rightPWM > maxRightPWM) rightPWM = maxRightPWM;

  // setting the speed and direction parameters for the motors
  if (leftPWM < 0) setMotor(LEFT, FORWARD, -leftPWM);
  else setMotor(LEFT, REVERSE, leftPWM);
  if (rightPWM < 0) setMotor(RIGHT, FORWARD, -rightPWM);
  else setMotor(RIGHT, REVERSE, rightPWM);
}

//ce qui vient dans ma tête, pas encore testé
void asservit_angulaire()
{
  if(rightClicks == 0) return; //eviter diviser par zero
  if(leftClicks / rightClicks < -0.95 && leftClicks / rightClicks > -1.05) asservit();
  else return;
}

//ce qui vient dans ma tête, pas encore testé
void asservit_distance()
{
  //eviter diviser par zero
  if(rightClicks != 0 && ((leftClicks / rightClicks > -0.95) || (leftClicks / rightClicks < -1.05)))
  {
    asservit();
  }
  else return;
}


  bool checkValidData() // checking if the card type and index are the correct ones
  {
    if (cardType == CARD_TYPE && cardIndex == CARD_INDEX) return true;
    else
    {
      delay(32);
      while (Serial.available() > 0) Serial.read();
      return false;
    }
  }
 
  void sendData()
  {
    digitalWrite(Serial_Send, HIGH);
    Serial.write(CARD_TYPE);
    Serial.write(CARD_INDEX);
    Serial.write(replyCommand);
    Serial.write((unsigned char)(replyArgCount + 128));
    for (i = 0; i < replyArgCount; i++) Serial.write(replyArg[i]);
    delay(240 + 60 * (int)replyArgCount);
    digitalWrite(Serial_Send, LOW);
  }
 
  void doSerial() // UART processing function
  {
    
    if (Serial.available() >= 4 && !doingSerial)
    {

      doingSerial = true;
      //detachInterrupt(Left_INT);
      //detachInterrupt(Right_INT);
 
      digitalWrite(Status_Red, HIGH);
 
      cardType = Serial.read();
      cardIndex = Serial.read();
      cardCommand = Serial.read();
      cardArgCount = Serial.read();
      Serial.println(cardCommand);
      Serial.println(cardArgCount);
 
 
      if (true)
      {
        while (Serial.available() < cardArgCount) {};
        
        for (i = 0; i < cardArgCount; i++)
        {
          cardArg[i] = Serial.read();
        }
 
        switch (cardCommand)
        {
          case 129: // set new destinations in CLICKS :: SetNewTarget() [4 args, 2 vars]
            {
              newLeftTarget = 256 * (long)cardArg[1] + (long)cardArg[0];
              newRightTarget = 256 * (long)cardArg[3] + (long)cardArg[2];
 
              leftTarget = newLeftTarget - 32768;
              rightTarget = newRightTarget - 32768;
              break;
            }
 
          case 50: // reset destination (destination = current position) :: ResetTarget() [0 args, 0 vars]
            {
              leftTarget = leftClicks;
              rightTarget = rightClicks;
              break;
            }
 
          case 131: // set the maximum speed in clicks/second :: SetMaxSpeedC() [2 args, 1 var]
            {
              maxSpeedC = 256 * (int)cardArg[1] + (int)cardArg[0];
              break;
            }
 
          case 132: // set the maximum acceleration in clicks/second^2 :: SetMaxAccC() [2 args, 1 var]
            {
              maxAccC = 256 * (int)cardArg[1] + (int)cardArg[0];
              break;
            }
 
          case 133: // set the track width (the distance between the left and right wheels) in milimeters :: SetTrackWidth() [2 args, 1 var]
            {
              trackWidth = 256 * (int)cardArg[1] + (int)cardArg[0];
              break;
            }
 
          case 134: // set the number of clicks per revolution :: SetCPR() [2 args, 1 var]
            {
              cpr = 256 * (int)cardArg[1] + (int)cardArg[0];
              break;
            }
 
          case 135: // set the wheel diameter in millimeters :: SetWheelDiameter() [3 args, 2 var]
            {
              if (cardArg[0] == 0) leftWheelDiameter = 256 * (int)cardArg[2] + (int)cardArg[1];
              else rightWheelDiameter = 256 * (int)cardArg[2] + (int)cardArg[1];
              break;
            }
 
          case 1: // go straight forward/backward for a distance expressed in millimeters :: GoStraigth() [4 args, 2 var] -- old [2 args, 1 var]
            {
              /* OLD FUNCTION - 2 args, 1 var
                deltaForward = 256 * (long)cardArg[1] + (long)cardArg[0];
                deltaForward = deltaForward - 32768;
 
                tempDelta = (float) deltaForward;
                tempDelta = tempDelta * (float) cpr / ((float) M_PI * (float) wheelDiameter);
                deltaForward = (long) tempDelta;
 
                leftTarget = leftClicks + deltaForward;
                rightTarget = rightClicks + deltaForward;
                break;
              */
              Serial.println("fonction_avancer");
              int distance;
              distance = 256 * (long)cardArg[1] + (long)cardArg[0];
              distance = distance - 32768;
              int Speed;
              Speed = 256 * (long)cardArg[3] + (long)cardArg[2];
              Speed = Speed - 32768;
              GoStraight(distance, Speed);
              break;
            }
 
          case 137: // turn around an angle expressed in radians :: TurnRad() [4 args, 2 var] -- old [2 args, 1 var]
            {
              /* OLD FUNCTION - 2 args, 1 var
                intDeltaAngle = 256 * (long)cardArg[1] + (long)cardArg[0];
                intDeltaAngle = intDeltaAngle - 32768;
 
                deltaAngle = (float) intDeltaAngle / (float) 1000;
 
                deltaAngle = deltaAngle * (float) trackWidth * (float) cpr;
                deltaAngle = deltaAngle / ((float) 2 * m_pi * (float) wheelDiameter);
 
                deltaForward = (long) deltaAngle;
 
                leftTarget = leftClicks - deltaForward;
                rightTarget = rightClicks + deltaForward;
              */
 
              double angle = 256 * (long)cardArg[1] + (long)cardArg[0];
              angle = angle - 32768;
              angle = angle/1000;
              int Speed = 256 * (long)cardArg[3] + (long)cardArg[2];
              Speed = Speed - 32768;
              TurnRad(angle,Speed);
              break;
            }
 
          case 138: // set the maximum PWM parameter (0 - 255) :: SetMaxPWM() [1 arg, 1 var]
            {
              maxLeftPWM = (int) cardArg[0];
              maxRightPWM = (int) cardArg[0];
              break;
            }
 
          case 139: // compute and send back the current angle of the robot :: GetAngleRad() [RETURN: 2 args, 1 var]
            {
              /*deltaClicks = rightClicks - leftClicks;
                tempAngle = (float) deltaClicks * (float) m_pi * (float) wheelDiameter;
                tempAngle = tempAngle / (float) cpr;
                tempAngle = tempAngle / (float) trackWidth;*/
 
              tempAngleLeft = (float) leftClicks * (float) m_pi * (float) wheelDiameter;
              tempAngleLeft = tempAngleLeft / (float) cpr;
              tempAngleLeft = tempAngleLeft / (float) trackWidth;
 
              tempAngleRight = (float) rightClicks * (float) m_pi * (float) wheelDiameter;
              tempAngleRight = tempAngleRight / (float) cpr;
              tempAngleRight = tempAngleRight / (float) trackWidth;
 
              tempAngle = tempAngleRight - tempAngleLeft;
 
              while (tempAngle < -m_pi) tempAngle = tempAngle + 2 * m_pi;
              while (tempAngle > m_pi) tempAngle = tempAngle - 2 * m_pi;
 
              tempAngle = tempAngle * 1000.0;
              intAngle = (long) tempAngle;
              intAngle = intAngle + 32768;
 
              replyCommand = 139;
              replyArgCount = 2;
              replyArg[1] = (unsigned char) (intAngle / 256);
              replyArg[0] = (unsigned char) (intAngle - 256 * (long) replyArg[1]);
 
              sendData();
              break;
            }
 
          case 140: // compute and send back the coordinates of the robot (x,y,angle) :: GetCurrentCoord() [RETURN: 6 args, 3 var]
            {
              /*deltaClicks = rightClicks - leftClicks;
                tempAngle = (float) deltaClicks * (float) m_pi * (float) wheelDiameter;
                tempAngle = tempAngle / (float) cpr;
                tempAngle = tempAngle / (float) trackWidth;*/
 
              tempAngleLeft = (float) leftClicks * (float) m_pi * (float) wheelDiameter;
              tempAngleLeft = tempAngleLeft / (float) cpr;
              tempAngleLeft = tempAngleLeft / (float) trackWidth;
 
              tempAngleRight = (float) rightClicks * (float) m_pi * (float) wheelDiameter;
              tempAngleRight = tempAngleRight / (float) cpr;
              tempAngleRight = tempAngleRight / (float) trackWidth;
 
              tempAngle = tempAngleRight - tempAngleLeft;
 
              while (tempAngle < -m_pi) tempAngle = tempAngle + 2 * m_pi;
              while (tempAngle > m_pi) tempAngle = tempAngle - 2 * m_pi;
 
              tempAngle = tempAngle * 1000.0;
              intAngle = (long) tempAngle;
              intAngle = intAngle + 32768;
 
              x = xClicks * (float) m_pi * (float) leftWheelDiameter / (float) cpr;
              y = yClicks * (float) m_pi * (float) rightWheelDiameter / (float) cpr;
 
              xInt = (long) x + 32768;
              yInt = (long) y + 32768;
 
              replyCommand = 140;
              replyArgCount = 6;
              replyArg[1] = (unsigned char) (intAngle / 256);
              replyArg[0] = (unsigned char) (intAngle - 256 * (long) replyArg[1]);
              replyArg[3] = (unsigned char) (xInt / 256);
              replyArg[2] = (unsigned char) (xInt - 256 * (long) replyArg[3]);
              replyArg[5] = (unsigned char) (yInt / 256);
              replyArg[4] = (unsigned char) (yInt - 256 * (long) replyArg[5]);
 
              sendData();
              break;
            }
 
          case 141: // set the target speed :: SetSpeed() [4 args, 2 var]
            {
              int leftRPM = 256 * (long) cardArg[1] + (long) cardArg[0];
              int rightRPM = 256 * (long) cardArg[3] + (long) cardArg[2];
              leftRPM = leftRPM - 32768;
              rightRPM = rightRPM - 32768;
              SetTargetSpeed(leftRPM, rightRPM);
              
              break;
            }
 
          case 142: // set working mode :: SetWorkingMode() [1 arg, 1 var]
            {
              PIDmode = (int) (cardArg[0] - 128);
              break;
            }
 
          case 143: // find out if the robot has arrived to its destination :: HasArrived() [RETURN: 1 arg, 1 var]
            {
              replyCommand = 143;
              replyArgCount = 1;
 
              if (hasArrived == true) replyArg[0] = 129;
              else replyArg[0] = 128;
 
              sendData();
 
              break;
            }
        }
      }
 
      //attachInterrupt(Left_INT, leftUpdate, CHANGE);
      //attachInterrupt(Right_INT, rightUpdate, CHANGE);
      digitalWrite(Status_Red, LOW);
      doingSerial = false;
    }
  }
 
  // ##### The program's main loop. #####
  void loop()
  {
    //la fonction asservit() était écrite explicitement mais je les ai supprimer
    //en remplaçant par un appel à la fonction
    
    //tester asservit()
    asservit();
    //tester asservit_polaire()
    //asservit_polaire()
    //tester asservit_distance();
    //asservit_distance();
    doSerial();
 
    // and, of course, blinking
    if (statusBlue == false) {
      statusBlue = true;
      digitalWrite(Status_Blue, HIGH);
    }
    else {
      statusBlue = false;
      digitalWrite(Status_Blue, LOW);
    }
  }
