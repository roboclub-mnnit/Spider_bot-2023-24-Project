#include <Adafruit_PWMServoDriver.h>                             //Include the PWM Driver library

Adafruit_PWMServoDriver Spider = Adafruit_PWMServoDriver(0x40);    //Create an object of board 1


double NormalMotorPosition[16] = {90, 178, 100, 88,   // front right leg {-, ankle, knee, hip}
                                  90, 5,   88,  90,   // front left leg {-, ankle, knee, hip}
                                  90, 0,   88,  88,   // rear right leg {-, ankle, knee, hip}
                                  90, 178, 88,  92};  // rear left leg {-, ankle, knee, hip}

double CurrentMotorPosition[16] = {90, 178, 100, 88,   // front right leg {-, ankle, knee, hip}
                                   90, 5,   88,  90,   // front left leg {-, ankle, knee, hip}
                                   90, 0,   88,  88,   // rear right leg {-, ankle, knee, hip}
                                   90, 178, 88,  92};  // rear left leg {-, ankle, knee, hip}

double FinalMotorPosition[16] = {90, 178, 100, 88,   // front right leg {-, ankle, knee, hip}
                                 90, 5,   88,  90,   // front left leg {-, ankle, knee, hip}
                                 90, 0,   88,  88,   // rear right leg {-, ankle, knee, hip}
                                 90, 178, 88,  92};  // rear left leg {-, ankle, knee, hip}
double Position[12] = {16.5, 0, 0,
                       16.5, 0, 0,
                       16.5, 0, 0,
                       16.5, 0, 0};
double P[12] = {0, 0, 0,
                0, 0, 0,
                0, 0, 0,
                0, 0, 0};

double a1 = 3, a2 = 5.5, a3 = 8;
double v=8.5, w=9.5, d=6.374, beta=atan(v/w);
double xc=0, yc=0, zc=0;
double a=2, b=3;

void setup()
{
  Serial.begin(9600);
  Spider.begin();      
  Spider.setOscillatorFrequency(27000000);    //Set the PWM oscillator frequency, used for fine calibration
  Spider.setPWMFreq(50);          //Set the servo operating frequency 50Hz
  delay(1000);
  setMotors(FinalMotorPosition);
  delay(2000);
  standing();
  delay(500);
  //shiftLeft();
  
  //Trajectory(0);
  //centerLeftRight(-3);
  //Trajectory(0);
  
  
  
  //moveUpDown(2);
  //delay(500);
  //turnRight();
  //centerShift(3.5,0);
  //standing2();
  
  //centerLeftRight(3);
  //centerLeftRight(-3);
  //moveBot();
  
  //turnRight();
  
  //delay(500);
  //centerShift(2,0);
  //Serial.println(Position[3*0]);
  //Serial.println(Position[3*0+1]);
  
  //turn(1,20);
  //rightTurn();
  //moveBot();
  //centerShift(4,0);
  //check(2,1);
  //down(3);
  //standing2();
  //centerLeftRight(4);
  //standing2();
  //delay(1000)
  //moveUpDown(2);
  //standing2();
  
  //legDown(4,2);
  //centerShift(-3,2);
  //centerLeftRight(-2);
  //centerFrontBack(1);
  //Trajectory(0);
  //standing();
  //centerLeftRight(-3);
  //moveLeg(0);
}

void loop()
{
 
  String str="";
  if(Serial.available()){
    str=Serial.readStringUntil('\n');

    String data1 = str.substring(0, str.indexOf('/'));
    int x = data1.toInt();
    str.remove(0, str.indexOf('/')+1);
    String data2 = str.substring(0, str.indexOf('/'));
    float y = data2.toFloat();
  
    Serial.print(x);
    Serial.print(" ");
    Serial.println(y);
    
    FinalMotorPosition[x] = y;
    setMotors(FinalMotorPosition);
  }

  moveBot();

  /*
  down(3);
  standing2();
  delay(1000); 
  centerLeftRight(4);
  standing2();
  delay(1000);
  */
  
  /*
  delay(1000);
  moveUpDown(2);
  delay(1000);
  centerLeftRight(4);
  delay(1000);
  centerLeftRight(-2);
  //centerFrontBack(1);
  delay(1000);
  //standing();
  */
}


void setMotors(double FinalMotorPosition[]){
  for(int t=0; t<=10; t++){
    
    for(int i=0; i<16; i++){
      int angle = CurrentMotorPosition[i] + t*(FinalMotorPosition[i] - CurrentMotorPosition[i])/100;
      int PWM_value = map(angle, 0, 180, 120, 460); // pulse lenght 120 for 0 degree and 460 for 180 degree
      Spider.setPWM(i, 0, PWM_value);
    }
    
  }
  for(int i=0; i<16; i++){
    CurrentMotorPosition[i] = FinalMotorPosition[i];
  }
}

void setAngle(double q, int n){
  double Q = q*(180/3.14);
  if(n==1||n==2||n==7||n==11||n==13||n==14)
    Q=NormalMotorPosition[n]-Q;
  else
    Q=NormalMotorPosition[n]+Q;
  if(Q>180)
    Q=Q-180;
  FinalMotorPosition[n]=Q;
}

void setAngle2(double Q, int n){
  if(n==1||n==2||n==7||n==11||n==13||n==14)
    Q=FinalMotorPosition[n]-Q;
  else
    Q=FinalMotorPosition[n]+Q;
  if(Q>180)
    Q=Q-180;
  FinalMotorPosition[n]=Q;
}

void invKin(double px, double py, double pz, int l){
  double R1=sqrt(px*px+py*py)-a1;
  double R2=sqrt(R1*R1+pz*pz);
  double q1=atan(py/px);
  double q2=atan(pz/R1)+acos((a2*a2+R2*R2-a3*a3)/(2*a2*R2));
  double q3=acos((R2*R2-a2*a2-a3*a3)/(2*a2*a3));
  setAngle(q3,4*l+1);
  setAngle(q2,4*l+2);
  setAngle(q1,4*l+3);
}

void standing(){
  double P[12] = {5.49, 5.49, -5.25,
                  5.49, 5.49, -5.25,
                  5.49, 5.49, -5.25,
                  5.49, 5.49, -5.25};
  for(int l=0;l<4;l++)
    invKin(P[3*l],P[3*l+1],P[3*l+2],l);
  setMotors(FinalMotorPosition);
  for(int i=0; i<12; i++)
    Position[i] = P[i];
}

void standing2(){
  double P[12] = {5.49, 5.49, -7.25,
                  5.49, 5.49, -7.25,
                  5.49, 5.49, -7.25,
                  5.49, 5.49, -7.25};
  for(int l=0;l<4;l++)
    invKin(P[3*l],P[3*l+1],P[3*l+2],l);
  setMotors(FinalMotorPosition);
  for(int i=0; i<12; i++)
    Position[i] = P[i];
}

void moveUpDown(double k){
  double t=0;
  while(t<=1){
    zc=k*t;
    for(int l=0;l<4;l++){
      invKin(Position[3*l],Position[3*l+1],Position[3*l+2]-zc,l); 
      P[3*l+2]=Position[3*l+2]-zc;
  } 
    setMotors(FinalMotorPosition);
    t=t+0.5;
  }
  for(int l=0;l<4;l++)
    Position[3*l+2] = P[3*l+2];
  zc=0;
  
  /*double MotorPosition[16] = {90, 99.11, 113.37, 133.02,   // front right leg {-, ankle, knee, hip}
                              90, 85.89,   74.63,  44.98,   // front left leg {-, ankle, knee, hip}
                              90, 80.89,   74.63,  42.98,   // rear right leg {-, ankle, knee, hip}
                              90, 97.11, 101.37,  137.02};
  for(int i=0; i<16; i++)
    FinalMotorPosition[i] = MotorPosition[i];
  setMotors(FinalMotorPosition);*/
  
  /*
  for(int l=0;l<4;l++){
      invKin(Position[3*l],Position[3*l+1],Position[3*l+2]-4,l);
      setMotors(FinalMotorPosition);
  }
  */
} 

void legDown(double zp, int l){
  Position[3*l+2] = Position[3*l+2]-zp;
  invKin(Position[3*l],Position[3*l+1],Position[3*l+2],l); 
  Position[9-3*l+2]=Position[9-3*l+2]+zp;
  invKin(Position[9-3*l],Position[9-3*l+1],Position[9-3*l+2],l);
  
  //setAngle2(30,4*l+2);
  //setAngle2()
}

void down(int l){
  double MotorPosition[16] = {90, 75.48, 92.47, 133.02,   
                              90, 107.52, 88, 35,   
                              90, 102.52, 88, 53,   
                              90, 75.48, 50,  137.02};
  for(int i=0; i<16; i++)
    FinalMotorPosition[i] = MotorPosition[i];
  setMotors(FinalMotorPosition);                        
}



void centerShift(double zp,int l){
  double alpha=asin(zp/d),px,py,pz;
  /*
  for(int i=0;i<4;i++){
    if(i==l){
      Position[3*i]=Position[3*i]+2*d*cos(beta)*(cos(alpha-1));
      Position[3*i+1]=Position[3*i+1]+2*d*sin(beta)*(cos(alpha-1));
      Position[3*i+2]=Position[3*i+2]+zp;
    }
    else if(i==(3-l)){
      Position[3*i+2]=Position[3*i+2]-zp;
    }
    else{
      Position[3*i]=Position[3*i]+d*cos(beta)*(cos(alpha-1));
      Position[3*i+1]=Position[3*i+1]+d*sin(beta)*(cos(alpha-1));
    }
    invKin(Position[3*i],Position[3*i+1],Position[3*i+2],i); 
  }
  */
  
  
  Position[3*l]=Position[3*l]+d*cos(beta)*(1-cos(alpha));
  Position[3*l+1]=Position[3*l+1]+d*sin(beta)*(1-cos(alpha));
  Position[3*l+2]=Position[3*l+2]-zp;
  invKin(Position[3*l],Position[3*l+1],Position[3*l+2],l);
  
  Position[9-3*l]=Position[9-3*l]+d*cos(beta)*(1-cos(alpha));
  Position[9-3*l+1]=Position[9-3*l+1]+d*sin(beta)*(1-cos(alpha));
  Position[9-3*l+2]=Position[9-3*l+2]+zp;
  invKin(Position[9-3*l],Position[9-3*l+1],Position[9-3*l+2],3-l);
  
  setMotors(FinalMotorPosition);
}

void Trajectory(int l){
  double t=0,py,pz;
  while(t<=1){
    if(l==0||l==1)
      py=Position[3*l+1]+2*a*t;
    else if(l==2||l==3)
      py=Position[3*l+1]-2*a*t;
    pz=Position[3*l+2]+2*b*sqrt(t-t*t);
    invKin(Position[3*l],py,pz,l);
    setMotors(FinalMotorPosition);
    t=t+0.5;
  }
  Position[3*l+1]=py;
}

void moveLeg(int l){
  centerShift(2,l);
  Trajectory(l);
  centerShift(-2,l);
}

void moveBot(){
  
  moveLeg(0);
  moveLeg(3);
  moveLeg(1);
  moveLeg(2);
  standing2();
  
  /*
  moveLeg(0);
  moveLeg(3);
  moveLeg(1);
  moveLeg(2);
  standing2();
  */
}

void centerLeftRight(double k){
  double t=0,px=0,py=0;
  while(t<=1){
    xc=k*t;
    //yc=k*t;
    for(int l=0;l<4;l++){
      if(l==0||l==2)
        px=Position[3*l]-xc;
      else
        px=Position[3*l]+xc;
      /*
      if(l==0||l==1)
        py=Position[3*l+1]-yc;
      else
        py=Position[3*l+1]+yc;
      */
      py=Position[3*l+1];
      invKin(px,py,Position[3*l+2],l);
    }
    setMotors(FinalMotorPosition);
    t=t+0.5;
    //Serial.println(xc);
  }
}

void centerFrontBack(double k){
  double t=0,py=0;
  while(t<=1){
    yc=k*t;
    for(int l=0;l<4;l++){      
      if(l==0||l==1)
        py=Position[3*l+1]-yc;
      else
        py=Position[3*l+1]+yc;
      
      invKin(Position[3*l],py,Position[3*l+2],l);
    }
    setMotors(FinalMotorPosition);
    t=t+0.5;
    //Serial.println(xc);
  }
}

void turnRight(){
  rightTurn(-20);
  delay(10);
  rightTurn(-20);
  delay(10);
  /*rightTurn(25);
  delay(100);
  rightTurn(25);
  delay(100);*/
}

void rightTurn(double a){
  centerShift(2,0);
  turn(0,a);
  centerShift(-2,0);
  centerShift(2,3);
  turn(3,a);
  centerShift(-2,3);
  centerShift(2,1);
  turn(1,-a);
  centerShift(-2,1);
  centerShift(2,2);
  turn(2,-a);
  centerShift(-2,2);
  standing2();
}

void turn(int l,double gamma){
  gamma=(gamma)*(3.14/180);
  //upTurn(l,gamma/2);
  //gamma=gamma/2;
  double t=0,pz;
  double alpha=atan((Position[3*l+1]+(v/2))/(Position[3*l]+(w/2)));
  double r = sqrt(Position[3*l]*Position[3*l]+Position[3*l+1]*Position[3*l+1])+d;
  /*
  if(l==1||l==2){
    Position[3*l]=r*cos(alpha+gamma)+w/2;
    Position[3*l+1]=r*sin(alpha+gamma)-v/2;
  }
  */
  while(t<=1){
  Position[3*l]=r*cos(alpha+gamma*t)-w/2;
  Position[3*l+1]=r*sin(alpha+gamma*t)-v/2;
  pz=Position[3*l+2]+2*b*sqrt(t-t*t);
  invKin(Position[3*l],Position[3*l+1],pz,l);
  setMotors(FinalMotorPosition);
  t=t+0.5;
  //Serial.println(alpha);
  //Serial.println(Position[3*l]);
  //Serial.println(Position[3*l]+1);
  }
}

void upTurn(int l,double gamma){
  double alpha=atan((Position[3*l+1]+(v/2))/(Position[3*l]+(w/2)));
  double r = sqrt(Position[3*l]*Position[3*l]+Position[3*l+1]*Position[3*l+1])+d;
  Position[3*l]=r*cos(alpha+gamma)-w/2;
  Position[3*l+1]=r*sin(alpha+gamma)-v/2;
  invKin(Position[3*l],Position[3*l+1],Position[3*l+2]+3,l);
  setMotors(FinalMotorPosition);
}

void check(double zp,int l){
  double alpha=asin(zp/d),px,py,pz;
  /*
  for(int i=0;i<4;i++){
    if(i==l){
      Position[3*i]=Position[3*i]+2*d*cos(beta)*(cos(alpha-1));
      Position[3*i+1]=Position[3*i+1]+2*d*sin(beta)*(cos(alpha-1));
      Position[3*i+2]=Position[3*i+2]+zp;
    }
    else if(i==(3-l)){
      Position[3*i+2]=Position[3*i+2]-zp;
    }
    else{
      Position[3*i]=Position[3*i]+d*cos(beta)*(cos(alpha-1));
      Position[3*i+1]=Position[3*i+1]+d*sin(beta)*(cos(alpha-1));
    }
    invKin(Position[3*i],Position[3*i+1],Position[3*i+2],i); 
  }
  */
  
  
  Position[3*l]=Position[3*l]+d*cos(beta)*(1-cos(alpha));
  Position[3*l+1]=Position[3*l+1]+d*sin(beta)*(1-cos(alpha));
  Position[3*l+2]=Position[3*l+2]-zp;
  invKin(Position[3*l],Position[3*l+1],Position[3*l+2],l);
  
  
  Position[9-3*l]=Position[9-3*l]+d*cos(beta)*(1-cos(alpha));
  Position[9-3*l+1]=Position[9-3*l+1]+d*sin(beta)*(1-cos(alpha));
  Position[9-3*l+2]=Position[9-3*l+2]+zp;
  invKin(Position[9-3*l],Position[9-3*l+1],Position[9-3*l+2],3-l);
  
  setMotors(FinalMotorPosition);
}

void shiftLeft(){
  double MotorPosition[16] = {90,  60, 70, 133.02,   // front right leg {-, ankle, knee, hip}
                              90,  5, 88,  44.98,   // front left leg {-, ankle, knee, hip}
                              90,  80, 88,  42.98,   // rear right leg {-, ankle, knee, hip}
                              90,  35, 40, 137.02};
  for(int i=0; i<16; i++)
    FinalMotorPosition[i] = MotorPosition[i];
  setMotors(FinalMotorPosition);
}
