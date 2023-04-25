// libraries
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PCA9685_ADDR 0x40 //**
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
#include "math.h"
#include <Pixy2SPI_SS.h>

// objects
Pixy2SPI_SS pixy;

//**CONSTANTS**
//-------------
float abs_0 = 4000;                                     //ms position of absolute 0 degrees
float abs_90 = 8000;                                    //ms position of absolute 90 degrees
float toDeg = 180 / PI;                                 // radians to degrees conversion factor
int x = 0, y = 1, z = 2;                                // defines x, y, and z array indexes to be used
String ID[6] = { "a1", "a2", "b1", "b2", "c1", "c2" };  //servo ID's

//**USER DEFINED VALUES**
// each servo movement from absolute 0 to absolute 90 degrees this digines different range.
float range[6][2] = { { -45, 45 }, { 45, -45 },  // a1, a2
                      { -45, 45 },
                      { 45, -45 },  // b1, b2
                      { -45, 45 },
                      { 45, -45 } };  // c1, c2

// angle offset value for each servo that redefines the position of the lower range of each servo

float offset[6] = { 4, 0,    // channel #0 and channel #1
                  -7, 3,    // channel #2 and channel #3
                    1, 0 };  // channel #4 and channel #5

//**INVERSE KINEMATICS**
//----------------------

// user defined lengths (in mm)
float l0 = 73.025;
float lf = 67.775;
float d1 = 36.8893;
float d2 = 38.1;
float m = 12.7;
float p1 = 31.75;
float p2 = 129;

//calculates normal vectors nab, nac, and nbc for each side of the triangle
float nab[3] = { sqrt(3) * 0.5, -0.5, 0 };
float nac[3] = { sqrt(3) * 0.5, 0.5, 0 };
float nbc[3] = { 0, 1, 0 };

// intermediate variables
float t = (pow(lf, 2) * sqrt(3)) / 2;
float u = sqrt(pow(l0, 2) + pow(d1, 2)) * sin((2 * PI / 3) - atan(l0 / d1));  //(2*pi/3 is 120 degrees)

// calculates points a10, a20, b10, b20, c10, and c20
float a10[3] = { (d2 - u * sqrt(3)) / 2, (-u - d2 * sqrt(3)) / 2, 0 };
float a20[3] = { -a10[x], a10[y], 0 };
float b10[3] = { (u * sqrt(3) + d2) / 2, (d2 * sqrt(3) - u) / 2, 0 };
float b20[3] = { d2, u, 0 };
float c10[3] = { -b20[x], b20[y], 0 };
float c20[3] = { -b10[x], b10[y], 0 };

// calculates vectors, ab, ac, and bc
float ab[3] = { a20[x] - b10[x], a20[y] - b10[y], a20[z] - b10[z] };
float ac[3] = { a10[x] - c20[x], a10[y] - c20[y], a10[z] - c20[z] };
float bc[3] = { b20[x] - c10[x], b20[y] - c10[y], b20[z] - c10[z] };

float theta[6];                      //array for calculated theta values a1, a2, b1, b2, c1, c2
float nz = 1;                        //nz is defined earlier
float hz_nor = 118.19374266158451;  //normal hz value (this is the height when all servos are zero degree)
float R_MAX = 0.25;                  // max radius of the nx and ny graph when nz = 1

//PIXY 2 CAM
//----------------------
float origin[2] = { 0, 0 };           // X and Y co-ords of the origin
float R_Platform = 0;                 // the distance from the center of the platform to the corner of the platform seen from the PIXY 2 CAM
float ball[2];                        // X and Y co-ordinates of the ball

// PID ALGORITHM
//----------------------
float error[2];       // Ball Error
float error_prev[2];  // previous error value for calculating derivative. Derivative = (error-previous_error)/(change in time)
float deriv[2];       // derivative 
float deriv_prev[2];  // previous derivative value
float kp = 6e-4;      // proportional constant
float kd = 0.56;      // derivative constant
float out[2];         // output values (nx and ny)
float TIME_i;         // initial time
float TIME_f;         // final time
float time;           //change in time

void setup() {
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);  //set PWM frequency to 50Hz
  pixy.init();
}

void loop() {
  findBall();                   // To find ball location
  if (ball[x] == 4004 && ball[y] == 4004) { // if ball position is 4004 then the ball is not detected and platform goes to home home position
    InverseKinematics(0, 0, hz_nor, 0, 0, 0);  //hx, hy, hz, nx, ny, ax
    moveServos(20, 20);
  } else {
    PD();  // calculation of proportional and derivative terms. Give to the platform
  }
}

//Functions
void moveServo(int i, float pos, int spd, int acc) {        //moves servo i to an input position at a certain speed and acceleration value
  pos = pos + offset[i];                                    //adds offset amount to the input position
  pos = map(pos, range[i][0], range[i][1], abs_0, abs_90);  //converts input pos to ms position
  pwm.setPWM(i, 0, pos);                                    //drives motor to calculated position
}

void moveServos(int spd, int acc) {                            //moves servos to their calculated positions at a certain speed and acceleration value
  float pos;
  for (int i = 0; i < 6; i++) {
    pos = theta[i] + offset[i];                               //adds offset amount to the calculated angle
    pos = map(pos, range[i][0], range[i][1], abs_0, abs_90);  //converts input pos to ms position
    pwm.setPWM(i, 0, pos);                                    //drives motor to calculated position
  }
}

void stop() {                                                  //Stop the program and all servos
  for (int i = 0; i < 6; i++) {
    pwm.setPWM(i, 0, 0);                                       //stops servo i
  }
  while (1) {}
}

void findBall() {                                              // find the location of the ball using the pixy2 cam
  pixy.ccc.getBlocks();

                                                             // If there is 1 ball detected then collect the data
  if (pixy.ccc.numBlocks == 1) {
    //sets current X and Y co-ords
    ball[x] = pixy.ccc.blocks[0].m_x;                          // absolute X location of the ball
    ball[y] = pixy.ccc.blocks[0].m_y;                          // absolute Y location of the ball
  }
  else if (pixy.ccc.numBlocks > 1) {
    ball[x] = 4004;  // X component of the ball
    ball[y] = 4004;  // Y component of the ball
  }
  // If there is no ball detected, then print so
  else {
    ball[x] = 4004;  // X component of the ball
    ball[y] = 4004;  // Y component of the ball
  }
}

void PD() {  // calculates the P snd D values and move all the servos
  // calculates the error of the ball
  // the error is the difference of the location of the center of the ball and the center of the platform
  // it is essentially a vector pointing from the center of the platform to the center of the ball
  error[x] = origin[x] - ball[x];  // x component of error
  error[y] = ball[y] - origin[y];  // y component of error

  TIME_f = millis();       // final time
  time = TIME_f - TIME_i;  // change in time
  TIME_i = millis();
  // sets initial time
  deriv[x] = (error[x] - error_prev[x]) / time;  // x component of derivative
  deriv[y] = (error[y] - error_prev[y]) / time;  // y component of derivative

  // checks if derivative is NaN or INF. If so, set to zero
  if (isnan(deriv[x]) || isinf(deriv[x])) {  // x component of derivative
    deriv[x] = 0;
  }
  if (isnan(deriv[y]) || isinf(deriv[y])) {  // y component of derivative
    deriv[y] = 0;
  }

  // sets previous error to current error
  error_prev[x] = error[x];  // x component of previous error
  error_prev[y] = error[y];  // x component of previous error


  float r_BALL = sqrt(pow(error[x], 2) + pow(error[y], 2));  // caculation of distance from center of plat to the ball

  if (r_BALL > R_Platform) {
    // checks to see if the platform should be moved to the home position
    // checks if the ball is on the platform by comparing r_BALL and R_Platform. 
    //If r_BALL is greater than R_Platform, the ball is off the platform
    // and the platform should be in the home position
    InverseKinematics(0, 0, hz_nor, 0, 0, 0);  //hx, hy, hz, nx, ny, ax
    moveServos(20, 20);
  }

  else {  //if the ball should not be in the home position then calculate PD outputs
  
    out[x] = (error[x] * kp) + (deriv[x] * kd);  
    out[y] = (error[y] * kp) + (deriv[y] * kd);  

    // error prevention
    float r_OUT = sqrt(pow(out[x], 2) + pow(out[y], 2));  // Magnitude of the out vector calculation
    if (r_OUT > R_MAX) {                                  
      out[x] = out[x] * (R_MAX / r_OUT);
      out[y] = out[y] * (R_MAX / r_OUT);
    }

    //move platform
    InverseKinematics(0, 0, hz_nor, out[x], out[y], 0);  //hx, hy, hz, nx, ny, ax
    moveServos(0, 0);
  }
}

void InverseKinematics(float hx, float hy, float hz, float nx, float ny, float ax) {  // calculates theta values 
  //define vectors and points
  float a[3] = { ax, 0, 0 }, a1f[3], a2f[3];
  float b[3], b1f[3], b2f[3];
  float c[3], c1f[3], c2f[3];

  float n[3] = { nx, ny, nz };  // defines normal vector
  n[x] = nx / mag(n);
  n[y] = ny / mag(n);
  n[z] = nz / mag(n);  // converts vector 'n' to a unit vector

  float h[3] = { hx, hy, hz };  // defined point h which is center of the platform

  // ** CALCULATIONS**
  float e[3], g[3], k[3];

  // af components
  e[0] = a[x] - h[x];                                                                                                       
  a[z] = ((n[y] * sqrt(pow(lf, 2) * (1 - pow(n[x], 2)) - pow(e[0], 2)) - n[z] * n[x] * e[0]) / (1 - pow(n[x], 2))) + h[z];  
  g[0] = a[z] - h[z];                                                                                                       
  a[y] = h[y] - sqrt(pow(lf, 2) - pow(g[0], 2) - pow(e[0], 2));                                                             
  k[0] = a[y] - h[y];                                                                                                       

  float w = sqrt(3) * (n[x] * g[0] - n[z] * e[0]);  // intermediate variable

  // bf components
  b[y] = h[y] + ((sqrt(pow(w, 2) - 3 * pow(lf, 2) * (1 - pow(n[y], 2)) + pow(2 * k[0], 2)) - w) / 2);  
  k[1] = b[y] - h[y];                                                                                  
  b[x] = ((e[0] * k[1] - n[z] * t) / k[0]) + h[x];                                                     
  e[1] = b[x] - h[x];                                                                                  
  b[z] = ((n[x] * t + g[0] * k[1]) / k[0]) + h[z];                                                     
  g[1] = b[z] - h[z];                                                                                  

  // cf components
  c[y] = h[y] + ((w + sqrt(pow(w, 2) - 3 * pow(lf, 2) * (1 - pow(n[y], 2)) + pow(2 * k[0], 2))) / 2);  
  k[2] = c[y] - h[y];                                                                                  
  c[x] = ((e[0] * k[2] + n[z] * t) / k[0]) + h[x];                                                     
  e[2] = c[x] - h[x];                                                                                  
  c[z] = ((g[0] * k[2] - n[x] * t) / k[0]) + h[z];                                                     
  g[2] = c[z] - h[z];                                                                                  


  // a1
  a1f[x] = a[x] + (m / lf) * (n[z] * k[0] - n[y] * g[0]);  
  if (e[0] == 0) {                                         
    a1f[y] = a[y];                                         
    a1f[z] = a[z];                                         
  } else {
    a1f[y] = a[y] + ((a1f[x] - a[x]) * k[0] - n[z] * lf * m) / e[0];  
    a1f[z] = a[z] + (n[y] * lf * m + (a1f[x] - a[x]) * g[0]) / e[0];  
  }
  float a1[3] = { a1f[x] - a10[x], a1f[y] - a10[y], a1f[z] - a10[z] };  

  // a2
  a2f[x] = 2 * a[x] - a1f[x];                                           
  a2f[y] = 2 * a[y] - a1f[y];                                           
  a2f[z] = 2 * a[z] - a1f[z];                                           
  float a2[3] = { a2f[x] - a20[x], a2f[y] - a20[y], a2f[z] - a20[z] };  

  // b1
  b1f[x] = b[x] + (m / lf) * (n[z] * k[1] - n[y] * g[1]);               
  b1f[y] = b[y] + ((b1f[x] - b[x]) * k[1] - n[z] * lf * m) / e[1];      
  b1f[z] = b[z] + (n[y] * lf * m + (b1f[x] - b[x]) * g[1]) / e[1];      
  float b1[3] = { b1f[x] - b10[x], b1f[y] - b10[y], b1f[z] - b10[z] };  

  // b2
  b2f[x] = 2 * b[x] - b1f[x];                                           
  b2f[y] = 2 * b[y] - b1f[y];                                           
  b2f[z] = 2 * b[z] - b1f[z];                                           
  float b2[3] = { b2f[x] - b20[x], b2f[y] - b20[y], b2f[z] - b20[z] };  

  // c1
  c1f[x] = c[x] + (m / lf) * (n[z] * k[2] - n[y] * g[2]);               
  c1f[y] = c[y] + ((c1f[x] - c[x]) * k[2] - n[z] * lf * m) / e[2];      
  c1f[z] = c[z] + (n[y] * lf * m + (c1f[x] - c[x]) * g[2]) / e[2];      
  float c1[3] = { c1f[x] - c10[x], c1f[y] - c10[y], c1f[z] - c10[z] };  
  // c2
  c2f[x] = 2 * c[x] - c1f[x];                                           
  c2f[y] = 2 * c[y] - c1f[y];                                           
  c2f[z] = 2 * c[z] - c1f[z];                                          
  float c2[3] = { c2f[x] - c20[x], c2f[y] - c20[y], c2f[z] - c20[z] };  

  //**STAGE 3 CALCULATIONS**

  // theta_a1
  float a1s[3] = { nac[x] * dot(a1, nac), nac[y] * dot(a1, nac), nac[z] * dot(a1, nac) };                                
  float Magn_a1s = mag(a1s);                                                                                              
  float a1_proj[3] = { a1[x] - a1s[x], a1[y] - a1s[y], a1[z] - a1s[z] };                                                 
  float Magn_a1_Projec = mag(a1_proj);                                                                                      
  float Magn_p2a1 = sqrt(pow(p2, 2) - pow(Magn_a1s, 2));                                                                   
  theta[0] = acos(-dot(a1_proj, ac) / (2 * d2 * Magn_a1_Projec));                                                           
  theta[0] = (theta[0] - acos((pow(Magn_a1_Projec, 2) + pow(p1, 2) - pow(Magn_p2a1, 2)) / (2 * Magn_a1_Projec * p1))) * toDeg;  

  // theta_a2
  float a2s[3] = { nab[x] * dot(a2, nab), nab[y] * dot(a2, nab), nab[z] * dot(a2, nab) };                                
  float Magn_a2s = mag(a2s);                                                                                              
  float a2_proj[3] = { a2[x] - a2s[x], a2[y] - a2s[y], a2[z] - a2s[z] };                                                 
  float Magn_a2_Projec = mag(a2_proj);                                                                                      
  float Magn_p2a2 = sqrt(pow(p2, 2) - pow(Magn_a2s, 2));                                                                   
  theta[1] = acos(-dot(a2_proj, ab) / (2 * d2 * Magn_a2_Projec));                                                           
  theta[1] = (theta[1] - acos((pow(Magn_a2_Projec, 2) + pow(p1, 2) - pow(Magn_p2a2, 2)) / (2 * Magn_a2_Projec * p1))) * toDeg;  

  // theta_b1
  float b1s[3] = { nab[x] * dot(b1, nab), nab[y] * dot(b1, nab), nab[z] * dot(b1, nab) };                                
  float Magn_b1s = mag(b1s);                                                                                              
  float b1_proj[3] = { b1[x] - b1s[x], b1[y] - b1s[y], b1[z] - b1s[z] };                                                 
  float Magn_b1_Projec = mag(b1_proj);                                                                                      
  float Magn_p2b1 = sqrt(pow(p2, 2) - pow(Magn_b1s, 2));                                                                   
  theta[2] = acos(dot(b1_proj, ab) / (2 * d2 * Magn_b1_Projec));                                                            
  theta[2] = (theta[2] - acos((pow(Magn_b1_Projec, 2) + pow(p1, 2) - pow(Magn_p2b1, 2)) / (2 * Magn_b1_Projec * p1))) * toDeg;  

  // theta_b2
  float b2s[3] = { nbc[x] * dot(b2, nbc), nbc[y] * dot(b2, nbc), nbc[z] * dot(b2, nbc) };                                
  float Magn_b2s = mag(b2s);                                                                                              
  float b2_proj[3] = { b2[x] - b2s[x], b2[y] - b2s[y], b2[z] - b2s[z] };                                                 
  float Magn_b2_Projec = mag(b2_proj);                                                                                      
  float Magn_p2b2 = sqrt(pow(p2, 2) - pow(Magn_b2s, 2));                                                                   
  theta[3] = acos(-dot(b2_proj, bc) / (2 * d2 * Magn_b2_Projec));                                                           
  theta[3] = (theta[3] - acos((pow(Magn_b2_Projec, 2) + pow(p1, 2) - pow(Magn_p2b2, 2)) / (2 * Magn_b2_Projec * p1))) * toDeg;  

  // theta_c1
  float c1s[3] = { nbc[x] * dot(c1, nbc), nbc[y] * dot(c1, nbc), nbc[z] * dot(c1, nbc) };                                
  float Magn_c1s = mag(c1s);                                                                                              
  float c1_proj[3] = { c1[x] - c1s[x], c1[y] - c1s[y], c1[z] - c1s[z] };                                                 
  float Magn_c1_Projec = mag(c1_proj);                                                                                      
  float Magn_p2c1 = sqrt(pow(p2, 2) - pow(Magn_c1s, 2));                                                                   
  theta[4] = acos(dot(c1_proj, bc) / (2 * d2 * Magn_c1_Projec));                                                            
  theta[4] = (theta[4] - acos((pow(Magn_c1_Projec, 2) + pow(p1, 2) - pow(Magn_p2c1, 2)) / (2 * Magn_c1_Projec * p1))) * toDeg;  

  //theta_c2
  float c2s[3] = { nac[x] * dot(c2, nac), nac[y] * dot(c2, nac), nac[z] * dot(c2, nac) };                                
  float Mag_c2s = mag(c2s);                                                                                              
  float c2_proj[3] = { c2[x] - c2s[x], c2[y] - c2s[y], c2[z] - c2s[z] };                                                 
  float Magn_c2_Projec = mag(c2_proj);                                                                                      
  float Magn_p2c2 = sqrt(pow(p2, 2) - pow(mag_c2s, 2));                                                                   
  theta[5] = acos(dot(c2_proj, ac) / (2 * d2 * Magn_c2_Projec));                                                            
  theta[5] = (theta[5] - acos((pow(Magn_c2_Projec, 2) + pow(p1, 2) - pow(Magn_p2c2, 2)) / (2 * Magn_c2_Projec * p1))) * toDeg;  

  for (int i = 0; i < 6; i++) {  //checks for errors

    if (abs(theta[i]) > 40) {
      stop();
    }
    if (isnan(theta[i])) {
      stop();
    }
  }
}

float MAG(float array[]) {  //finds the magnitude of an array of size 3
  float MAG = 0;
  for (int i = 0; i < 3; i++) {
    MAG = MAG + pow(array[i], 2);  //adds component i of array squared
  }
  MAG = sqrt(MAG);
  return MAG;
}

float dot(float array1[], float array2[]) {  //calculates the dot product of two arrays
  return array1[0] * array2[0] + array1[1] * array2[1] + array1[2] * array2[2];
}