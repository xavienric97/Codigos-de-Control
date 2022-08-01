#include <Arduino.h>
#include <Wire.h> //This is for i2C
#include <AS5600.h>
#include <HardwareSerial.h>
#include <PID_v1.h>
#include <cstdlib>

using namespace std;
// // I2C pins:
// // STM32: SDA: PB7 --- SCL: PB6

// // Serial Print Pins:
// // PA3: RX2 --- PA2: TX2

// Stepper Motor: 12V
// In2: CCW Positive PB8
// In1: CW Negative PB9

//////////////////////////

// Error = 0 dentro del rango aceptable
// 90 to 270
// Actual movement: {{530, 530}, {570, 560}, {0, 0}, {530, 530}, {0,0}}

HardwareSerial Serial2(PA3, PA2); // Para el serial print con un Arduino UNO

int32_t motors[5][4] = {{530, 530, 1023, -1023}, {2125, 2125, 4095, -4095}, {8500, 8500, 16383, -16383}, {34000, 34000, 65535, -65535}, {0,0, 1023, -1023}};
int32_t motor_num = 0;
int32_t dir, dead_zone_up = motors[motor_num][0], dead_zone_down = motors[motor_num][1], max_up = motors[motor_num][2], max_down = motors[motor_num][3];
double error, tolerancia, tol_up, tol_down;

double convertRawAngleToDegrees(word);
int32_t direction(float diferencia);

void position_correction(float lectura, double val);

double Setpoint, Input, Output, Final_Time, Previous_Time, Previous_Angle;
bool done = false;
int32_t gains[4] = {25, 100, 400, 1000};
static double KP=gains[0], KI=0, KD=0;
PID myPID(&Input, &Output, &Setpoint, KP, KI, KD, DIRECT);

double timer_diff, rpm_timer = 0, pos_diff;
float angular_vel;
void calc_RPM(double Current_Angle);

AMS_5600 ams5600;

int ang, lang = 0;

void setup()
{
  Serial2.begin(115200);
  Wire.begin();
  Wire.setTimeout(1000);
  pinMode(PB8, OUTPUT); //Pins para el ouput PWM del motor
  pinMode(PB9, OUTPUT); //^^^
  analogWriteResolution(10); //Resolucion de 1023
  Setpoint = 100;
  Input = convertRawAngleToDegrees(ams5600.getRawAngle());
  Previous_Angle = Input;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(20);
  myPID.SetOutputLimits(max_down + dead_zone_down, max_up-dead_zone_up);
  analogWriteFrequency(20000);
  Serial2.println("STARTED");
  delay(5000);
}

void loop()
{
    // Deteccion del magneto por el enoder
    if(ams5600.detectMagnet() == 0){
      while(1){
        if(ams5600.detectMagnet() == 1) break;

        else
        {
          Serial2.println("Can not detect magnet");
          analogWrite(PB8, 1);
          analogWrite(PB9, 1);
          Serial2.println(ams5600.getMagnetStrength());  
        }
      }
    }

    // Visualizar la lectura del angulo
    // Serial2.println("------------------------------");
    Input = convertRawAngleToDegrees(ams5600.getRawAngle());

    tolerancia = 1;
    tol_up = Setpoint+tolerancia;
    tol_down = Setpoint-tolerancia;
    error = Input - Setpoint;

    myPID.Compute();
    // Serial2.println(Input);
    position_correction(Input, Output);
    calc_RPM(Input);
}

void calc_RPM(double Current_Angle)
{
  timer_diff = (millis() - rpm_timer)/1000;

  if (timer_diff > 0.02)
  {
    // Serial2.println(timer_diff);
    pos_diff = Current_Angle - Previous_Angle;
    // Serial2.println(pos_diff);
    Previous_Angle = Current_Angle;
    angular_vel = (pos_diff/(timer_diff))/6;
    //Serial2.println(Current_Angle);
    // Serial2.println(Current_Angle);
    Serial2.println(angular_vel);
    rpm_timer = millis();
  }
}

void position_correction(float lectura, double val)
{
  if(lectura > tol_down && lectura < tol_up)
  {
    analogWrite(PB8, 1);
    analogWrite(PB9, 1);
    if(!done){
      Final_Time = millis() - Previous_Time;
      done = true;
    }
    else if(done && millis() > Final_Time + Previous_Time + 5000){
      Serial2.print("Final Time:");
      Serial2.println(Final_Time/1000);
      Serial2.print("Current Time:");
      Serial2.println(millis()/1000);
      Serial2.print(abs(Setpoint - Previous_Angle));
      Serial2.println(" degree change");
      while(Serial2.available() == 0){}
      Setpoint = Serial2.readString().toInt();
      Previous_Angle = Setpoint;
      Previous_Time = millis();
    }
  }
  else if (val  > 0)
  {
    val += dead_zone_up;
    if(val > max_up)
    {
      val = max_up;
    }

    // Serial2.println(val);
    analogWrite(PB9, 0);
    // Serial2.println(val);
    analogWrite(PB8, val);
    // digitalWrite(PB9, 0);
    // digitalWrite(PB8, 1);
    // Serial2.println("CCW or Positive");

    if(done){ done = false;}
  }
  else
  {
    val -= dead_zone_down;
    if(val < max_down)
    {
      val = max_down;
    }
    // Serial2.println(val);
    // digitalWrite(PB8, 0);
    // digitalWrite(PB9, 1);
    analogWrite(PB8, 0);
    // Serial2.println(val);
    analogWrite(PB9,-val);
    // Serial2.println("CW or Negative");

    if(done){ done = false;}
  }
}


double convertRawAngleToDegrees(word newAngle)
{
  if (newAngle == 0xFFFF) return 0xFFFF;
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  float retVal = newAngle * 0.087890625;
  return retVal;
}
