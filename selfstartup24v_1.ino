                                                                                                  #include <SFE_BMP180.h>
#include <Wire.h>
SFE_BMP180 pressure;

const int aHpin =  10; // pwm
const int aLpin =  11; // pwm (used as normal output)
const int bHpin =  9; // pwm
const int bLpin =  8; // normal output
const int cHpin =  3; // pwm
const int cLpin =  7; // normal output
const int ledpin = 13;
const int voltagePhaseApin = A1;
double pressureThreshold = 0.01;
float voltagePhaseA;
const int dutycycleInit = 60;
int dutycycle = dutycycleInit;
enum{INIT,WAITING_FOR_CANCEL,WAITING,WARNING,SPINUP,RUNNING,BREAKING,TEST} state = INIT;
int commutation = 0;
const float commutation_angle = 3.14159/3;
const float voltageScale = 48./1024.;
int voltageThreshold = 13;
int ledToggle = LOW;
float dT;
unsigned long latestRunningTime = 0;
unsigned long currentTime;
unsigned long startupTime;
double pressure_mean, pressure_stddev_mean;



void freeWheel()
{
  Serial.println("Freewheel");
  digitalWrite(cLpin, LOW);
  digitalWrite(bLpin, LOW);
  digitalWrite(aLpin, LOW);
  analogWrite(bHpin, 255);
  analogWrite(cHpin, 255);
  analogWrite(aHpin, 255);
}

void getPressure(double& P, double& T)
{
  char status;
  double p0,a;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.
        T = 25.0;
        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return;
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}

void setup() {
  delay(1000);
  pinMode(aHpin, OUTPUT);
  pinMode(aLpin, OUTPUT);
  pinMode(bHpin, OUTPUT);
  pinMode(bLpin, OUTPUT);
  pinMode(cHpin, OUTPUT);
  pinMode(cLpin, OUTPUT);
  pinMode(ledpin, OUTPUT);
  freeWheel();
  analogWrite(ledpin, ledToggle);

  //TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  // Set the frequency by a 3-bit prescale
  //TCCR2B = _BV(CS20);


  // Code for Available PWM frequency for D3 & D11:
  //TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  //TCCR2B = _BV(CS22);
  
  //TCCR2B |= _BV(COM2B1) | _BV(WGM20) | _BV(WGM21);
  //TCCR2B = TCCR2B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
  
  // Code for Available PWM frequency for D9 & D10:  
  //TCCR1B = TCCR1B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
  //TCCR1B |= _BV(COM1B1) | _BV(WGM10) | _BV(WGM11);
  //TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
  //TCCR1B = _BV(CS12);
  //cli();
  TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz

  //TCCR0B = TCCR0B & B11111000 | B00000010;
  
  
  // Non inverted:
  //TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);

  // Inverted:
  //TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(COM2A0) | _BV(COM2B0) | _BV(WGM20);  

  /*
  // set prescaler to 8 and starts PWM
  TCCR0B |= (1 << CS01);

  // set PWM for 50% duty cycle
  OCR0A = 128;
  */

  
  //sei();
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);


  // Initialize pressure sensor
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    Serial.println("BMP180 init fail (disconnected?)\n\n");
    //while(1); // Pause forever.
  }

  delay(3000);
}

void state0()
{
  // AH BL
  analogWrite(cHpin, 255);
  analogWrite(aHpin, 255-dutycycle);
}

void state1()
{
  // AH CL
  digitalWrite(bLpin, LOW);
  digitalWrite(cLpin, HIGH);
}

void state2()
{
  // BH CL
  analogWrite(aHpin, 255);
  analogWrite(bHpin, 255-dutycycle);
}

void state3()
{
  // BH AL
  digitalWrite(cLpin, LOW);
  digitalWrite(aLpin, HIGH);
}

void state4()
{
  // CH AL
  analogWrite(bHpin, 255);
  analogWrite(cHpin, 255-dutycycle);
}

void state5()
{
  // CH BL
  digitalWrite(aLpin, LOW);
  digitalWrite(bLpin, HIGH);
}

void lockOnState0()
{
  freeWheel();
  state5();
  for (int i=0; i<dutycycle; i++)
  {
      analogWrite(aHpin, 255-i);
      delay(10);
  }
  delay(1000);
  freeWheel();
}

int morse[] = {
  3, 1, 3, 1, 3, 3, -6
};

void warning(int duty, int pin)
{
  for (int thisNote = 0; thisNote < 7; thisNote++) {
    int dur = morse[thisNote]*12;
    if (dur>0)
    {
      analogWrite(pin, 255-duty);
      delay(dur);
    }
    else
    {
      analogWrite(pin, 255);
      delay(-dur);
    }
    analogWrite(pin, 255);
    delay(50);
  }
}

void playWarning(int duty)
{
  freeWheel();
  state0();
  warning(duty, aHpin);
  state1();
  warning(duty, aHpin);
  state2();
  warning(duty, bHpin);
  state3();
  warning(duty, bHpin);
  state4();
  warning(duty, cHpin);
  state5();
  warning(duty, cHpin);
}

int interpolate(float x0, int y0, float x1, int y1, float x)
{
  return y0 + int((x-x0)/(x1-x0) * float(y1-y0));
}

int dutyState=-1;
int getDuty(float dT)
{
  int duty, ds;
  if (dT < 0.04)
  {
    if (dT < 0.02)
    {
      if (dT < 0.01)
      {
        if (dT < 0.007)
        {
          if (dT < 0.005)
          {
            if (dT < 0.004)
            {
              if (dT < 0.0036)
              {
                ds=7;
                duty = interpolate(0.0036,225,0.0007,250,dT);
              }
              else
              {
                ds=6;
                duty = interpolate(0.004,205,0.0036,225,dT);
              }
            }
            else
            {
              ds=5;
              duty = interpolate(0.005,170,0.004,205,dT);
            }
          }
          else
          {
            ds=4;
            duty = interpolate(0.007,135,0.005,170,dT);
          }
        }
        else
        {
          ds=3;
          duty = interpolate(0.01,100,0.007,135,dT);
        }
      }
      else
      {
        ds=2;
        duty = interpolate(0.02,75,0.01,100,dT);
      }
    }
    else
    {
      ds=1;
      duty = interpolate(0.04,60,0.02,75,dT);
    }
  }
  else
  {
    ds=0;
    duty = 60;
  }

  if (ds != dutyState)
  {
    Serial.print("dT = ");
    Serial.println(dT,4);
  }
  dutyState = ds;
  return duty;
}

float maxVoltage = -5.;
unsigned long lastVoltageTime = 0;
unsigned long lastBreakingTime = 0;
int breakOn = LOW;

void loop()
{
  switch(state)
  {
    case INIT:
      {
        voltagePhaseA = analogRead(voltagePhaseApin)*voltageScale;
        Serial.print("INIT: measured voltage = ");
        Serial.println(voltagePhaseA);
        if (voltagePhaseA > voltageThreshold)
        {
          delay(5000);
          lockOnState0();
          latestRunningTime = millis();
          pressure_stddev_mean = 0.0;
          double T;
          getPressure(pressure_mean, T);
          state = WAITING;
          Serial.println("WAITING");
        }
        delay(500);
      }
      break;
    case WAITING_FOR_CANCEL:
      {
        currentTime = millis();
        if ((currentTime - latestRunningTime) > 15000)
        {
          state = WAITING;
          latestRunningTime = currentTime;
        }

        voltagePhaseA = analogRead(voltagePhaseApin)*voltageScale;
        if (voltagePhaseA > voltageThreshold)
        {
          delay(5000);
          lockOnState0();
          state = INIT;
        }

        delay(20);
      }
      break;
    case WAITING:
      {
        double pressure, temp;
        getPressure(pressure, temp);
        pressure_mean += 0.01 * (pressure - pressure_mean);
        float pressure_stddev = fabs(pressure - pressure_mean);
        pressure_stddev_mean = 0.1 * (pressure_stddev - pressure_stddev_mean);
        if (fabs(pressure_stddev_mean) > 0.005)
        {
          Serial.print("std dev: ");
          Serial.print(pressure_stddev, 4);
          Serial.print(", mean: ");
          Serial.print(pressure_stddev_mean, 4);
          Serial.print(", pressure: ");
          Serial.println(pressure, 4);
          Serial.print(", temp: ");
          Serial.println(temp, 4);
        }

        if (pressure_stddev_mean > pressureThreshold)
        {
          currentTime = millis();
          if ((currentTime - latestRunningTime) < 5000)
          {
            state = WARNING;
            startupTime = currentTime;
          }
          latestRunningTime = currentTime;
        }

        voltagePhaseA = analogRead(voltagePhaseApin)*voltageScale;
        if (voltagePhaseA > voltageThreshold)
        {
          delay(5000);
          lockOnState0();
          state = INIT;
        }

        delay(20);
      }
      break;
    /*case LOCK:
      {
        Serial.println("LOCK");
        lockOnState0();
        state = WARNING;
      }
      break;*/
    case WARNING:
      {
        Serial.println("WARNING");
        playWarning(dutycycle);
        //lockOnState0();
        commutation = 0;
        dT = 0.1;//0.05;
        state = SPINUP;
        Serial.println("SPINUP");
      }
      break;
    case RUNNING:
      {
        voltagePhaseA = analogRead(voltagePhaseApin)*voltageScale;
        currentTime = millis();

        if (voltagePhaseA > maxVoltage)
          maxVoltage = voltagePhaseA;
        if ((currentTime - lastVoltageTime) > 500)
        {
          Serial.print("RUNNING: measured voltage = ");
          Serial.println(maxVoltage);
          if (maxVoltage > 996.)
          {
            state = BREAKING;
          }
          lastVoltageTime = currentTime;
          maxVoltage = -5.;
        }
        

        if (voltagePhaseA > voltageThreshold)
        {
          latestRunningTime = currentTime;
        }
        else if ((currentTime-latestRunningTime) > 15000)
        {
          Serial.println("Turbine stopped.");

          if ((currentTime - startupTime) < 2*60000)
          {
            Serial.println("Startup failed");
            pressureThreshold += 0.002;
          }
          else if ((currentTime-startupTime) > 5*60000)
          {
            Serial.println("Running time > 5 minutes");
            pressureThreshold -= 0.002;
          }
          
          delay(10000);
          Serial.println("Entering WAITING state");
          lockOnState0();
          latestRunningTime = millis();
          pressure_stddev_mean = 0.0;
          double temp;
          getPressure(pressure_mean, temp);
          state = WAITING_FOR_CANCEL;
        }
        //delay(1000);
      }
      break;
    case BREAKING:
      {
        currentTime = millis();
        if ((currentTime - lastBreakingTime) > 5)
        {
          lastBreakingTime = currentTime;
          if (breakOn == HIGH)
            breakOn = LOW;
          else
          {
            breakOn = HIGH;
            voltagePhaseA = analogRead(voltagePhaseApin)*voltageScale;
            if (voltagePhaseA > maxVoltage)
              maxVoltage = voltagePhaseA;
            if ((currentTime - lastVoltageTime) > 500)
            {
              //Serial.print("BREAKING: measured voltage = ");
              //Serial.println(maxVoltage);
              if (maxVoltage < 16.)
              {
                state = RUNNING;
                breakOn = LOW;
              }
              lastVoltageTime = currentTime;
              maxVoltage = -5.;
            }
          }

          digitalWrite(cLpin, breakOn);
          digitalWrite(bLpin, breakOn);
          digitalWrite(aLpin, breakOn);
        }
      }
      break;
    case SPINUP:
      {
        switch(commutation)
        {
          case 0:
            state1();
            commutation=1;
            break;
          case 1:
            state2();
            commutation=2;
            break;
          case 2:
            state3();
            commutation=3;
            break;
          case 3:
            state4();
            commutation=4;
            break;
          case 4:
            state5();
            commutation=5;
            break;
          case 5:
            state0();
            commutation=0;
            break;
        }
        ledToggle = !ledToggle;
        digitalWrite(ledpin, ledToggle);
  
        float angular_speed = commutation_angle / dT;
        angular_speed += 30.0*dT;//50.0 * dT;
        dT = commutation_angle / angular_speed;
        //Serial.println(angular_speed);
        //delay(1000);

        dutycycle = getDuty(dT) +2;
        //float duty = -5800.0*dT + 125.0;
        //dutycycle = int(duty);
        //if (dutycycle < 63)
          //dutycycle = 63;
        
        //Serial.println(dutycycle);
  
        if (dT <= 0.0036) //0.004)
        {
          freeWheel();
          latestRunningTime = millis();
          state = RUNNING;
          dutycycle = dutycycleInit;
          Serial.println("RUNNING");
          delay(5000);
        }
  
        if (dT < 0.016)
          delayMicroseconds(dT*1000000);
        else
          delay(dT*1000);
  
        //delay(1000);
      }
      break;
    case TEST:
      {
        /*
        analogWrite(buckPin, 127);
        Serial.print("startval=");
        Serial.println(startval);
        startval -= 1;
        if (startval < 0)
          startval=0;
        
        analogWrite(boostPin, 255);
        
        Serial.print("turbine: current=");
        int currentTurbine = analogRead(currentTurbinePin);
        Serial.print(currentTurbine);
        
        Serial.print(", voltage=");
        int voltageTurbine = analogRead(voltageTurbinePin);
        Serial.println(voltageTurbine);

        Serial.print("battery: voltage=");
        int voltageBattery = analogRead(voltageBatteryPin);
        Serial.println(voltageBattery);
        */
        
        
        digitalWrite(bLpin, HIGH);
        digitalWrite(cLpin, LOW);
        digitalWrite(aLpin, LOW);
        analogWrite(cHpin, 255-dutycycle);
        analogWrite(aHpin, 255);
        analogWrite(bHpin, 255);

        delay(1000);

        digitalWrite(aLpin, LOW);
        digitalWrite(cLpin, LOW);
        digitalWrite(bLpin, LOW);
        analogWrite(cHpin, 255);
        analogWrite(aHpin, 255);
        analogWrite(bHpin, 255);


        //playWarning(80);
        //delay(1000);
        //playWarning(80);
        
        //delay(5000);
        //analogWrite(bHpin, 255);
        //Serial.println("2 sec");

        /*voltagePhaseA = analogRead(voltagePhaseApin);
        Serial.print("TEST: ");
        Serial.print((int)voltagePhaseA);
        voltagePhaseA *= voltageScale;
        Serial.print(", measured voltage = ");
        Serial.println(voltagePhaseA);
*/
        Serial.print("TEST: ");
        Serial.println(commutation);
        commutation++;
        delay(1000);
        

        //playWarning(15);
        
        //delay(64000);
      }
      break;
    default:
      {
        Serial.println("default");
        delay(1000);
      }
      break;
  }
}
