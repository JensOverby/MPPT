#define PWM_START_DUTY   50
#define PWM_STEADY_DUTY   100
#define PHASE_THRESHOLD 50
#define COMMUTATION_CYCLES_PER_SEC 40

#define A_SD 11
#define B_SD 10
#define C_SD 9
#define A_IN 12
#define B_IN 4
#define C_IN 2
#define DEBUG_PIN 13

enum{INIT,WAITING_FOR_CANCEL,WAITING,SPINUP,RUNNING,BREAKING,TEST} state = INIT;

byte bldc_step = 0, motor_speed;
unsigned int i;

volatile bool toggle = false;

unsigned long spinupTime, runningTime=0;

void setup()
{
  pinMode(A_IN, OUTPUT);
  pinMode(B_IN, OUTPUT);
  pinMode(C_IN, OUTPUT);

  pinMode(A_SD, OUTPUT);
  pinMode(B_SD, OUTPUT);
  pinMode(C_SD, OUTPUT);

  freeWheel();

  // Timer1 module setting: set clock source to clkI/O / 1 (no prescaling)
  //TCCR1A = 0;
  TCCR1A =  0x21 | 0x81;
  TCCR1B = 0x01;
  // Timer2 module setting: set clock source to clkI/O / 1 (no prescaling)
  //TCCR2A = 0;
  TCCR2A =  0x81;
  TCCR2B = 0x01;
  // Analog comparator setting
  ACSR   = 0x10;           // Disable and clear (flag bit) analog comparator interrupt

  motor_speed = PWM_START_DUTY;

  pinMode(DEBUG_PIN, OUTPUT);

  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  initWindSensor();

  digitalWrite(DEBUG_PIN, HIGH);

  delay(3000);
}

// Analog comparator ISR
ISR (ANALOG_COMP_vect)
{
  if (toggle)
    return;
  toggle = true;
  //digitalWrite(DEBUG_PIN, toggle? HIGH : LOW);

  bldc_move();
  bldc_step++;
  bldc_step %= 6;

  // BEMF debounce
  /*for(i = 0; i < 50; i++) {
    if(bldc_step & 1){
      if(!(ACSR & 0x20)) i -= 1;
    }
    else {
      if((ACSR & 0x20))  i -= 1;
    }
  }*/
}

void bldc_move()        // BLDC motor commutation function
{
  switch(bldc_step){
    case 0:
      AH_BL();
      BEMF_C_RISING();
      break;
    case 1:
      AH_CL();
      BEMF_B_FALLING();
      break;
    case 2:
      BH_CL();
      BEMF_A_RISING();
      break;
    case 3:
      BH_AL();
      BEMF_C_FALLING();
      break;
    case 4:
      CH_AL();
      BEMF_B_RISING();
      break;
    case 5:
      CH_BL();
      BEMF_A_FALLING();
      break;
  }
}
 
void BEMF_A_RISING()
{
  ADCSRB = (0 << ACME);    // Select AIN1 as comparator negative input
  ACSR |= 0x03;            // Set interrupt on rising edge
}

void BEMF_A_FALLING()
{
  ADCSRB = (0 << ACME);    // Select AIN1 as comparator negative input
  ACSR &= ~0x01;           // Set interrupt on falling edge
}

void BEMF_B_RISING()
{
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 as comparator negative input
  ACSR |= 0x03;
}

void BEMF_B_FALLING()
{
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 as comparator negative input
  ACSR &= ~0x01;
}

void BEMF_C_RISING()
{
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 as comparator negative input
  ACSR |= 0x03;
}

void BEMF_C_FALLING()
{
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 as comparator negative input
  ACSR &= ~0x01;
}

//ADCSRA &= ~ (1 << ADEN);

void AH_BL()
{
  analogWrite(A_SD, motor_speed);
  digitalWrite(A_IN, HIGH);
  
  digitalWrite(B_SD, HIGH);
  digitalWrite(B_IN, LOW);
  
  digitalWrite(C_SD, LOW);
  digitalWrite(C_IN, LOW);
}

void AH_CL()
{
  analogWrite(A_SD, motor_speed);
  digitalWrite(A_IN, HIGH);

  digitalWrite(B_SD, LOW);
  digitalWrite(B_IN, LOW);
  
  digitalWrite(C_SD, HIGH);
  digitalWrite(C_IN, LOW);
}

void BH_CL()
{
  digitalWrite(A_SD, LOW);
  digitalWrite(A_IN, LOW);

  analogWrite(B_SD, motor_speed);
  digitalWrite(B_IN, HIGH);
  
  digitalWrite(C_SD, HIGH);
  digitalWrite(C_IN, LOW);
}

void BH_AL()
{
  digitalWrite(A_SD, HIGH);
  digitalWrite(A_IN, LOW);

  analogWrite(B_SD, motor_speed);
  digitalWrite(B_IN, HIGH);
  
  digitalWrite(C_SD, LOW);
  digitalWrite(C_IN, LOW);
}

void CH_AL()
{
  digitalWrite(A_SD, HIGH);
  digitalWrite(A_IN, LOW);

  digitalWrite(B_SD, LOW);
  digitalWrite(B_IN, LOW);
  
  analogWrite(C_SD, motor_speed);
  digitalWrite(C_IN, HIGH);
}

void CH_BL()
{
  digitalWrite(A_SD, LOW);
  digitalWrite(A_IN, LOW);

  digitalWrite(B_SD, HIGH);
  digitalWrite(B_IN, LOW);
  
  analogWrite(C_SD, motor_speed);
  digitalWrite(C_IN, HIGH);
}

void freeWheel()
{
  ADCSRA = 135;
  ADCSRB = 0;

  digitalWrite(A_SD, LOW);
  digitalWrite(A_IN, LOW);

  digitalWrite(B_SD, LOW);
  digitalWrite(B_IN, LOW);
  
  digitalWrite(C_SD, LOW);
  digitalWrite(C_IN, LOW);
}

void makeAcknowledge(bool positive)
{
  digitalWrite(DEBUG_PIN, LOW);
  delay(300);
  
  motor_speed = PWM_START_DUTY;

  if (positive)
  {
    for (int j=0; j<6; j++)
    {
      delay(200);
      bldc_move();
      bldc_step++;
      bldc_step %= 6;
    }
  }
  else
  {
    for (int j=0; j<6; j++)
    {
      delay(200);
      bldc_move();
      bldc_step--;
      if (bldc_step < 0)
        bldc_step = 5;
    }
  }
  freeWheel();

  digitalWrite(DEBUG_PIN, HIGH);
}

bool isRunning(unsigned long& refTime)
{
  unsigned int valuePhaseB = analogRead(A2);
  unsigned long now = millis();
  if (valuePhaseB > PHASE_THRESHOLD)
    refTime = now;
  else if ((now-refTime) > 5000)
    return false;
  return true;
}

const float data[][2]= {{0.04,60}, {0.02,75}, {0.01,100}, {0.007,135}, {0.005,170}, {0.004,205}};
const int data_sz = sizeof(data)/8;
int data_ptr = 0;
float interpolate(float x0, float y0, float x1, float y1, float x) { return y0 + (x-x0)/(x1-x0) * (y1-y0); }

float getInterpolatedY(float x)
{
  if (x > data[0][0])
    return 0.;
  if (x <= data[data_sz-1][0])
    return data[data_sz-1][1];

  while (data[data_ptr+1][0] > x)
    data_ptr++;
  while (data[data_ptr][0] <= x)
    data_ptr--;
  float y = interpolate(data[data_ptr][0],data[data_ptr][1],data[data_ptr+1][0],data[data_ptr+1][1],x);
  return y;
}

unsigned int debArray[100];
unsigned int count_ = 0;

void loop()
{
  /*motor_speed = 40;
  bldc_step = 0;
  bldc_move();
  while (true)
    delay(1000);*/

  switch(state)
  {
    case INIT:
      {
        unsigned int valuePhaseB = analogRead(A2);
        Serial.print("INIT: measured speed value = ");
        Serial.println(valuePhaseB);
        if (valuePhaseB > PHASE_THRESHOLD)
        {
          runningTime = millis();
          while (isRunning(runningTime))
            delay(50);

          makeAcknowledge(true);
          windSpeedClear();
          runningTime = millis();
          state = WAITING_FOR_CANCEL;
          Serial.println("WAITING");
        }
        delay(500);
      }
      break;
    case WAITING_FOR_CANCEL:
      {
        unsigned long now = millis();
        if ((now - runningTime) > 25000)
        {
          state = WAITING;
          runningTime = now;
        }

        unsigned int valuePhaseB = analogRead(A2);
        if (valuePhaseB > PHASE_THRESHOLD)
        {
          runningTime = millis();
          while (isRunning(runningTime))
            delay(50);

          makeAcknowledge(false);
          state = INIT;
        }

        delay(20);
      }
      break;
    case WAITING:
      {
        if (getWindSpeedOK())
        {
          state = SPINUP;
          Serial.println("SPINUP");
        }
        
        unsigned int valuePhaseB = analogRead(A2);
        if (valuePhaseB > PHASE_THRESHOLD)
        {
          runningTime = millis();
          while (isRunning(runningTime))
            delay(50);

          makeAcknowledge(true);
          state = INIT;
        }

        delay(20);
      }
      break;
    case SPINUP:
      {
        digitalWrite(DEBUG_PIN, LOW);
        delay(300);

        spinupTime = millis();

        const float commutation_angle = 3.14159/3;
        float dT = 0.05;

        while (true)
        {
          float angular_speed = commutation_angle / dT;
          angular_speed += 20.0*dT;//50.0 * dT;
          dT = commutation_angle / angular_speed;
  
          motor_speed = getInterpolatedY(dT);

          bldc_move();
          bldc_step++;
          bldc_step %= 6;

          if (dT < 0.016)
            delayMicroseconds(dT*1000000);
          else
            delay(dT*1000);

          //if (dT < 0.01)
          //  break;
          if (motor_speed > PWM_STEADY_DUTY)
            break;
        }        
        
        unsigned long globalMaxComTime = dT*1000000;
        unsigned long comTime = globalMaxComTime;
        unsigned long t_old = micros();

        motor_speed = PWM_STEADY_DUTY;
        unsigned int turnCounter=0, turnCounterOld = 0;
        unsigned long nextCheckTime = micros() + 1000000;
        bool cancelled = false;
        toggle = true;
        bldc_step += 1;
        bldc_step %= 6;
        count_ = 0;
        ACSR |= 0x08;                    // Enable analog comparator interrupt
        delayMicroseconds(comTime/4);// - comTime/4);
        toggle = false;
        while(true)
        {
          unsigned long maxComTime = comTime + comTime/2 + 1;
          if (maxComTime > globalMaxComTime)
            maxComTime = globalMaxComTime;
          unsigned long t;
          unsigned long dt;
          bool forced = false;

          while (true)
          {
            t = micros();
            dt = t - t_old;

            noInterrupts();
            if (dt > maxComTime)
            {
              if (!toggle)
              {
                toggle = true;
                bldc_move();
                bldc_step++;
                bldc_step %= 6;
                forced = true;
              }
            }
            interrupts();
            
            if (toggle)
              break;
          }
          debArray[count_] = comTime;
          if (count_ < 99)
            count_++;

          if (motor_speed < 200)
            motor_speed++;

          if (!forced)
            comTime = 0.1*float(dt) + 0.9*float(comTime);
  
          t_old = t;

          // Prevent BEMF debounce
          delayMicroseconds(comTime/4);// - comTime/4);// - comTime/2);
          toggle = false;
          turnCounter++;

          runningTime = micros();
          if (runningTime > nextCheckTime)
          {
            if (turnCounter > COMMUTATION_CYCLES_PER_SEC*6)
              break;
            if (turnCounter <= turnCounterOld)
            {
              cancelled = true;
              break;
            }
            turnCounterOld = turnCounter;
            turnCounter = 0;
            nextCheckTime += 1000000;
          }
        }

        ACSR   = 0x10;           // Disable and clear (flag bit) analog comparator interrupt
        freeWheel();
        delay(100);

        digitalWrite(DEBUG_PIN, HIGH);
        
        for (count_=0;count_<100;count_++)
          Serial.println(debArray[count_]);

        Serial.print("cycles/sec: ");
        Serial.println(turnCounter/6);
        if (cancelled)
          Serial.println("CANCEL SPINUP");
        runningTime = millis();
        state = RUNNING;
        Serial.println("RUNNING");

        //delay(5000);
      }
      break;
    case RUNNING:
      {
        if (!isRunning(runningTime))
        {
          unsigned long now = millis();
          Serial.println("Turbine stopped.");

          if ((now - spinupTime) < 2*60000)
          {
            Serial.println("Startup failed");
            windSensorFeedback(false);
          }
          else if ((now - spinupTime) > 5*60000)
          {
            Serial.println("Running time > 5 minutes");
            windSensorFeedback(true);
          }
          
          Serial.println("Entering WAITING state");
          windSpeedClear();
          runningTime = millis();
          state = WAITING_FOR_CANCEL;
        }
      }
      break;
    case TEST:
      {
        Serial.println("TEST");
        delay(1000);
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
