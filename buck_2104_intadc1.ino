//#define DEBUG

char commandBuffer[20];
enum WorkSpcRole {CMD_IDLE, CMD_HELP, CMD_VERBOSE, CMD_DUTY, /*CMD_REGULATE,*/ CMD_DUMPLOAD};

const int IinPin = A6;
const int VinPin = A7;
const int VoutPin = A0;
const int highPin = 3;
const int dumploadPin = 11;
const int enableDriverPin = 2;
const int externActivatePin = 5;

float pwmMin = 30;
float pwmMax = 242;
float duty_cycle = pwmMin;

unsigned long timeStamp, timeStamp1, timeStamp2, timeStamp_voltage, timeStamp_duty = 0;

enum circuitStateEnum {INIT, CIRCUIT_DOWN, CIRCUIT_READY};
int circuitState = INIT;

float VoutMax = 14;
float VinMax = 28;

#ifdef DEBUG
float VoutMin = -10;
#else
float VoutMin = 10;
#endif

float IoutMax = 10;
float inputWattOld = 0.;

int verboseLevel = 2;

float Vin_filter=0, Vout_filter=0, Iin_filter=0, externActivate_filter=0;
float Vin_filter_old = 0.;

#define ADC0 0
#define ADC6 6
#define ADC7 7

// Define the ADC channels used.  ADLAR will be zero.
#define ADCH0 ((1 << REFS0) | ADC0)
#define ADCH6 ((1 << REFS0) | ADC6)
#define ADCH7 ((1 << REFS0) | ADC7)

volatile int analogValue0;
volatile int analogValue6;
volatile int analogValue7;
volatile int adcChan;

void AdcIntSetup()
{
  // Set for the first ADC channel.
  ADMUX = ADCH0;

  // Set ADEN, prescale (128), and enable interrupts.
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADIE);

  // Clear everything.  No free running because ADATE is clear in ADSRA.
  ADCSRB = 0;

  // Enable global interrupts.
  sei();

  // Kick off the first ADC.
  analogValue0 = 0;
  analogValue6 = 0;
  analogValue7 = 0;

  // Set ADSC to start ADC conversion.
  ADCSRA |= (1 << ADSC);
}

// The ADC interrupt function.
ISR(ADC_vect)
{
  // Get the ADC channel causing the interrupt.
  int adcChanTmp = ADMUX & ((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));

  switch (adcChanTmp)
  {
    case ADC0:
      analogValue0 = ADCL | (ADCH << 8);
      ADMUX = ADCH6;
      // Set ADSC to start the next ADC conversion.
      ADCSRA |= (1 << ADSC);
      break;
    case ADC6:
      analogValue6 = ADCL | (ADCH << 8);
      ADMUX = ADCH7;
      // Set ADSC to start the next ADC conversion.
      ADCSRA |= (1 << ADSC);
      break;
    case ADC7:
      analogValue7 = ADCL | (ADCH << 8);
      ADMUX = ADCH0;
      break;
  }

  adcChan = adcChanTmp;
}

long int samplesCount = 0;

void sample(float k=0.002)
{
  float k1 = 1. - k;
  while (adcChan != ADC7);
  adcChan = ADC0;
  int a0 = analogValue0;
  int a6 = analogValue6;
  int a7 = analogValue7;
  // Set ADSC to start the next ADC conversion.
  ADCSRA |= (1 << ADSC);
  
  //int Vin_ = analogRead(VinPin);
  //float Vin = 50.*(float(Vin_)/1023.);
  float Vin = 50.*(float(a7)/1023.);
  Vin_filter = k1*Vin_filter + k*Vin;
  //int Vout_ = analogRead(VoutPin);
  //float Vout = 50.*(float(Vout_)/1023.);
  float Vout = 50.*(float(a0)/1023.);
  Vout_filter = k1*Vout_filter + k*Vout;
  //int Iin_ = analogRead(IinPin);
  //float Iin = (5. * float(Iin_ - 511) / 1024.) / 0.066 - 0.04;
  float Iin = (5. * float(a6 - 511) / 1024.) / 0.066 - 0.04;
  Iin_filter = k1*Iin_filter + k*Iin;

#ifdef DEBUG
  float externActivate = 1;
#else
  float externActivate = digitalRead(externActivatePin);
#endif

  externActivate_filter = 0.95*externActivate_filter + 0.05*externActivate;

  pwmMin = 255 * Vout_filter / Vin_filter - 1;
  if (pwmMin < 30)
    pwmMin = 30;

  samplesCount++;
}

void initialize_serial();
void printHelp();
int getCommand();

void breakNow(int voltageBegin=0, int voltageEnd=0, int duty=255, int rampedDuty=255, bool dump=false)
{
  digitalWrite(enableDriverPin, false);
  unsigned long lastTimeDuty = 0;
  float ramped_duty = 0;

  if (dump)
    Serial.println("Dumping V -> amps/volt (dI/dV) curve (voltage,A/V):");
  
  unsigned long timeBegin = millis();
  while (true)
  {
    sample(0.02);
    if (millis() > timeBegin + 20000)
    {
      Serial.println("Timeout");
      break;
    }

    if (Vin_filter > voltageBegin)
      break;
  }

  float lastVoltage = Vin_filter;
  float lastCurrent = Iin_filter;
  timeBegin = millis();
  while (true)
  {
    sample(0.02);

    unsigned long currentTime = millis();
    if (currentTime > timeBegin + 20000)
    {
      Serial.println("Timeout");
      break;
    }

    if((Vin_filter < voltageEnd) || (Vout_filter < VoutMin) || (Vin_filter < (Vout_filter+0.4)))
      break;

    if (currentTime > (lastTimeDuty + 50))
    {
      if (ramped_duty < duty)
        ramped_duty++;
      analogWrite(dumploadPin, ramped_duty);

      lastTimeDuty = currentTime;

      float dV = Vin_filter - lastVoltage;

      lastVoltage = Vin_filter;
      float dC = Iin_filter - lastCurrent;
      lastCurrent = Iin_filter;
      //float wattage = -dV * dC / 0.050;
      float dI = dC / -dV;

      if (dump)
      {
        Serial.print(Vin_filter);
        Serial.print(",");
        Serial.println(dI);
      }
    }
  }
  analogWrite(dumploadPin, 0);
  circuitState = INIT;
}

void execCommand()
{
  switch (commandBuffer[0])
  {
  case CMD_VERBOSE:
    {
      sscanf((char*)(commandBuffer+1), "%d", &verboseLevel);
    }
    break;
  case CMD_DUTY:
    {
      digitalWrite(enableDriverPin, true);
      int counter = 0;
      int duty;
      sscanf((char*)(commandBuffer+1), "%d", &duty);
      while (!getCommand())
      {
        if (duty>duty_cycle)
          duty_cycle++;
        else if (duty<duty_cycle)
          duty_cycle--;

        analogWrite(highPin, duty_cycle);
        sample();

        counter++;
        if (counter>100)
        {
          counter=0;
          float inputWatt = Vin_filter*Iin_filter;
          float Iout = inputWatt/Vout_filter;
          dump(Iout, inputWatt);
        }
        delay(1);
      }
      circuitState = INIT;
      digitalWrite(enableDriverPin, false);
      //return true;
    }
    break;
  case CMD_DUMPLOAD:
    {
      int voltageBegin = -1, voltageEnd = -1, duty = -1;
      sscanf((char*)(commandBuffer+1), "%d %d %d", &voltageBegin, &voltageEnd, &duty);
      if (voltageBegin == -1 || voltageEnd == -1 || duty == -1)
      {
        Serial.println("wrong parameters!");
        break;
      }

      breakNow(voltageBegin, voltageEnd, duty, 0, true);

      circuitState = INIT;
    }
    break;
  case CMD_HELP:
    printHelp();
    break;
  default:
    break;
  }

  commandBuffer[0] = CMD_IDLE;
}

void dump(float& Iout, float& watt)
{
  //printf("duty=%f, Vin=%fV, Iin_filter=%fA, Vout=%fV, Iout=%fA, watt=%fW\n", duty_cycle, Vin_filter, Iin_filter, Vout_filter, Iout, watt);
  Serial.print("duty=");
  Serial.print(duty_cycle);
  Serial.print(", Vin=");
  Serial.print(Vin_filter);
  Serial.print("V, Iin_filter=");
  Serial.print(Iin_filter);
  Serial.print("A, Vout=");
  Serial.print(Vout_filter);
  Serial.print("V, Iout=");
  Serial.print(Iout);
  Serial.print("A, watt=");
  Serial.print(watt);
  Serial.print("W, smpls=");
  Serial.println(samplesCount);
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

bool shutDownFlag = false;

void evalOnState(float watt)
{
  float Vin_corrected;
  unsigned long currentTime = millis();
  switch (circuitState)
  {
    case INIT:
      if (timeStamp == 0)
        timeStamp = currentTime;
      else if (currentTime > (timeStamp + 2000))
      {
        Serial.println("State: Circuit Down");
        circuitState = CIRCUIT_DOWN;
      }
      break;
    case CIRCUIT_DOWN:

      if (externActivate_filter < 0.5) //digitalRead(externActivatePin) == LOW)
        break;
      
      Vin_corrected = Vin_filter * pwmMax/255. - 0.5;
      if ((Vin_corrected > Vout_filter))
      {
        if ((Vout_filter > VoutMin) && (Vout_filter < VoutMax))
        {
          duty_cycle = pwmMin;
          analogWrite(highPin, duty_cycle);
          digitalWrite(enableDriverPin, true);
          digitalWrite(LED_BUILTIN, true);
          Serial.println("State: Gate Driver Enabled. Circuit Ready");
          circuitState = CIRCUIT_READY;
          timeStamp = currentTime;
        }
      }
      break;
    case CIRCUIT_READY:
      bool ok = true;

      if (externActivate_filter < 0.5) //digitalRead(externActivatePin) == LOW)
      {
        digitalWrite(enableDriverPin, false);
        Serial.println("externActivatePin is LOW");
        ok = false;
      }
      
      if (Iin_filter < -0.02)
      {
        digitalWrite(enableDriverPin, false);
        Serial.print("Iin_filter < -0.02");
        ok = false;
      }
      Vin_corrected = Vin_filter * pwmMax/255.;
      if (Vin_corrected <= Vout_filter)
      //if (Vin_filter <= Vout_filter)
      {
        digitalWrite(enableDriverPin, false);
        Serial.print("Vin_filter <= Vout_filter: ");
        Serial.print(Vin_filter);
        Serial.print(" <= ");
        Serial.println(Vout_filter);
        ok = false;
      }
      if (Vout_filter >= VoutMax && duty_cycle <= pwmMin)
      {
        digitalWrite(enableDriverPin, false);
        Serial.print("Vout_filter >= VoutMax: ");
        Serial.print(Vout_filter);
        Serial.print(" >= ");
        Serial.println(VoutMax);
        ok = false;
      }
      if (Vout_filter < VoutMin && duty_cycle >= pwmMax)
      {
        digitalWrite(enableDriverPin, false);
        Serial.print("Vout_filter < VoutMin: ");
        Serial.print(Vout_filter);
        Serial.print(" < ");
        Serial.println(VoutMin);
        ok = false;
      }
      if (Vin_filter > VinMax && duty_cycle >= pwmMax)
      {
        Serial.print("Vin_filter > VinMax: BREAKING to 16V");
        breakNow(16);
      }
      if (shutDownFlag)
      {
        //if (watt < 1)
        if (Iin_filter < 0.04)
        {
          if (currentTime > (timeStamp+4000))
          {
            digitalWrite(enableDriverPin, false);
            //Serial.println("watt < 1 timeout");
            Serial.println("I < 0.04 timeout");
            while (millis() < (timeStamp+8000))
            {
              sample();
              delay(1);
            }
            ok = false;
          }
        }
        else
        {
          shutDownFlag = false;
          timeStamp = currentTime;
        }
      }
      //else if (watt < 1 && currentTime > (timeStamp+10000))
      else if (Iin_filter < 0.04 && currentTime > (timeStamp+10000))
      {
        timeStamp = currentTime;
        shutDownFlag = true;
      }

      if (!ok)
      {
        Serial.println("State: Circuit Down");
        circuitState = CIRCUIT_DOWN;
        shutDownFlag = false;
        digitalWrite(LED_BUILTIN, false);
      }
      break;
    default:
      break;
  }
}


const float trackingStepConst = 0.5;
float targetInputVoltage = 16.;

void setup()
{
  // put your setup code here, to run once:
  pinMode(highPin, OUTPUT);
  pinMode(dumploadPin, OUTPUT);
  pinMode(enableDriverPin, OUTPUT);
  pinMode(externActivatePin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, false);

  TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  
  digitalWrite(enableDriverPin, false);
  analogWrite(highPin, 0);
  analogWrite(dumploadPin, 0);

  AdcIntSetup();

  Serial.begin(2400);
  initialize_serial();

  Serial.println("UP");
}

void loop()
{
  unsigned long currentTime = millis();
  
  if (getCommand())
    execCommand();

  sample();
  float inputWatt = Vin_filter*Iin_filter;
  float Iout = inputWatt/Vout_filter;

  evalOnState(inputWatt);

  if (circuitState == CIRCUIT_READY)
  {
    if((Vout_filter>VoutMax) || (inputWatt > 1 && Iout>IoutMax))
    {
      duty_cycle -= 1; 
    }
    else
    {
      if (currentTime > timeStamp_voltage + 500)
      {
        timeStamp_voltage = currentTime;
    
        float inputWattDelta = inputWatt - inputWattOld;
        inputWattOld = inputWatt;
        float inputVoltageDelta = Vin_filter - Vin_filter_old;
        Vin_filter_old = Vin_filter;

        //float dI_kinetic = getInterpolatedY(Vin_filter);
        //float dW_kinetic = inputVoltageDelta * dI_kinetic;
        //inputWattDelta += dW_kinetic;

        float trackingStep;
        if (inputWattDelta > 0.)
        {
          if (inputVoltageDelta > 0.)
            targetInputVoltage += trackingStepConst;
          else
            targetInputVoltage -= trackingStepConst;
        }
        else
        {
          if (inputVoltageDelta > 0.)
            targetInputVoltage -= trackingStepConst;
          else
            targetInputVoltage += trackingStepConst;
        }

        // Clamp at max/min 2V distance
        if (targetInputVoltage > (Vin_filter+2.))
          targetInputVoltage = Vin_filter+2.;
        else if (targetInputVoltage < (Vin_filter-2.))
          targetInputVoltage = Vin_filter-2.;

        switch (verboseLevel)
        {
          case 1:
            Serial.println(duty_cycle);
            break;
          case 2:
            Serial.print("duty=");
            Serial.print(duty_cycle);
            Serial.print(", dW=");
            Serial.print(inputWattDelta);
            Serial.print(", dV=");
            Serial.print(inputVoltageDelta);
            Serial.print(", vol=");
            Serial.print(Vin_filter);
            Serial.print(", refvol=");
            Serial.print(targetInputVoltage);
            Serial.print(", outvol=");
            Serial.print(Vout_filter);
            Serial.print(", I=");
            Serial.print(Iin_filter);
            Serial.print(", W=");
            Serial.print(inputWatt);
            Serial.print(", smpls=");
            Serial.println(samplesCount);
            break;
          default:
            break;
        }
        samplesCount = 0;

        if (targetInputVoltage < 12.)
        {
          targetInputVoltage = 13.;
          if (verboseLevel > 0)
            Serial.println("resetting tracking");
        }
        else if (targetInputVoltage > VinMax)
        {
          targetInputVoltage = VinMax;
        }
       
      }

      if (currentTime > timeStamp_duty + 50)
      {
        timeStamp_duty = currentTime;
        duty_cycle -= targetInputVoltage - Vin_filter;
      }

    }

    //duty_cycle = constrain(duty_cycle, 30, 230);
    if (duty_cycle > pwmMax)
      duty_cycle = pwmMax;
    else if (duty_cycle < pwmMin)
      duty_cycle = pwmMin;
      
    analogWrite(highPin, duty_cycle);
    //analogWrite(lowPin, duty_cycle+d);
  }
  else if (verboseLevel > 0)
  {
    if (currentTime > (timeStamp1+5000))
    {
      dump(Iout, inputWatt);
      samplesCount=0;
      timeStamp1 = currentTime;
    }
  }

  //delayMicroseconds(500);
}
