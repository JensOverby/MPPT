char commandBuffer[20];
enum WorkSpcRole {CMD_IDLE, CMD_HELP, CMD_VERBOSE, CMD_DUTY, CMD_REGULATE};

const int IinPin = A6;
const int VinPin = A7;
const int VoutPin = A0;
const int highPin = 3; //10;
const int dumploadPin = 11;
//const int lowPin = 9;
//const int closedCircuitPin = 8;
const int enableDriverPin = 2;

//const int d = 2;
float duty_cycle = 30;
int duty_cycle_old = 0;
unsigned long timeStamp, timeStamp1, timeStamp2, timeStamp_voltage, timeStamp_duty = 0;

enum circuitStateEnum {INIT, CIRCUIT_DOWN, INPUT_VOLTAGE_OK, BATTERY_OK, CIRCUIT_READY};
int circuitState = INIT;

float VoutMax = 12.5;
float VoutMin = -10.0;
float IoutMax = 1.3;
float inputWattOld = 0.;

int Verbose = 1;

float Vin_filter, Vout_filter, Iin_filter = 0.;
float Vin_filter_old = 0.;

//float median[3];
//static int m_i = 0;

void sample()
{
  int Vin_ = analogRead(VinPin);
  float Vin = 50.*(Vin_/1023.);
  Vin_filter = 0.99*Vin_filter + 0.01*Vin;
  int Vout_ = analogRead(VoutPin);
  float Vout = 50.*(Vout_/1023.);
  Vout_filter = 0.99*Vout_filter + 0.01*Vout;
  int Iin_ = analogRead(IinPin);
  float Iin = 30.*(Iin_-512.)/511.;
  Iin_filter = 0.996*Iin_filter + 0.004*Iin;

  /*
  median[m_i] = 30.*(Iin_-512.)/511.;
  m_i++;
  if (m_i > 2)
  {
    m_i = 0;
    float average = (median[0] + median[1] + median[2])/3.;
    float d0 = fabs(median[0]-average);
    float d1 = fabs(median[1]-average);
    float d2 = fabs(median[2]-average);
    int i;
    if (d0 < d1)
    {
      if (d0 < d2)
        i = 0;
      else
        i = 2;
    }
    else if (d1 < d2)
      i = 1;
    else
      i = 2;
  
    Iin_filter = 0.95*Iin_filter + 0.05*median[i];
  }*/
}

void initialize_serial();
void printHelp();
int getCommand();

bool execCommand() //struct axis_state_s* axis)
{
  switch (commandBuffer[0])
  {
  case CMD_IDLE:
    break;
  case CMD_VERBOSE:
    {
      sscanf((char*)(commandBuffer+1), "%d", &Verbose);
    }
    commandBuffer[0] = CMD_IDLE;
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
        //analogWrite(lowPin, duty_cycle+d);
        sample();

        //digitalWrite(enableDriverPin, Vin_filter > Vout_filter);
        //evalOnState();

        //if (Verbose==2)
        {
          counter++;
          if (counter>100)
          {
            counter=0;
            float inputWatt = Vin_filter*Iin_filter;
            float Iout = inputWatt/Vout_filter;
            dump(Iout, inputWatt);
          }
        }
        
        delay(1);
      }
      return true;
    }
    break;
  case CMD_HELP:
    printHelp();
    commandBuffer[0] = CMD_IDLE;
    break;
  default:
    break;
  }
  return false;
}

struct I_Node
{
  float voltage;
  float current;
};

I_Node iCurve[11];
I_Node iCurve_modified[11];
int arrayPointer = 0;



void setup()
{
  iCurve[0].voltage = 12;
  iCurve[0].current = 0;
  iCurve[1].voltage = 14;
  iCurve[1].current = 1.;
  iCurve[2].voltage = 16;
  iCurve[2].current = 2.;
  iCurve[3].voltage = 18;
  iCurve[3].current = 3.;
  iCurve[4].voltage = 20;
  iCurve[4].current = 4.;
  iCurve[5].voltage = 22;
  iCurve[5].current = 5.;
  iCurve[6].voltage = 24;
  iCurve[6].current = 6.;
  iCurve[7].voltage = 26;
  iCurve[7].current = 7.;
  iCurve[8].voltage = 28;
  iCurve[8].current = 8.;
  iCurve[9].voltage = 30;
  iCurve[9].current = 9.;
  iCurve[10].voltage = 32;
  iCurve[10].current = 10.;
  memcpy(iCurve_modified, iCurve, sizeof(iCurve));

  // put your setup code here, to run once:
  pinMode(highPin, OUTPUT);
  pinMode(dumploadPin, OUTPUT);
  //pinMode(lowPin, OUTPUT);
  pinMode(enableDriverPin, OUTPUT);
  //pinMode(closedCircuitPin, OUTPUT);
  
  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1A = (TCCR1A & 0x0F) | 0xC0 ; // set pin 9 inverted
  
  digitalWrite(enableDriverPin, false);
  //digitalWrite(closedCircuitPin, false);
  analogWrite(highPin, 0);
  analogWrite(dumploadPin, 0);

  /*pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS21);
  OCR2A = 50;
  OCR2B = 49;*/

  Serial.begin(9600);

  initialize_serial();

  Serial.println("UP");
  /*Serial.println("CountDown");
  for (int i=10;i>0;i--)
  {
    Serial.print(i);
    delay(1000);
  }
  Serial.println();*/
}

void dump(float& Iout, float& watt)
{
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
  Serial.println("W");
}

float interpolate(float x0, float y0, float x1, float y1, float x)
{
  return y0 + (x-x0)/(x1-x0) * (y1-y0);
}

float relToX0(float x0, float x1, float x)
{
  return (x1 - x) / (x1-x0);
}


float getExpectedCurrent()
{
  if (Vin_filter < iCurve[0].voltage)
    return 0.;
  if (Vin_filter >= iCurve[10].voltage)
    return 9999.;

  while (iCurve[arrayPointer+1].voltage < Vin_filter)
    arrayPointer++;
  while (iCurve[arrayPointer].voltage >= Vin_filter)
    arrayPointer--;
  float current = interpolate(iCurve[arrayPointer].voltage,iCurve[arrayPointer].current,iCurve[arrayPointer+1].voltage,iCurve[arrayPointer+1].current,Vin_filter);
  return current;
}

void updateExpectedCurrent()
{
  unsigned long currentTime = millis();
  if (currentTime > (timeStamp2+2000))
  {
    timeStamp2 = currentTime;
    float dV = Vin_filter - Vin_filter_old;
    int dD = duty_cycle - duty_cycle_old;
    if (dV < 0)
    {
      dV *= -1.;
      dD *= -1;
    }
    Vin_filter_old = Vin_filter;
    duty_cycle_old = duty_cycle;

    
    if (dV < 0.3)
    {
      if (dD > 0)
      {
        if (Vin_filter < iCurve[0].voltage)
          return;
        if (Vin_filter >= iCurve[10].voltage)
          return;

        while (iCurve[arrayPointer+1].voltage < Vin_filter)
          arrayPointer++;
        while (iCurve[arrayPointer].voltage >= Vin_filter)
          arrayPointer--;

        float a = relToX0(iCurve[arrayPointer].voltage, iCurve[arrayPointer+1].voltage, Vin_filter);
        float b = 1. - a;



        
      }
    }
  }

}

void evalOnState()
{
  unsigned long currentTime = millis();
  switch (circuitState)
  {
    case INIT:
      if (timeStamp == 0)
        timeStamp = currentTime;
      else if (currentTime > (timeStamp + 3000))
      {
        if (Verbose==1)
          Serial.println("State: Circuit Down");
        circuitState = CIRCUIT_DOWN;
      }
      break;
    case CIRCUIT_DOWN:
      if ((Vin_filter > VoutMax))
      {
        //digitalWrite(closedCircuitPin, true);
        timeStamp = millis();
        if (Verbose==1)
          Serial.println("State: Input Voltage OK");
        circuitState = INPUT_VOLTAGE_OK;
      }
      break;
    case INPUT_VOLTAGE_OK:
      if (currentTime > (timeStamp + 500))
      {
        if ((Vout_filter > VoutMin) && (Vout_filter < VoutMax))
        {
          //digitalWrite(disableDriverPin, false);
          if (Verbose==1)
            Serial.println("State: Battery OK");
          circuitState = BATTERY_OK;
          timeStamp = currentTime;
        }
      }
      if ((Vin_filter < VoutMax))
      {
        //digitalWrite(closedCircuitPin, false);
        if (Verbose==1)
          Serial.println("State: Circuit Down");
        circuitState = CIRCUIT_DOWN;
      }
      break;
    case BATTERY_OK:
      if (Vin_filter > Vout_filter)
      {
        digitalWrite(enableDriverPin, true);
        analogWrite(highPin, duty_cycle);
        if (Verbose==1)
          Serial.println("State: Gate Driver Enabled. Circuit Ready");
        circuitState = CIRCUIT_READY;
      }
      else
      {
        //digitalWrite(closedCircuitPin, false);
        digitalWrite(enableDriverPin, false);
        if (Verbose==1)
          Serial.println("State: Circuit Down");
        circuitState = CIRCUIT_DOWN;
      }
      break;
    case CIRCUIT_READY:
      if (Vin_filter <= Vout_filter || Vout_filter >= VoutMax || Vout_filter < VoutMin)
      {
        //digitalWrite(closedCircuitPin, false);
        digitalWrite(enableDriverPin, false);
        if (Verbose==1)
          Serial.println("State: Circuit Down");
        circuitState = CIRCUIT_DOWN;
      }
      break;
    default:
      break;
  }
}


const float trackingStepConst = 0.5;
float targetInputVoltage = 16.;

void loop()
{
  /*analogWrite(highPin, duty_cycle);
  while (true)
  {
    digitalWrite(disableDriverPin, true);
    Serial.println("off");
    delay(1000);
    digitalWrite(disableDriverPin, false);
    //analogWrite(highPin, duty_cycle);
    //analogWrite(lowPin, duty_cycle+d);
    Serial.println("on");
    delay(1000);
  }*/

  unsigned long currentTime = millis();
  
  if (getCommand())
  {
    while (execCommand());
  }

  sample();
  float inputWatt = Vin_filter*Iin_filter;
  float Iout = inputWatt/Vout_filter;

  if (Verbose==1)
  {
    if (currentTime > (timeStamp1+5000))
    {
      dump(Iout, inputWatt);
      timeStamp1 = currentTime;
    }
  }

  if (commandBuffer[0]==CMD_REGULATE)
  {
    evalOnState();
  
    if (circuitState == CIRCUIT_READY)
    {
      if((Vout_filter>VoutMax) || (inputWatt > 1 && Iout>IoutMax))
      {
        duty_cycle -= 1; 
      }
      else if ((Vout_filter<VoutMax) && (Iout<IoutMax))
      {

        if (currentTime > timeStamp_voltage + 250)
        {
          timeStamp_voltage = currentTime;
      
          float inputWattDelta = inputWatt - inputWattOld;
          inputWattOld = inputWatt;
          float inputVoltageDelta = Vin_filter - Vin_filter_old;
          Vin_filter_old = Vin_filter;

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

          // Clamp at max/min 1V distance
          if (targetInputVoltage > (Vin_filter+2.))
            targetInputVoltage = Vin_filter+2.;
          else if (targetInputVoltage < (Vin_filter-2.))
            targetInputVoltage = Vin_filter-2.;

          Serial.print("duty = ");
          Serial.print(duty_cycle);
          Serial.print(", dW = ");
          Serial.print(inputWattDelta);
          Serial.print(", dV = ");
          Serial.print(inputVoltageDelta);
          Serial.print(", vol = ");
          Serial.print(Vin_filter);
          Serial.print(", refvol = ");
          Serial.print(targetInputVoltage);
          Serial.print(", I = ");
          Serial.print(Iin_filter);
          Serial.print(", W = ");
          Serial.println(inputWatt);

          if (targetInputVoltage < 12.)
          {
            targetInputVoltage = 13.;
            Serial.println("resetting tracking");
          }
          else if (targetInputVoltage > 40.)
          {
            targetInputVoltage = 39.;
            Serial.println("resetting tracking");
          }
         
        }
  
        if (currentTime > timeStamp_duty + 50)
        {
          timeStamp_duty = currentTime;
          duty_cycle -= targetInputVoltage - Vin_filter;
        }

      }

      //duty_cycle = constrain(duty_cycle, 30, 230);
      if (duty_cycle > 230)
        duty_cycle = 230;
      else if (duty_cycle < 30)
        duty_cycle = 30;
        
      analogWrite(highPin, duty_cycle);
      //analogWrite(lowPin, duty_cycle+d);
    }
  }
}
