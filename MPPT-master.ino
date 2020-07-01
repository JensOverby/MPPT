#include <EEPROM.h>

const int IinPin = 14;
const int VinPin = 15;
const int VoutPin = 16;
const int highPin = 10;
const int lowPin = 9;
const int disableDriverPin = 12;

const int d = 2;
int duty_cycle = 230;
int last_duty_cycle = 0;

float VoutMax = 9.7;
float IoutMax = 3.;

float watt_old = 0.;
float Vout_filter_old = 0.;

int Verbose = 0;

char commandBuffer[20];

enum WorkSpcRole {CMD_IDLE, CMD_REGULATE, CMD_HELP, CMD_VERBOSE, CMD_DUTY, CMD_CALIBRATE_CURRENT, CMD_STORE_EEPROM};

float Vin_filter, Vout_filter, Iin_filter;


struct StoreStruct
{
  float A;
  float B;
  bool supplyIsOutputVoltage;
};

StoreStruct store;

void setup() {
  // put your setup code here, to run once:
  EEPROM.get(0, store);
  store.A = 3.67;
  store.B = -2.11;
  store.supplyIsOutputVoltage = false;
  pinMode(highPin, OUTPUT);
  pinMode(lowPin, OUTPUT);
  pinMode(disableDriverPin, OUTPUT);
  TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  TCCR1A = (TCCR1A & 0x0F) | 0xC0 ; // set pin 9 inverted
  
  digitalWrite(disableDriverPin, true);
  analogWrite(highPin, 0);
  analogWrite(lowPin, 0);

  /*pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS21);
  OCR2A = 50;
  OCR2B = 49;*/

  Serial.begin(9600);

  initialize_serial();
  
  /*Serial.println("CountDown");
  for (int i=10;i>0;i--)
  {
    Serial.print(i);
    delay(1000);
  }
  Serial.println();*/
}

bool sample(float& Iin, float& Iout, float& watt)
{
  int Vin_ = analogRead(VinPin);
  float Vin = 50.*(Vin_/1023.);
  Vin_filter = 0.9*Vin_filter + 0.1*Vin;
  int Vout_ = analogRead(VoutPin);
  float Vout = 50.*(Vout_/1023.);
  Vout_filter = 0.9*Vout_filter + 0.1*Vout;

  int Iin_ = analogRead(IinPin);
  Iin = 30.*(Iin_/1023.);
  bool filtered = medianFilter(Iin);
  if (filtered)
    Iin_filter = 0.95*Iin_filter + 0.05*Iin;
  Iin = store.A*Iin_filter + store.B;
  
  watt = Vin_filter*Iin;
  Iout = watt/Vout_filter;

  return filtered;
}

int verbose_counter = 0;
int regulate = 0;

bool execCommand() //struct axis_state_s* axis)
{
  switch (commandBuffer[0])
  {
  case CMD_IDLE:
    break;
  case CMD_REGULATE:
    {
      sscanf((char*)(commandBuffer+1), "%d", &regulate);
    }
    break;
  case CMD_VERBOSE:
    {
      sscanf((char*)(commandBuffer+1), "%d", &Verbose);
    }
    commandBuffer[0] = CMD_IDLE;
    break;
  case CMD_CALIBRATE_CURRENT:
    {
      sscanf((char*)(commandBuffer+1), "%f %f %f", &store.A, &store.B);
    }
  case CMD_STORE_EEPROM:
    {
      EEPROM.put(0, store);
    }
    commandBuffer[0] = CMD_IDLE;
    break;
  case CMD_DUTY:
    {
      int duty;
      sscanf((char*)(commandBuffer+1), "%d", &duty);
      while (!getCommand())
      {
        if (duty>duty_cycle)
          duty_cycle++;
        else if (duty<duty_cycle)
          duty_cycle--;

        analogWrite(highPin, duty_cycle);
        analogWrite(lowPin, duty_cycle+d);
        float Iin, Iout, watt;
        sample(Iin, Iout, watt);
        evalOnState();

        if (Verbose==2)
        {
          verbose_counter++;
          if (verbose_counter>100)
          {
            verbose_counter=0;
            dump(Iin, Iout, watt);
          }
        }
        
        //delay(1);
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

void dump(float& Iin, float& Iout, float& watt)
{
  Serial.print("duty=");
  Serial.print(duty_cycle);
  Serial.print(", Vin=");
  Serial.print(Vin_filter);
  Serial.print("V, Iin=");
  Serial.print(Iin);
  Serial.print("A (");
  Serial.print(Iin_filter);
  Serial.print("), Vout=");
  Serial.print(Vout_filter);
  Serial.print("V, Iout=");
  Serial.print(Iout);
  Serial.print("A, watt=");
  Serial.print(watt);
  Serial.println("W");
}

void evalOnState()
{
  float* supply;
  if (store.supplyIsOutputVoltage)
    supply = &Vout_filter;
  else
    supply = &Vin_filter;

  if (*supply > 9. && Vin_filter > Vout_filter)
  {
    digitalWrite(disableDriverPin, false);
    if (Verbose==1)
      Serial.println("Gate Driver Enabled");
  }
  else
  {
    digitalWrite(disableDriverPin, true);
    if (Verbose==1)
      Serial.println("Gate Driver Disabled");
  }
}

void regulate_duty(float Iout, float Vout)
{
  if((Vout>VoutMax) || (Iout>IoutMax))
  {
    if(duty_cycle>30) 
      duty_cycle -=1; 
  }
  else if ((Vout<VoutMax) && (Iout<IoutMax))
  {
    if(duty_cycle<240) 
      duty_cycle+=1; 
  }
  analogWrite(highPin, duty_cycle);
  analogWrite(lowPin, duty_cycle+d);
}

void loop()
{
  if (getCommand())
  {
    while (execCommand());
  }

  if (regulate)
  {
    evalOnState();
    float Iin, Iout, watt;
    if (sample(Iin, Iout, watt))
    {
      float walk = (watt - watt_old)/watt;
      float dir = (Vout_filter - Vout_filter_old) * walk;
      watt_old = watt;
      Vout_filter_old = Vout_filter;
    
      VoutMax += 0.01 * dir;
    
      regulate_duty(Iout, Vout_filter);
  
      if (Verbose==3)
      {
        verbose_counter++;
        if (verbose_counter>20)
        {
          verbose_counter=0;
          Serial.print("VoutMax=");
          Serial.print(VoutMax);
          Serial.print(", walk=");
          Serial.print(walk*1000.);
          Serial.print(", dir=");
          Serial.println(dir*1000.);
          dump(Iin, Iout, watt);
        }
      }
    }
  }

  delay(10);
}
