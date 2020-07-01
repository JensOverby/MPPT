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
float IoutMax = 1.3;
float lastWatt = 0.;

int Verbose = 0;

char commandBuffer[20];

enum WorkSpcRole {CMD_IDLE, CMD_HELP, CMD_VERBOSE, CMD_DUTY, CMD_CALIBRATE_CURRENT};

float Vin_filter, Vout_filter, Iin_filter;

float median[3];
static int m_i = 0;

static float A = 3.55;
static float B = -2.21;


void sample(float& Iin, float& Iout, float& watt)
{
  int Vin_ = analogRead(VinPin);
  float Vin = 50.*(Vin_/1023.);
  Vin_filter = 0.9*Vin_filter + 0.1*Vin;
  int Vout_ = analogRead(VoutPin);
  float Vout = 50.*(Vout_/1023.);
  Vout_filter = 0.9*Vout_filter + 0.1*Vout;
  int Iin_ = analogRead(IinPin);
  median[m_i] = 30.*(Iin_/1023.);
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
  }
  Iin = A*Iin_filter + B;
  watt = Vin_filter*Iin;
  Iout = watt/Vout_filter;
}

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
  case CMD_CALIBRATE_CURRENT:
    {
      sscanf((char*)(commandBuffer+1), "%f %f %f", &A, &B);
    }
  case CMD_DUTY:
    {
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
        analogWrite(lowPin, duty_cycle+d);
        float Iin, Iout, watt;
        sample(Iin, Iout, watt);
        evalOnState();

        if (Verbose==2)
        {
          counter++;
          if (counter>100)
          {
            counter=0;
            dump(Iin, Iout, watt);
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





void setup() {
  // put your setup code here, to run once:
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
  
  Serial.println("CountDown");
  for (int i=10;i>0;i--)
  {
    Serial.print(i);
    delay(1000);
  }
  Serial.println();
}

// Iout = output current
// Vout = output voltage
void regulate(float Iout, float Vout)
{
  if((Vout>VoutMax) || (Iout>IoutMax))
  {
    if(duty_cycle>30) 
      duty_cycle -=1; 
  }
  else if ((Vout<VoutMax) && (Iout<IoutMax))
  {
    /*if (watt > lastWatt)
    {
      
    }*/

    if(duty_cycle<240) 
      duty_cycle+=1; 
  }
  analogWrite(highPin, duty_cycle);
  analogWrite(lowPin, duty_cycle+d);

  /*if (watt > 0.1)
  {
    analogWrite(lowPin, duty_cycle+d);
    return true;
  }

  analogWrite(lowPin, 255);
  return false;*/
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
  if (Vin_filter > Vout_filter)
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

void loop()
{
  if (getCommand())
  {
    while (execCommand());
  }

  float Iin, Iout, watt;
  sample(Iin, Iout, watt);

  if (Verbose==1)
  {
    dump(Iin, Iout, watt);
  }

  regulate(Iout, Vout_filter);
  evalOnState();

  delay(10);
}
