/* Sensorless brushless DC (BLDC) motor control with Arduino UNO (Arduino DIY ESC).
 * This is a free software with NO WARRANTY.
 * https://simple-circuit.com/
 */
 
#define PWM_START_DUTY    55

#define A_SD 11
#define B_SD 10
#define C_SD 9
#define A_IN 12
#define B_IN 4
#define C_IN 2

byte bldc_step = 0, motor_speed;
unsigned int i;

volatile bool toggle = false;
#define DEBUG_PIN 13

void setup()
{
  pinMode(A_IN, OUTPUT);
  pinMode(B_IN, OUTPUT);
  pinMode(C_IN, OUTPUT);

  pinMode(A_SD, OUTPUT);
  pinMode(B_SD, OUTPUT);
  pinMode(C_SD, OUTPUT);

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
}

// Analog comparator ISR
ISR (ANALOG_COMP_vect)
{
  digitalWrite(DEBUG_PIN, toggle? HIGH : LOW);
  toggle = !toggle;

  bldc_move();
  bldc_step++;
  bldc_step %= 6;

  // BEMF debounce
  for(i = 0; i < 100; i++) {
    if(bldc_step & 1){
      if(!(ACSR & 0x20)) i -= 1;
    }
    else {
      if((ACSR & 0x20))  i -= 1;
    }
  }
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
 
void loop()
{
  //SET_PWM_DUTY(PWM_START_DUTY);    // Setup starting PWM with duty cycle = PWM_START_DUTY
  unsigned int j = 50;
  // Motor start
  while(j > 5) {
    delay(j);
    bldc_move();
    bldc_step++;
    bldc_step %= 6;
    j = j - 2;
  }

  motor_speed = 255;
  ACSR |= 0x08;                    // Enable analog comparator interrupt
  while(1)
  {
    delay(1000);
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
