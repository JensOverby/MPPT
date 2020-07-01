#include "printf.h"

static unsigned int bytesReceived = 0;
static unsigned int bytesExpected = 0;
static unsigned int hashValue = 0;



void myputs(char* str)
{
  Serial.print(str);
}

void initialize_serial()
{
  /* Initialize USB VCP */
  init_printf(myputs);
  commandBuffer[0] = CMD_IDLE;
}

void printHelp()
{
  printf("Commands:\n");
  printf("-----------------\n");
  printf("dccal     DC Calibration\n");
  printf("resind <time constant>  Measure Resistance, Induction, Kp, Ki\n");
  printf("pwmspin <duty>    Run pwm spin\n");
  printf("openspin    Run Open Loop spin\n");
  printf("initenc <duty>    Init Encoder\n");
  printf("foctest <amp>     Run Closed Loop FOC test\n");
  printf("veltest <omega>   Speed test\n");
  printf("postest <pos>     Position test\n");
  printf("gotodeg <deg>     Goto angle (deg)\n");
  printf("dumpsamp    Dump samlings to console\n");
  printf("phasetest   Test phase current readings\n");
  printf("steptest <amp>    Step of 120 deg\n");
  printf("<return>      Back to Idle state\n");
  printf("\n");
}

int getCommand()
{
  // If something arrived at VCP
  uint8_t c;
  while (Serial.available() > 0)
  {
    c = Serial.read();
    if (bytesReceived == 0)
    {
      switch(c)
      {
      case CMD_IDLE:
        bytesExpected = 1;
        break;
      default:
        if (c == '\n')
        {
          if (hashValue != 0)
            printf("hash value = %u\n", hashValue);
          c = CMD_IDLE;
          hashValue = 0;
          bytesExpected = 1;
          break;
          //continue;
        }
        hashValue += c;
        hashValue += (hashValue << 10);
        hashValue ^= (hashValue >> 6);

        switch (hashValue)
        {
        case 18536: // help
          bytesExpected = -1;
          c = CMD_HELP;
          break;
        case 38105: // verbose
          bytesExpected = -1;
          c = CMD_VERBOSE;
          break;
        case 32204: // duty
          bytesExpected = -1;
          c = CMD_DUTY;
          break;
        case 6994: // calcur
          bytesExpected = -1;
          c = CMD_CALIBRATE_CURRENT;
          break;
        default:
          break;
        }

        if (bytesExpected == -1)
          break;

        continue;
      }
    }

    if (hashValue != 0 && c == '\n')
    {
      commandBuffer[bytesReceived] = 0;
      hashValue = 0;
      bytesExpected = 0;
    }
    else
      commandBuffer[bytesReceived] = c;

    bytesReceived++;

    if (bytesReceived >= bytesExpected)
    {
      bytesReceived = 0;
      return 1;
    }
  }

  return 0;
}
