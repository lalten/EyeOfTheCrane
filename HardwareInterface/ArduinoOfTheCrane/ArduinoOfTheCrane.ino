const int PIN_KORB_LINKS = 7;
const int PIN_KORB_RECHTS = 8;
const int PIN_PWM_0 = 5; // hub auf (250)/ab(lower)
const int PIN_PWM_1 = 6; // nicken 250=hoch
const int PIN_PWM_2 = 9; // teleskop 100=rein, 250=raus
const int PIN_PWM_3 = 10; // higher=left turn

const int NEUTRAL = 199;

// 0pwm   = 24V
// 200pwm = 5.17V
// 255pwm = 0V

void setup() {
  pinMode(PIN_KORB_LINKS, OUTPUT);
  pinMode(PIN_KORB_RECHTS, OUTPUT);
  pinMode(PIN_PWM_0, OUTPUT);
  pinMode(PIN_PWM_1, OUTPUT);
  pinMode(PIN_PWM_2, OUTPUT);
  pinMode(PIN_PWM_3, OUTPUT);
  Serial.begin(115200);
  setOutput(NEUTRAL,NEUTRAL,NEUTRAL,NEUTRAL,0);
}

char buf[100] = {0};
int buf_cnt = 0;
long last_reception_micros = 0;

void loop() {
  long now = micros();
  while(Serial.available())
  {
    if(buf_cnt >= 100)
    {
      buf_cnt=0;
    }
    char c = Serial.read();
    buf[buf_cnt++] = c;
    if(c == '\n')
    {
      buf[buf_cnt] = '\0';
      buf_cnt=0;
      interpreteCommand();
    }
  }
  if(now - last_reception_micros > 1000)
  {
      buf_cnt=0;
  }
}

void interpreteCommand()
{
  int pwm0, pwm1, pwm2, pwm3, korb;
  sscanf(buf, "%d,%d,%d,%d,%d", &pwm0, &pwm1, &pwm2, &pwm3, &korb);

  setOutput(pwm0, pwm1, pwm2, pwm3, korb);
  
  Serial.print("Got");
  Serial.print(" PWM0=");
  Serial.print(pwm0, DEC);
  Serial.print(", PWM1=");
  Serial.print(pwm1, DEC);
  Serial.print(", PWM2=");
  Serial.print(pwm2, DEC);
  Serial.print(", PWM3=");
  Serial.print(pwm3, DEC);
  Serial.print(", Korb=");
  Serial.print(korb, DEC);
  Serial.println();
}

void setOutput(int pwm0, int pwm1, int pwm2, int pwm3, int korb)
{
  if(korb < 0)
  {
    digitalWrite(PIN_KORB_LINKS, LOW);
    digitalWrite(PIN_KORB_RECHTS, HIGH);
  }
  else if(korb > 0)
  {
    digitalWrite(PIN_KORB_LINKS, HIGH);
    digitalWrite(PIN_KORB_RECHTS, LOW);
  }
  else
  {
    digitalWrite(PIN_KORB_LINKS, HIGH);
    digitalWrite(PIN_KORB_RECHTS, HIGH);
  }

  //-100..100 --> 0V..10V --> 100%pwm..58.3%pwm
  analogWrite(PIN_PWM_0, pwm0); //map(pwm0, -100,100,  255,149));
  analogWrite(PIN_PWM_1, pwm1); //map(pwm1, -100,100,  255,149));
  analogWrite(PIN_PWM_2, pwm2); //map(pwm2, -100,100,  255,149));
  analogWrite(PIN_PWM_3, pwm3); //map(pwm3, -100,100,  255,149));
  
}

