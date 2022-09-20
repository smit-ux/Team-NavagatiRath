//M1-----------M2
//
//4 - PG5
//5 - PE3
//6 - PH3
//7 - PH4
//11 PB5 - pwm  FOR SERVE
//12 PB6-dir for serving
//-VE VALE FOR SERVING

//             DIRECTION PINS
//   MOTOR 1- PG5 | MOTOR 2- PH5
//            PWM PINS
//   MOTOR 1- PE3 | MOTOR 2- PH4
//    OC3A     |    OC4B
int sig = 1, s2 = 1, s3 = 1;
void motor_setup() {

  DDRG = (1 << PING5);
  DDRE = (1 << PINE3);
  DDRH = (1 << PINH5) | (1 << PINH4);
  DDRB = (1 << PINB5) | (1 << PINB6);

  // ALL DIRECiTION PINS ARE SET HIGH ==> Base Anti-clock
  PORTG |= (1 << PING5);
  PORTH |= (1 << PINH3) | (1 << PINH5);
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  TCCR3A = (1 << COM3A1) | (1 << WGM31);
  TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31); // 1/8 PRESCALER , NON INVERTING MODE
  TCCR4A = (1 << COM4B1) | (1 << WGM41);
  TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS41); // 1/8 PRESCALER , NON INVERTING MODE

  ICR3 = 2000;
  ICR4 = 2000;
  ICR1 = 2000;

}
void motor1_value(int a)
{
  a *= 0.96 * sig;
  if (a >= 0)
  {
    PORTG &= ~(1 << PING5); //DIR FOR MOTOR1.WHEEL ANTICLOCKWISE

  }

  else if (a < 0)
  {
    PORTG |= (1 << PING5); //DIR FOR MOTOR1. WHEEL CLOCKWISE

  }
  OCR3A = constrain(abs(a), 0, 1500); //MOTOR1
}
void motor2_value(int a)
{
  a *= sig;
  if (a >= 0)
  {
    PORTH &= ~(1 << PINH5); //DIR FOR MOTOR2.WHEEL ANTICLOCKWISE

  }

  else if (a < 0)
  {
    PORTH |= (1 << PINH5); //DIR FOR MOTOR2.WHEEL CLOCKWISE

  }
  OCR4B = constrain(abs(a), 0, 1500); //MOTOR2
}
void serving(int a11)
{
  if (a11 >= 0)
  {
    PORTB &= ~(1 << PINB6); //DIR FOR MOTOR3.WHEEL ANTICLOCKWISE
  }

  else if (a11 < 0)
  {
    PORTB |= (1 << PINB6); //DIR FOR MOTOR3.WHEEL CLOCKWISE

  }
  OCR1A = constrain(abs(a11), 0, 1000); //MOTOR1
}
void up() {
  
  motor1_value(1400);//forward
  motor2_value(-1400);
}
void down() {
  motor1_value(-1000);//backward
  motor2_value(1000);
}
void left() {
  motor1_value(-400);//left
  motor2_value(-400);
}
void right() {
  motor1_value(400);
  motor2_value(400);//right
}

//void up_left() {
//  motor1_value(800);//forward
//  motor2_value(-1000);
//}
//
//void up_right() {
//  motor1_value(1000);//forward
//  motor2_value(-800);
//}
//void down_left() {
//  motor1_value(-1000);//backward
//  motor2_value(800);
//}
//void down_right() {
//  motor1_value(-800);//backward
//  motor2_value(1000);
//}

void hold() {
  motor1_value(0);
  motor2_value(0);
}

void rotate() {
  motor1_value(-500);//left rotate
  motor2_value(-500);
}
