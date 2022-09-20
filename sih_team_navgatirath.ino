//     ^
//M1-----------M2
//(-)         (+)
//             DIRECTION PINS
//   MOTOR 1- PG5 | MOTOR 2- PH3
//            PWM PINS
//   MOTOR 1- PE3 | MOTOR 2- PH4
//    OC3A     |    OC4B
#include<avr/io.h>
#include<avr/delay.h>
#include<avr/interrupt.h>
#include "motor.h"
#define timeout 8000

uint32_t prev_millis = 0;
bool g = 0;
int distance1, distance2, distance3, distance4;
int us1 = 0, us2 = 0, us3 = 0, us4 = 0, d1, d2, d3, d4;
long duration;
int distance = 200;
uint8_t t_dtype, t_data, i1 = 0, checksum1, check1, dtype, u1_th = 20, u2_th = 20, u3_th = 20, u4_th = 20;
int16_t data, angle, dist;
int main() {
  init();
  motor_setup();
  UCSR0B = 0x98;
  UBRR0 = 103;
  DDRF = 0xFF;
  DDRK = 0XC0;
  DDRA = 0xFF;
  sei();
  //  motor1_value(1000);
  //  motor2_value(1000);
  while (1)
  {
    ////////////////////////////////////////////////////////
    PORTF = 0x01;
    _delay_us(10);
    PORTF = 0x00;
    duration = pulseIn(A8, HIGH, timeout);
    distance1 = duration * 0.034 / 2;
    PORTF = 0x02;
    _delay_us(10);
    PORTF = 0x00;
    duration = pulseIn(A9, HIGH, timeout);
    distance2 = duration * 0.034 / 2;
    PORTF = 0x04;
    _delay_us(10);
    PORTF = 0x00;
    duration = pulseIn(A10, HIGH, timeout);
    distance3 = duration * 0.034 / 2;
    PORTF = 0x08;
    _delay_us(10);
    PORTF = 0x00;
    duration = pulseIn(A11, HIGH, timeout);
    distance4 = duration * 0.034 / 2;
    ////////////////////////////////////////////////////////////

    //    if ((distance1 < u1_th) && distance1)d1 = 1;
    if ((distance2 < u2_th) && distance2) {
      d3++;
      if (d3 > 3) {
        d2 = 1;
        PORTK |= 0x80;
        sendpi();
      }
    }
    else {
      PORTK &= ~0x80;
      d2 = 0;
      d3 = 0;
    }
    
    
    if (dtype == 0x15) {
      if (data & 0x01) {
        up();
      }
      else if (data & 0x02) {
        down();
      }
      else if (data & 0x04) {
        left();
      }
      else if (data & 0x08) {
        right();
      }
    }
    else if (dtype == 0x10) {
      u1_th = data;
    }
    else if (dtype == 0x20) {
      u2_th = data;
    }
    else if (dtype == 0x30) {
      u3_th = data;
      
    }
    else if (dtype == 0x05) {
      if (g == 0) {
        hold();
        serving(-800);
        PORTK|=0x40;
        prev_millis = millis();
        g = 1;
      }
    }

    if (millis() - prev_millis > 4000 && g == 1) {
      serving(0);
      PORTK&=~0x40;
      g = 0;
    }
  }
}

ISR(USART0_RX_vect) {
  uint8_t temp = UDR0;
  if (i1 < 2) {

    if (temp == 0x01) {
      i1++;
    }
    else {
      i1 = 0;
    }
  }
  else {
    if (i1 == 2) {
      t_dtype = temp;
      checksum1 += t_dtype;
      i1++;
    }
    else if (i1 == 3)
    {
      t_data = temp;
      checksum1 += t_data;
      i1++;
    }
    else {

      check1 = temp;

      if ((checksum1 & 0xFF) == check1) {
        dtype = t_dtype;
        data = t_data;
        checksum1 = 0;
      }
      i1 = 0;
    }
  }
}

void sendpi() {
  PORTA = d1 | (d2 << 1) | (d3 << 2) | (d4 << 3);
  if ((UCSR0A & (1 << UDRE0))) {
    UDR0 = d1 | (d2 << 1) | (d3 << 2) | (d4 << 3);
  }
  //    digitalWrite(A15,1);}
  d1 = 0;
  d2 = 0;
  d3 = 0;
  d4 = 0;
}


//        if (data & 0x04) up_left();
//        else if (data & 0x08) up_right();
//        if (data & 0x04) down_left();
//        else if (data & 0x08) down_right();
