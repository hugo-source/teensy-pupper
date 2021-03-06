//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Version: Beta 1.1

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6;
unsigned long rising_edge_start_7, rising_edge_start_8, rising_edge_start_9;
unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
unsigned long channel_7_raw, channel_8_raw, channel_9_raw;
int ppm_counter = 0;
unsigned long time_ms = 0;

/*
void readPPM_setup(int pin) {
  // DESCRIPTION: Initialize software interrupts on radio channel pin for PPM type receiver
  // Declare interrupt pins
  pinMode(pin, INPUT_PULLUP);
  delay(20);
  //Attach interrupt and point to corresponding ISR function
  attachInterrupt(digitalPinToInterrupt(pin), getPPM, CHANGE);
}
*/

//void readPWM_setup(int ch1, int ch2, int ch3, int ch4, int ch5, int ch6, int ch7, int ch8, int ch9) {
//void readPWM_setup(int ch1, int ch2, int ch3, int ch4, int ch5, int ch6, int ch7, int ch8) {
void readPWM_setup(int ch1, int ch2, int ch3, int ch4, int ch5, int ch6, int ch7) {
  //DESCRIPTION: Initialize software interrupts on radio channel pins for PWM tpye receiver
  //Declare interrupt pins 
  pinMode(ch1, INPUT_PULLUP);
  pinMode(ch2, INPUT_PULLUP);
  pinMode(ch3, INPUT_PULLUP);
  pinMode(ch4, INPUT_PULLUP);
  pinMode(ch5, INPUT_PULLUP);
  pinMode(ch6, INPUT_PULLUP);
  pinMode(ch7, INPUT_PULLUP);
//  pinMode(ch8, INPUT_PULLUP);
//  pinMode(ch9, INPUT_PULLUP);
  delay(20);
  //Attach interrupt and point to corresponding ISR functions
  attachInterrupt(digitalPinToInterrupt(ch1), getCh1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch2), getCh2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch3), getCh3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch4), getCh4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch5), getCh5, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch6), getCh6, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch7), getCh7, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(ch8), getCh8, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(ch9), getCh9, CHANGE);
  delay(20);
}

unsigned long getRadioPWM(int ch_num) {
  //DESCRIPTION: Get current radio commands from interrupt routines 
  unsigned long returnPWM = 0;
  
  if (ch_num == 1) {
    returnPWM = channel_1_raw;
  }
  else if (ch_num == 2) {
    returnPWM = channel_2_raw;
  }
  else if (ch_num == 3) {
    returnPWM = channel_3_raw;
  }
  else if (ch_num == 4) {
    returnPWM = channel_4_raw;
  }
  else if (ch_num == 5) {
    returnPWM = channel_5_raw;
  }
  else if (ch_num == 6) {
    returnPWM = channel_6_raw;
  }
  else if (ch_num == 7) {
    returnPWM = channel_7_raw;
  }
  else if (ch_num == 8) {
    returnPWM = channel_8_raw;
  }
  else if (ch_num == 9) {
    returnPWM = channel_9_raw;
  };
  
  return returnPWM;
}




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




//INTERRUPT SERVICE ROUTINES 
/*
void getPPM() {
  unsigned long dt_ppm;
  int trig = digitalRead(PPM_Pin);
  
  if (trig==1) { //only care about rising edge
    dt_ppm = micros() - time_ms;
    time_ms = micros();

    
    if (dt_ppm > 5000) { //waiting for long pulse to indicate a new pulse train has arrived
      ppm_counter = 0;
    }
  
    if (ppm_counter == 1) { //first pulse
      channel_1_raw = dt_ppm;
    }
  
    if (ppm_counter == 2) { //second pulse
      channel_2_raw = dt_ppm;
    }
  
    if (ppm_counter == 3) { //third pulse
      channel_3_raw = dt_ppm;
    }
  
    if (ppm_counter == 4) { //fourth pulse
      channel_4_raw = dt_ppm;
    }
  
    if (ppm_counter == 5) { //fifth pulse
      channel_5_raw = dt_ppm;
    }
  
    if (ppm_counter == 6) { //sixth pulse
      channel_6_raw = dt_ppm;
    }
    
    ppm_counter = ppm_counter + 1;
  }
}
*/
void getCh1() {
  int trigger = digitalRead(ch1Pin);
  if(trigger == 1) {
    rising_edge_start_1 = micros();
  }
  else if(trigger == 0) {
    channel_1_raw = micros() - rising_edge_start_1;
  }
}

void getCh2() {
  int trigger = digitalRead(ch2Pin);
  if(trigger == 1) {
    rising_edge_start_2 = micros();
  }
  else if(trigger == 0) {
    channel_2_raw = micros() - rising_edge_start_2;
  }
}

void getCh3() {
  int trigger = digitalRead(ch3Pin);
  if(trigger == 1) {
    rising_edge_start_3 = micros();
  }
  else if(trigger == 0) {
    channel_3_raw = micros() - rising_edge_start_3;
  }
}

void getCh4() {
  int trigger = digitalRead(ch4Pin);
  if(trigger == 1) {
    rising_edge_start_4 = micros();
  }
  else if(trigger == 0) {
    channel_4_raw = micros() - rising_edge_start_4;
  }
}

void getCh5() {
  int trigger = digitalRead(ch5Pin);
  if(trigger == 1) {
    rising_edge_start_5 = micros();
  }
  else if(trigger == 0) {
    channel_5_raw = micros() - rising_edge_start_5;
  }
}

void getCh6() {
  int trigger = digitalRead(ch6Pin);
  if(trigger == 1) {
    rising_edge_start_6 = micros();
  }
  else if(trigger == 0) {
    channel_6_raw = micros() - rising_edge_start_6;
  }
}

void getCh7() {
  int trigger = digitalRead(ch7Pin);
  if(trigger == 1) {
    rising_edge_start_7 = micros();
  }
  else if(trigger == 0) {
    channel_7_raw = micros() - rising_edge_start_7;
  }
}
/*
void getCh8() {
  int trigger = digitalRead(ch8Pin);
  if(trigger == 1) {
    rising_edge_start_8 = micros();
  }
  else if(trigger == 0) {
    channel_8_raw = micros() - rising_edge_start_8;
  }
}

void getCh9() {
  int trigger = digitalRead(ch9Pin);
  if(trigger == 1) {
    rising_edge_start_9 = micros();
  }
  else if(trigger == 0) {
    channel_9_raw = micros() - rising_edge_start_9;
  }
}
*/
