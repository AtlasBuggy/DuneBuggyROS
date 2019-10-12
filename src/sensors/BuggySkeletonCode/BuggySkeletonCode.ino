//#include "update_gps"
//#include "update_imu"

//PINS:
//Note: 2, 3, 18, 19, 20, 21 pins for interrupts
//const int GPS_PIN;
const int RF_PIN;
//const int IMU_PIN;

const int BAUD_RATE = 115200;
const int FREQUENCY = 10; //Hertz (must divide 1000 evenly)

const byte LED = 13;

unsigned long lastTime = 0;

//update_gps myGps;
//update_imu myImu;

void setup() {
  //noInterrupts(); //stop intterupts

  Serial.begin(BAUD_RATE);

  //Initialize pins
  /*
  pinMode(RF_PIN, INPUT);
  digitalWrite(GPS_PIN, LOW);
  digitalWrite(RF_PIN, LOW);
  
  //attachInterrupt(digitalPinToInterrupt(GPS_PIN), ISR_update_GPS, CHANGE);

  //attachInterrupt(digitalPinToInterrupt(RF_PIN), ISR_update_RF, CHANGE);
  */

  //Initialize GPS: GPS uses Serial1 to send info to board
  init_gps();
  //Initialize IMU: IMU code currently prints some outputs to the main Serial
  init_imu();
  


  //Write Timer?
  pinMode (LED, OUTPUT);
  noInterrupts();
  // set up Timer 1
  TCCR1A = 0;          // normal operation
  TCCR1B = bit(WGM12) | bit(CS11) | bit (CS10);   // CTC, scale to clock / 64
  OCR1A =  24999;       // <65536, compare A register value 16000000/(64*10) - 1. 16*10^6/(64*(register-1)) = frq
  TIMSK1 = bit (OCIE1A);             // interrupt on Compare A Match
  //Initialize GPS

  //Initialize RF

  interrupts();

  lastTime = millis();

}

//Interrupts may not be needed with Serial I/O
/*
//When signal occurs on pin, interrupt writes to variable.
void ISR_update_GPS() {
  //detachInterrupt(digitalPinToInterrupt(GPS_PIN));

  //interrupts();

  //while(!Serial.available());


  //attachInterrupt(digitalPinToInterrupt(GPS_PIN),ISR_update_GPS, CHANGE));
}
void ISR_update_RF() {

}
*/

void sendMessage() {  
  //read the data
  
  //GPS data update occurs when the update in the loop adds enough bytes from Serial1 for a complete message from the gps
  
  //gets IMU data, stored in imu_vals[], 4 values
  update_imu();
  
  //Build string
  String msg;
  //Transmit string
  //Serial.print or println?
  Serial.println(msg);
}

//Write Timer?
ISR(TIMER1_COMPA_vect)
{
  static boolean state = false;
  state = !state;  // toggle
  digitalWrite (LED, state ? HIGH : LOW);
}


void loop() {
  
  if (millis() - lastTime >= 1000 / FREQUENCY) {
    sendMessage();
    lastTime = millis();
  }

  check_GPS();
  //noInterrupts();
  //Store 'atomic data' for message sending in this space, where interrupts can't write to the variables being read? Or just use volatile variables where variables are read from memory?

  //interrupts();

  //Just use a delay() node?
  //delay(1000/FREQUENCY);


}
