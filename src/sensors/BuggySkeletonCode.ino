

const int GPS_PIN;
const int RF_PIN;
const int BAUD_RATE = 19200;
const int FREQUENCY = 10; //Hertz (must divide 1000 evenly)

const byte LED = 13;

unsigned long lastTime = 0;

void setup() {
//noInterrupts(); //stop intterupts

Serial.begin(BAUD_RATE);

//Initialize pins
pinMode(GPS_PIN, INPUT);
pinMode(RF_PIN, INPUT);
digitalWrite(GPS_PIN, LOW);
digitalWrite(RF_PIN, LOW);
//
attachInterrupt(digitalPinToInterrupt(GPS_PIN),ISR_update_GPS, CHANGE);

attachInterrupt(digitalPinToInterrupt(RF_PIN),ISR_update_RF, CHANGE);





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


//When signal occurs on pin, interrupt writes to variable. 
void ISR_update_GPS(){
  //detachInterrupt(digitalPinToInterrupt(GPS_PIN));
    
  //interrupts();
  
  //while(!Serial.available());
  
 
  //attachInterrupt(digitalPinToInterrupt(GPS_PIN),ISR_update_GPS, CHANGE));
}
void ISR_update_RF(){
  
}

void sendMessage(){
  //noInterrrupts();
  //read the data
  //Interrupts();
  //Build string
  //Transmit string
}

//Write Timer?
ISR(TIMER1_COMPA_vect)
{
static boolean state = false;
  state = !state;  // toggle
  digitalWrite (LED, state ? HIGH : LOW);
}


void loop() {
  if(millis()-lastTime >= 1000/FREQUENCY){
    sendMessage();
    lastTime = millis();
  }
  
  //noInterrupts();
  //Store 'atomic data' for message sending in this space, where interrupts can't write to the variables being read? Or just use volatile variables where variables are read from memory?
      
  //interrupts();
  
  //Just use a delay() node?
  //delay(1000/FREQUENCY);
  

}
