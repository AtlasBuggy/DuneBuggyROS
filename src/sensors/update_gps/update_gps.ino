#include <SoftwareSerial.h>
#include "SkyTraqNmeaParser.h"

double gps_vals[3];
int safe = 0;

SkyTraqNmeaParser parser;
const GnssData* gdata;
U32 gnssUpdateFlag = 0;

bool GnssUpdated(U32 f, const char* buf, SkyTraqNmeaParser::ParsingType type);
bool update_globals(U32 f, const char* buf, SkyTraqNmeaParser::ParsingType type);

bool isSafe() {
  return safe == 3;
}

bool init_gps() {
  //Set callback function for parsing result notification
  parser.SetNotify(GnssUpdated);
  
  //For UART interrupt
  pinMode(2, INPUT);
  digitalWrite(2, LOW);
  
  //NS-HP output NMEA message in 115200 bps
  Serial.begin(115200);
  attachInterrupt(0, serialInterrupt, CHANGE);
  return true;
}

volatile boolean inService = false;
void serialInterrupt() {
  if(inService) return;
  inService = true;
  
  interrupts();
  while(!Serial.available());
  parser.Encode(Serial.read());
  
  inService = false;
}

bool GnssUpdated(U32 f, const char* buf, SkyTraqNmeaParser::ParsingType type){
  gnssUpdateFlag = gnssUpdateFlag | f;
  gdata = parser.GetGnssData();

  safe = 0;
  update_globals(f, buf, type);
  //return true to clear the flag in SkyTraqNmeaParseryTraq
  return true;
}

bool update_globals(U32 f, const char* buf, SkyTraqNmeaParser::ParsingType type) {
  U32 i = 0;
  const GnssData& gnss = *gdata;

  for(; i < 32; ++i)
  {
    U32 mask = (1 << i);
    switch((mask & f))
    {
    case SkyTraqNmeaParser::UpdateLatitude:
        gps_vals[0] = gnss.GetLatitude();
        safe++;
      break;
    case SkyTraqNmeaParser::UpdateLongitude:
        gps_vals[1] = gnss.GetLongitude();
        safe++;
       break;
    case SkyTraqNmeaParser::UpdateAltitude:
        gps_vals[2] = gnss.GetAltitudeInMeter();
        safe++;
      break;
    default:
      break;
    }
  }
  return true;
}



void setup() {
  init_gps();  
}

void loop() {}
