#include "SoftwareSerial.h"
#include "SkyTraqNmeaParser.h"

#define _TIME_DEBUGGER_   0

D64 gps_vals[3];
int safe_gps = 0;

#if (_TIME_DEBUGGER_)
U16 hour;
U16 minute;
D64 second;
#endif

SkyTraqNmeaParser parser;
const GnssData* gdata;
U32 gnssUpdateFlag = 0;

bool GnssUpdated(U32 f, const char* buf, SkyTraqNmeaParser::ParsingType type);
bool update_globals(U32 f, const char* buf, SkyTraqNmeaParser::ParsingType type);

bool isSafe_GPS() {
  return safe_gps == 3;
}

bool init_gps() {
  //Set callback function for parsing result notification
  parser.SetNotify(GnssUpdated);
  
  //For UART interrupt
  pinMode(2, INPUT);
  digitalWrite(2, LOW);
  
  //NS-HP output NMEA message in 115200 bps
  Serial2.begin(_BAUD_RATE_);
  //attachInterrupt(0, serialInterrupt_GPS, CHANGE);

  gps_vals[2] = -999;
  
  return true;
}

void serialEvent1(){
  parser.Encode(Serial2.read());
}

bool GnssUpdated(U32 f, const char* buf, SkyTraqNmeaParser::ParsingType type){
  gnssUpdateFlag |= f;
  gdata = parser.GetGnssData();
  safe_gps = 0;
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
    case SkyTraqNmeaParser::UpdateTime:
      gps_vals[0] = gnss.GetLatitude();
      gps_vals[1] = gnss.GetLongitude();
      gps_vals[2] = gnss.GetAltitudeInMeter();
      safe_gps = 3;

#if (_TIME_DEBUGGER_)
      hour = gnss.GetHour();
      minute = gnss.GetMinute();
      second = gnss.GetSecond();
#endif
      break;
// These only update when the numbers change from the GPS.
/*
    case SkyTraqNmeaParser::UpdateLatitude:
      gps_vals[0] = gnss.GetLatitude();
      safe_gps++;
      break;
    case SkyTraqNmeaParser::UpdateLongitude:
      gps_vals[1] = gnss.GetLongitude();
      safe_gps++;
      break;
    case SkyTraqNmeaParser::UpdateAltitude:
      gps_vals[2] = gnss.GetAltitudeInMeter();
      safe_gps++;
      break;
*/
    default:
      break;
    }
  }
  return true;
}

void write_gps_vals() {
  for(int i = 0; i < 3; i++) {
    Serial.print(gps_vals[i],4);
    Serial.print("\t");
  }
#if (_TIME_DEBUGGER_)
  Serial.print(hour);
  Serial.print(":");
  Serial.print(minute);
  Serial.print(":");
  Serial.print(second);
  Serial.print("\t");
#endif
}
