
#include <SoftwareSerial.h>
#include "SkyTraqNmeaParser.h"
#include "NMEA0183.h"

NMEA0183 nmea;
// The SkyTraqNmeaParser object
SkyTraqNmeaParser parser;
// The SkyTraqNmeaParser result
const GnssData* gdata;
// Notification of SkyTraqNmeaParser
U32 gnssUpdateFlag = 0;

//Callback function for SkyTraqNmeaParser notification
bool GnssUpdated(U32 f, const char* buf, SkyTraqNmeaParser::ParsingType type);
// Display parsing result
bool Display(U32 f, const char* buf, SkyTraqNmeaParser::ParsingType type);
// Show satellites information(from GSV token, and need turn on _SUPPORT_GPS_SATELLITES_, 
// _SUPPORT_GLONASS_SATELLITES_ or _SUPPORT_BEIDOU_SATELLITES_)
void ShowSatellites(const SatelliteInfo* si);

void setup()
{
  //Set callback function for parsing result notification
  parser.SetNotify(GnssUpdated);
  
  //For UART interrupt
  pinMode(2, INPUT);
  digitalWrite(2, LOW);
  
  //NS-HP output NMEA message in 115200 bps
  Serial1.begin(115200);
  Serial.begin(115200);
  //attachInterrupt(digitalPinToInterrupt(2), serialInterrupt, CHANGE);
  Serial.print("SUPPORTS SATELLITES: ");
  Serial.println(_SUPPORT_GPS_SATELLITES_);
  Serial.println("Init done!=================================");
}

//Set the flag to true to prevent re-entry of interrupt.
volatile boolean inService = false;

void serialEvent1(){
  //Serial.print("Available");
  int dat = Serial1.peek();
  //Serial.print(": Data: ");
  //Serial.println((char)dat);
  SkyTraqNmeaParser::ParsingType typ3 = parser.Encode(Serial1.read());
  /*
  if(typ3 != parser.None && typ3 != parser.BufferOverflow)Serial.println("Valid Data");
  else if(typ3==parser.None)Serial.println("none");
  
  if (nmea.update((char)dat))
    {
      /*
      Serial.print("NMEA0183 sentence accepted (");
      Serial.print(nmea.getFields());
      Serial.print(" fields): ");
      Serial.write(nmea.getSentence());
      Serial.println();
      
    }
    */
}

/*
void serialInterrupt()
{
 Serial.println("info");
 if (inService) 
 {
   return;
 }
 inService = true;

 interrupts();
 while(!Serial1.available());
 
  Serial.print("Available");
  int dat = Serial1.peek();
  Serial.print(": Data: ");
  Serial.println((char)dat);
  SkyTraqNmeaParser::ParsingType typ3 = parser.Encode(Serial1.read());
  if(typ3 != parser.None && typ3 != parser.BufferOverflow)Serial.println("Valid Data");
  else if(typ3==parser.None)Serial.println("none");
   /*
   if (nmea.update((char)dat))
    {
      Serial.print("NMEA0183 sentence accepted (");
      Serial.print(nmea.getFields());
      Serial.print(" fields): ");
      Serial.write(nmea.getSentence());
      Serial.println();
    }
  
 inService = false;
}
*/
void loop() {
}

void ShowSatellites(const SatelliteInfo* si)
{
  bool hasTitle = false;
  int max = GnssData::GetMaxSatelliteNum();
  for(int i = 0; i < max; ++i)
  {
    if(si[i].sv == 0)
      break;
    if(!hasTitle)
    {
      Serial.print("PRN \tAZ  \tEL  \tCN0  \tUsed\r\n");
      hasTitle = true;
    }
    Serial.print(si[i].sv);
    Serial.print('\t');
    Serial.print(si[i].azimuth);
    Serial.print('\t');
    Serial.print(si[i].elevation);
    Serial.print('\t');
    Serial.print(si[i].cno);
    Serial.print('\t');
    Serial.println((si[i].isInUse) ? 'Y' : 'N');
  }
}

bool GnssUpdated(U32 f, const char* buf, SkyTraqNmeaParser::ParsingType type)
{
  //Serial.println("updated");
  gnssUpdateFlag |= f;
  gdata = parser.GetGnssData();
  Display(f, buf, type);
  //return true to clear the flag in SkyTraqNmeaParseryTraq
  return true;
}

bool Display(U32 f, const char* buf, SkyTraqNmeaParser::ParsingType type)
{
  U32 i = 0;
  const GnssData& gnss = *gdata;

  for(; i < 32; ++i)
  {
    U32 mask = (1 << i);
    switch((mask & f))
    {
    case SkyTraqNmeaParser::NoUpdate:
      //Do nothing
      //Serial.println("No UPdate");
      break;
    case SkyTraqNmeaParser::UpdateDate:
      Serial.print("Date:");
      Serial.print(gnss.GetYear());
      Serial.print('/');
      Serial.print(gnss.GetMonth());
      Serial.print('/');
      Serial.println(gnss.GetDay());
      break;
    case SkyTraqNmeaParser::UpdateTime:
      Serial.print("Time:");
      Serial.print(gnss.GetHour());
      Serial.print(':');
      Serial.print(gnss.GetMinute());
      Serial.print(':');
      Serial.println(gnss.GetSecond());
      
      Serial.println();
      Serial.print(gnss.GetLatitude());
      Serial.print(" ");
      Serial.print(gnss.GetLongitude());
      Serial.print(" ");
      Serial.print(gnss.GetAltitudeInMeter());
      Serial.println();
      break;
    case SkyTraqNmeaParser::UpdateLatitude:
      Serial.print("Latitude:");
      Serial.println(gnss.GetLatitude());
      break;
    case SkyTraqNmeaParser::UpdateLongitude:
      Serial.print("Longitude:");
      Serial.println(gnss.GetLongitude());
      break;
    case SkyTraqNmeaParser::UpdateAltitude:
      Serial.print("Altitude:");
      Serial.println(gnss.GetAltitudeInMeter());
      break;
    case SkyTraqNmeaParser::UpdateCourse:
     Serial.print("Course:");
      Serial.println(gnss.GetCourseInDegree());
      break;
    case SkyTraqNmeaParser::UpdateSpeed:
      Serial.print("Speed:");
      Serial.println(gnss.GetSpeedInKmHr());
      break;
    case SkyTraqNmeaParser::UpdateQualitMode:
      Serial.print("Qualit Mode:");
      Serial.println(gnss.GetQualitMode());
      break;
    case SkyTraqNmeaParser::UpdateNumberOfSv:
      Serial.print("Number Of Sv:");
      Serial.println(gnss.GetNumberOfSv());
      break;
    case SkyTraqNmeaParser::UpdateHdop:
      Serial.print("HDOP:");
      Serial.println(gnss.GetHdop());
      break;
    case SkyTraqNmeaParser::UpdatePdop:
      Serial.print("PDOP:");
      Serial.println(gnss.GetPdop());
      break;
    case SkyTraqNmeaParser::UpdateVdop:
      Serial.print("VDOP:");
      Serial.println(gnss.GetVdop());
      break;
    case SkyTraqNmeaParser::UpdateNavigationMode:
      Serial.print("Navigation Mode:");
      Serial.println(gnss.GetNavigationMode());
      break;
    case SkyTraqNmeaParser::UpdateSatelliteInfo:
#if (_SUPPORT_GPS_SATELLITES_)
      ShowSatellites(gnss.GetGpsSatellites());
#endif
#if (_SUPPORT_GLONASS_SATELLITES_)
      ShowSatellites(gnss.GetGlonassSatellites());
#endif
#if (_SUPPORT_BEIDOU_SATELLITES_)
      ShowSatellites(gnss.GetBeidouSatellites());
#endif
      break;
    case SkyTraqNmeaParser::UpdateEnuVelocity:
      Serial.print("E-Velocity:");
      Serial.print(gnss.GetEVelocity());
      Serial.print("   N-Velocityy:");
      Serial.print(gnss.GetNVelocity());
      Serial.print("   U-Velocity:");
      Serial.println(gnss.GetUVelocity());
     break;
    case SkyTraqNmeaParser::UpdateRtkAge:
      Serial.print("RTK Age:");
      Serial.println(gnss.GetRtkAge());
      break;
    case SkyTraqNmeaParser::UpdateRtkRatio:
      Serial.print("RTK Ratio:");
      Serial.println(gnss.GetRtkRatio());
     break;
    case SkyTraqNmeaParser::UpdateEnuProjection:
      Serial.print("E-Projection:");
      Serial.print(gnss.GetEProjection());
      Serial.print("   N-Projection:");
      Serial.print(gnss.GetNProjection());
      Serial.print("   U-Projection:");
      Serial.println(gnss.GetUProjection());      
      break;
    case SkyTraqNmeaParser::UpdateBaselineLength:
       Serial.print("RTK Baseline Length:");
       Serial.println(gnss.GetBaselineLength());
       break;
    case SkyTraqNmeaParser::UpdateBaselineCourse:
      Serial.print("RTK Baseline Course:");
      Serial.println(gnss.GetBaselineCourse());
      break;
    default:
      Serial.println("nothing");
      break;
    }
  }
  return true;
}
