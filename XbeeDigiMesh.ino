#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

const int RX_Xbee = 2;
const int TX_Xbee = 3;
SoftwareSerial serialXbee(RX_Xbee, TX_Xbee);

const int RX_Gps = 6;
const int TX_Gps = 7;
SoftwareSerial serialGps(RX_Gps, TX_Gps);

Adafruit_GPS GPS(&serialGps);
#define GPSECHO  false
int incomingByte = 0;
int cnt = 0;
int ledStatus = 0;

void setup() {
  // Configure Xbee serial port
  pinMode(RX_Xbee, INPUT);
  pinMode(TX_Xbee, OUTPUT);
  serialXbee.begin(9600);
  
  // Configure GPS serial port
  //pinMode(RX_Gps, INPUT);
  //pinMode(TX_Gps, OUTPUT);
  //serialGps.begin(9600);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  
  delay(1000);
  
  // Configure LED
  pinMode(13, OUTPUT);
  
  // Configure Serial Console (USB) port
  Serial.begin(9600);
}

uint32_t timer = millis();
void loop() {
  cnt++;
  
  // Blink LED
  if(cnt >= 20000){
    cnt = 0;
    if(ledStatus == 0)
    {
      digitalWrite(13,HIGH);
      ledStatus = 1;
    } else {
      digitalWrite(13,LOW);
      ledStatus = 0;
    }
  }
  
  char c = GPS.read();
  if(GPSECHO)
    if(c) Serial.print(c);
    
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    serialXbee.print("\nTime: ");
    serialXbee.print(GPS.hour, DEC); serialXbee.print(':');
    serialXbee.print(GPS.minute, DEC); serialXbee.print(':');
    serialXbee.print(GPS.seconds, DEC); serialXbee.print('.');
    serialXbee.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    serialXbee.print("Fix: "); serialXbee.print((int)GPS.fix);
    serialXbee.print(" quality: "); serialXbee.println((int)GPS.fixquality); 
    if (GPS.fix) {
      serialXbee.print("Location: ");
      serialXbee.print(GPS.latitude, 4); serialXbee.print(GPS.lat);
      serialXbee.print(", "); 
      serialXbee.print(GPS.longitude, 4); serialXbee.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      serialXbee.print("Satellites: "); serialXbee.println((int)GPS.satellites);
    }
  }
  


  
  /*
  // Forward data from Xbee to USB Console
  if (serialXbee.available()) {
    Serial.write(serialXbee.read());
  }
  
  // Forward data from USB Console to Xbee
  if (Serial.available()) {
    serialXbee.write(Serial.read());
  }
  
  // Forward data from GPS to USB Console
  if (serialGps.available()) {
    
    Serial.write(serialGps.read());
    //serialXbee.write(serialGps.read());
  }
  */
}
