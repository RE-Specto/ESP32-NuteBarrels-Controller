//#include "main.h"
#include "NTPClient.h"
// #include "Expanders.h"
// #include "SystemState.h"
// #include "Barrels.h"
// #include "FlowSensor.h"
// #include "PresureSensor.h"
#include "WebServer.h"
// #include "Filesystem.h"
// #include "Modem.h"
// #include "NTPClient.h"
// #include "FMSD.h"
#include "globals.h"


/*-------- NTP Begin ----------*/
// send an NTP request to the time server at the given address
void NTPClass::sendNTPpacket(IPAddress &address, byte (&packetBuffer)[NTP_PACKET_SIZE])
{
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011; // LI, Version, Mode
    packetBuffer[1] = 0;          // Stratum, or type of clock
    packetBuffer[2] = 6;          // Polling Interval
    packetBuffer[3] = 0xEC;       // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;
    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    WebServer.UDP.beginPacket(address, NTP_PORT);
    WebServer.UDP.write(packetBuffer, NTP_PACKET_SIZE);
    WebServer.UDP.endPacket();
}

time_t NTPClass::getNtpTime()
{
    isTimeSync = true;                  // prevent retrigger untill done
    byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
    IPAddress ntpServerIP;              // NTP server's ip address
    while (WebServer.UDP.parsePacket() > 0)
        ;                                       // discard any previously received packets
    WiFi.hostByName(NTP_HOSTNAME, ntpServerIP); // get a random server from the pool
#ifdef DEBUG_NET
    LOG.print("NTP time request via ");
    LOG.print(ntpServerIP);
#endif
    sendNTPpacket(ntpServerIP, packetBuffer);
    uint32_t beginWait = millis();
    while (millis() - beginWait < NTP_TIMEOUT)
    {
        byte size = WebServer.UDP.parsePacket();
        if (size >= NTP_PACKET_SIZE)
        {
            LOG.println(" Received NTP Response");
            WebServer.UDP.read(packetBuffer, NTP_PACKET_SIZE); // read packet into the buffer
            unsigned long secsSince1900;
            // convert four bytes starting at location 40 to a long integer
            secsSince1900 = (unsigned long)packetBuffer[40] << 24;
            secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
            secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
            secsSince1900 |= (unsigned long)packetBuffer[43];
            uint16_t DTS = 0;                                              //mySettings.DTS?3600UL:0;
            uint16_t timeZone = 3;                                         // temporary
            return secsSince1900 - 2208988800UL + DTS + timeZone * 3600UL; // added DTS
        }
    }
    LOG.println("No NTP Response :-(");
    isTimeSync = false; // if failed - set back to false
    return 0;           // return 0 if unable to get the time
}

//RTC module
/*
RtcDS3231<TwoWire> Rtc(Wire);



void setupRTC (){
    Rtc.Begin();
    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    if (!Rtc.IsDateTimeValid()) {
        if (Rtc.LastError() != 0) // communication error
        {
              LOG.println(Rtc.LastError());
        }
        else // RTC lost confidence in the DateTime
            Rtc.SetDateTime(compiled);
    }
    if (!Rtc.GetIsRunning()) 
        Rtc.SetIsRunning(true);
    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled)
        Rtc.SetDateTime(compiled); // sets time to PC time if needed
    Rtc.Enable32kHzPin(false);
    Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone); 
}

//temporary test
void testPrintRTC() 
{
    //RtcDateTime now = Rtc.GetDateTime();
    //printDateTime(now);
    LOG.println(getDateTimeNow());
	RtcTemperature temp = Rtc.GetTemperature();
    LOG.print(temp.AsFloatDegC());
    LOG.println(" *C");
}


//void printDateTime(const RtcDateTime& dt)

// Returns Date-Time string
String getDateTimeNow(){
    RtcDateTime currentTime = Rtc.GetDateTime();    //get the time from the RTC
    char str[20];   //declare a string as an array of chars
    sprintf(str, "%d/%d/%d %d:%d:%d",     //%d allows to print an integer to the string
        currentTime.Year(),currentTime.Month(),currentTime.Day(),
        currentTime.Hour(),currentTime.Minute(),currentTime.Second());
    return (str);
}



alarm - needs an interrupt pin connected to RTC square wave / alarm pin (SQW) 
https://github.com/Makuna/Rtc/wiki/RtcDS3231-AlarmOne
https://techtutorialsx.com/2017/02/04/esp8266-ds3231-alarm-when-seconds-match/
*/
/*-------- NTP END ----------*/
NTPClass TimeClient;
