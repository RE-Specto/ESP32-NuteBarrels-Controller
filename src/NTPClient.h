//#pragma once
#ifndef NTP_CLIENT
#define NTP_CLIENT
#include <Arduino.h>
//#include <RtcDS3231.h>

#define SYNC_INTERVAL 600 // NTP sync - in seconds
#define NTP_PACKET_SIZE 48
// NTP time is in the first 48 bytes of message
#define NTP_HOSTNAME "pool.ntp.org"
#define NTP_PORT 123
#define NTP_TIMEOUT 5000
#define LOCAL_UDP_PORT 8888 // local port to listen for UDP packets

class NTPClass
{
private:
    bool isTimeSync = false;
    void sendNTPpacket(IPAddress &address, byte (&packetBuffer)[NTP_PACKET_SIZE]);
public:
    time_t getNtpTime();
};

extern NTPClass TimeClient;
#endif