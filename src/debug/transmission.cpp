#include "transmission.h"

#include <WiFi.h>

// For UDP sending
const char *udpAddress = "192.168.178.220";
const int udpPort = 8089;
WiFiUDP udp;

char packetBuffer[512];

void debugSendFFT(double majorPeakHz, const double bins[])
{
  // Prepare and send package
  snprintf(packetBuffer,
           sizeof packetBuffer,
           "bett peak=%.4f,hz1=%.4f,hz2=%.4f,hz3=%.4f,hz4=%.4f,hz5=%.4f,hz6=%.4f,hz7=%.4f,hz8=%.4f,hz9=%.4f,hz10=%.4f\n",
           majorPeakHz,
           bins[1],
           bins[2],
           bins[3],
           bins[4],
           bins[5],
           bins[6],
           bins[7],
           bins[8],
           bins[9],
           bins[10]);

  if (WiFi.status() == WL_CONNECTED)
  {
    udp.beginPacket(udpAddress, udpPort);
    udp.print(packetBuffer);
    udp.endPacket();
  }
}