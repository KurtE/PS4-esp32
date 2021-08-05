#include <PS4Controller.h>
#include <WiFi.h>

#define SerialOut Serial

// MACROS
char MAC[18];

volatile bool g_event_happened = false;
bool  g_previously_connected = false;


void ps4_callback() {
  g_event_happened = true;
}



void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 5000) ;
  Serial.print("ESP Board MAC Address:  ");
  WiFi.macAddress().toCharArray(MAC, 18);
  Serial.println(MAC);
  PS4.begin(MAC);
  Serial.println("Ready.");
  PS4.attach(&ps4_callback);
}

typedef union {
  uint32_t     data;
  ps4_button_t button;
} button_data_t;


ps4_t ps4data_prev;
// BUGBUG we won't copy or look at latestpacket...
uint32_t ps4_data_size = sizeof(ps4_t) - sizeof(uint8_t*);

void loop()
{
  bool isConnected = PS4.isConnected();
  if (g_previously_connected != isConnected) {
    if (isConnected) Serial.println("*** Connected ***");
    else Serial.println("*** DisConnected ***");
    g_previously_connected = isConnected;
  }
  /* At this point, the Esp32 is ready and waiting for the PS4 remote to connect, it will connect automatically once the PS4 remote is turned on using the PS4 button
        There are some buttons/events available in the "PS4Controller.h" library that are unused in this case*/
  if (g_event_happened && isConnected) {
    if (memcmp(&PS4.data, &ps4data_prev, ps4_data_size)) {
      memcpy(&ps4data_prev, &PS4.data, ps4_data_size);
      button_data_t button_data;
      g_event_happened = false;
      button_data.button = PS4.data.button;

      Serial.printf("%08X %3d - 0%4d %4d %4d %4d %4d %4d",
                    button_data.data, PS4.Battery(),
                    PS4.data.analog.stick.lx,
                    PS4.data.analog.stick.ly,
                    PS4.data.analog.stick.rx,
                    PS4.data.analog.stick.ry,
                    PS4.data.analog.button.l2,
                    PS4.data.analog.button.r2);

      // Trackpad touch 1: id, active, x, y
      uint16_t xc = ((PS4.data.latestPacket[37 + 12] & 0x0f) << 8) | PS4.data.latestPacket[36 + 12];
      uint16_t yc = PS4.data.latestPacket[38 + 12] << 4 | ((PS4.data.latestPacket[37 + 12] & 0xf0) >> 4);
      uint8_t  isTouched = (PS4.data.latestPacket[35 + 12] & 0x80) == 0;
      if (!isTouched) xc = yc = 0;
      Serial.printf(" - %4u %4u", xc, yc);

      // Accel info
      int accelx = (int16_t)(PS4.data.latestPacket[12 + 20] << 8) | PS4.data.latestPacket[12 + 19];
      int accelz = (int16_t)(PS4.data.latestPacket[12 + 22] << 8) | PS4.data.latestPacket[12 + 21];
      int accely = (int16_t)(PS4.data.latestPacket[12 + 24] << 8) | PS4.data.latestPacket[12 + 23];

      float ax = (float) accelx / 8192;
      float ay = (float) accely / 8192;
      float az = (float) accelz / 8192;
      Serial.printf(" - %4d(%3.2f) %4d(%3.2f) %4d(%3.2f)", accelx, ax, accely, ay, accelz, az);

      // Gyro info
      int gyroy = (int16_t)(PS4.data.latestPacket[12 + 14] << 8) | PS4.data.latestPacket[12 + 13];
      int gyroz = (int16_t)(PS4.data.latestPacket[12 + 16] << 8) | PS4.data.latestPacket[12 + 15];
      int gyrox = (int16_t)(PS4.data.latestPacket[12 + 18] << 8) | PS4.data.latestPacket[12 + 17];

      float gx = (float) gyrox * RAD_TO_DEG / 1024;
      float gy = (float) gyroy * RAD_TO_DEG / 1024;
      float gz = (float) gyroz * RAD_TO_DEG / 1024;
      Serial.printf(" - %4d(%3.2f) %4d(%3.2f) %4d(%3.2f)\n", gyrox, gx, gyroy, gy, gyroz, gz);
    }
  }
}
