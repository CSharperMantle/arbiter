#include <Arduino.h>
#include <stdint.h>
#include <pb_encode.h>
#include <WirelessClientMessage.pb.h>

void setup() {
  WirelessClientMessage my_message;
  uint8_t buffer[256];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  pb_encode(&stream, WirelessClientMessage_fields, &my_message);
  
  Serial.begin(115200);
  Serial.write(buffer, 256);
}

void loop() {

}
