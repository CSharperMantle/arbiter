/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Rong "Mantle" Bao.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Software, and to permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies
 * or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <Arduino.h>
#include <FreeRTOS.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include "WirelessClientMessage.pb.h"
#include "WirelessHostMessage.pb.h"

static const uint32_t BUFFER_SIZE = 512;
static const uint32_t QUEUE_SIZE = 10;

static const char *WIFI_SSID = "[REPLACE_ME]";
static const char *WIFI_PASS = "[REPLACE_ME]";
static const uint16_t HOST_UDP_PORT = 10010;
static const uint16_t HOST_TCP_PORT = 10011;

static WiFiClient client{};
static WiFiUDP udp{};

/**
 * \brief Runtime-editable system config.
 */
struct SystemConfig
{
  uint32_t id = static_cast<uint32_t>(-1);
  IPAddress local_ip{static_cast<uint32_t>(-1)};
  IPAddress host_ip{static_cast<uint32_t>(-1)};
};
static SystemConfig system_config{};

static uint8_t incoming_queue_storage[QUEUE_SIZE * sizeof(WirelessHostMessage)];
static StaticQueue_t incoming_queue{};
static QueueHandle_t incoming_queue_handle = nullptr;
static uint8_t outgoing_queue_storage[QUEUE_SIZE * sizeof(WirelessClientMessage)];
static StaticQueue_t outgoing_queue{};
static QueueHandle_t outgoing_queue_handle = nullptr;

/**
 * \brief Initialize all peripherals.
 *
 * Set up pins, and connect to WiFi.
 */
static void action_init_periph()
{
  printf("Initializing peripherals...\n");

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }

  system_config.local_ip = WiFi.localIP();

  printf("i: Connected to %s\n", WIFI_SSID);
  printf("v: MAC address: %s\n", WiFi.macAddress().c_str());
  printf("v: IP address: %s\n", system_config.local_ip.toString().c_str());
  printf("v: Subnet mask: %s\n", WiFi.subnetMask().toString().c_str());

  printf("Peripherals initialized.\n");
}

/**
 * \brief Handshake with Host.
 */
static void action_handshake()
{
  printf("i: Handshaking with Host...\n");

  // Universal return code container.
  uint32_t status = 0;

  {
    // Construct probe message.
    printf("v: Sending probe to Host...\n");
    WirelessClientMessage msg{
        .type = WirelessClientMessage_Type_HANDSHAKE_REQUEST,
    };
    uint8_t msg_buffer[BUFFER_SIZE];
    auto stream = pb_ostream_from_buffer(msg_buffer, sizeof(msg_buffer));
    status = pb_encode(&stream, WirelessClientMessage_fields, &msg);
    assert(status);

    // Send probe message via UDP.
    status = udp.beginPacket(system_config.local_ip, HOST_UDP_PORT);
    assert(status);
    status = udp.write(msg_buffer, stream.bytes_written);
    assert(status == stream.bytes_written);
    status = udp.endPacket();
    assert(status);
  }

  // Wait for the response from the server.
  while (true)
  {
    printf("v: Waiting for handshake response...\n");
    while (udp.parsePacket() == 0)
    {
      delay(1);
    }
    printf("v: Got a UDP packet.\n");
    uint8_t response_buffer[BUFFER_SIZE];
    udp.readBytes(response_buffer, sizeof(response_buffer));
    auto istream = pb_istream_from_buffer(response_buffer, sizeof(response_buffer));
    WirelessHostMessage response{};
    status = pb_decode(&istream, WirelessHostMessage_fields, &response);
    assert(status);
    if (response.type == WirelessHostMessage_Type_HANDSHAKE_ASSIGNMENT)
    {
      // Assignment answered.
      system_config.host_ip = udp.remoteIP();
      system_config.id = response.content.assignment.id;

      printf("v: Got handshake assignment from Host.\n");
      printf("i: Assigned ID: %d\n", system_config.id);
      printf("v: Host: %s\n", system_config.host_ip.toString().c_str());
      break;
    }
    else
    {
      printf("v: Not an expected response. \n");
    }
  }

  {
    // Now we need to confirm the handshake.
    WirelessClientMessage confirmation{
        .type = WirelessClientMessage_Type_HANDSHAKE_ASSIGNMENT_CONFIRMATION,
        .which_content = WirelessClientMessage_assignment_tag,
        .content = {
            .assignment = {
                .address_ipv4 = static_cast<uint32_t>(system_config.local_ip),
                .id = system_config.id,
            },
        },
    };
    uint8_t confirmation_buffer[BUFFER_SIZE];
    auto confirmation_stream = pb_ostream_from_buffer(confirmation_buffer, sizeof(confirmation_buffer));
    status = pb_encode(&confirmation_stream, WirelessClientMessage_fields, &confirmation);
    assert(status);

    // Send the confirmation.
    status = udp.beginPacket(system_config.host_ip, HOST_UDP_PORT);
    assert(status);
    status = udp.write(confirmation_buffer, confirmation_stream.bytes_written);
    assert(status == confirmation_stream.bytes_written);
    status = udp.endPacket();
    assert(status);
    printf("i: Handshake done.\n");
  }
}

/**
 * \brief Initialize relevant data structures.
 */
static void action_init_data_structures()
{
  printf("i: Initializing data structures...\n");
  incoming_queue_handle = xQueueCreateStatic(QUEUE_SIZE,
                                             sizeof(WirelessHostMessage),
                                             incoming_queue_storage,
                                             &incoming_queue);
  assert(incoming_queue_handle);
  outgoing_queue_handle = xQueueCreateStatic(QUEUE_SIZE,
                                             sizeof(WirelessClientMessage),
                                             outgoing_queue_storage,
                                             &outgoing_queue);
  assert(outgoing_queue_handle);
  printf("i: Data structures initialized.\n");
}

void setup()
{
  auto app_cpu = xPortGetCoreID();
  printf("v: User code running on CPU %d\n", app_cpu);

  action_init_periph();
  action_handshake();
  action_init_data_structures();
}

void loop()
{
  printf("v: Main thread quitting\n");
  vTaskDelete(nullptr);
}
