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

#include <cstdint>
#include <cstring>
#include <Arduino.h>
#include <FreeRTOS.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include "WirelessClientMessage.pb.h"
#include "WirelessHostMessage.pb.h"

/* == CONSTS == */

static const std::uint32_t BUFFER_SIZE = 512u;
static const std::uint32_t QUEUE_SIZE = 10u;

// static const char *WIFI_SSID = "302";
// static const char *WIFI_PASS = "^97Y3S8qht$97534";
static const char *WIFI_SSID = "cisco_3";
static const char *WIFI_PASS = "xiaoshiwuxian";
static const std::uint16_t HOST_UDP_PORT = 10010u;
static const std::uint16_t HOST_TCP_PORT = 10011u;
static const std::uint8_t HOST_MAX_CONNECTIONS = 20u;
static const std::uint16_t CLIENT_UDP_PORT = 10010u;

static const std::uint8_t GPIO_ONBOARD_LED = LED_BUILTIN;

/* == GLOBAL OBJECTS == */

static WiFiServer server{HOST_TCP_PORT, HOST_MAX_CONNECTIONS};
static WiFiUDP udp{};

/**
 * \brief Runtime-editable system config.
 */
static struct SystemState
{
  uint32_t id = static_cast<std::uint32_t>(-1);
  IPAddress local_ip{0u};
  IPAddress host_ip{0u};
  IPAddress subnet_mask{0u};

  volatile bool main_button_irq_issued = false;
} system_state{};

/* == HANDLES == */

static std::uint8_t incoming_queue_storage[QUEUE_SIZE * sizeof(WirelessClientMessage)];
static StaticQueue_t incoming_queue{};
static QueueHandle_t incoming_queue_handle = nullptr; // Incoming packet queue.
static std::uint8_t outgoing_queue_storage[QUEUE_SIZE * sizeof(WirelessHostMessage)];
static StaticQueue_t outgoing_queue{};
static QueueHandle_t outgoing_queue_handle = nullptr; // Outgoing packet queue.

/* == HELPERS == */

/**
 * \brief Get broadcast address for a specific IP address and mask.
 */
static constexpr std::uint32_t helper_get_broadcast_address(const std::uint32_t ip,
                                                            const std::uint32_t mask) noexcept
{
  return ip | ~mask;
}

/**
 * \brief Reverse the byte order of a 32-bit integer.
 *
 * \see https://forum.arduino.cc/t/reversed-byte-order/699873/6
 */
static constexpr std::uint32_t helper_reverse_bytes(const std::uint32_t x) noexcept
{
  return static_cast<std::uint32_t>((x & 0xFF) << 24 | ((x >> 8) & 0xFF) << 16 | ((x >> 16) & 0xFF) << 8 | ((x >> 24) & 0xFF));
}

/**
 * \brief Print an array of bytes in HEX to console.
 */
static inline void helper_print_byte_array(const std::uint8_t array[], const std::uint32_t size) noexcept
{
  for (std::uint32_t i = 0u; i < size; ++i)
  {
    printf("%02X ", array[i]);
  }
  printf("\n");
}

/* == ACTIONS (SUBROUTINES) == */

/**
 * \brief Initialize all peripherals.
 *
 * Set up pins, and connect to WiFi.
 */
static void action_init_periph()
{
  printf("i: Initializing peripherals...\n");

  pinMode(GPIO_ONBOARD_LED, OUTPUT);
  // Indicate that we are initializing.
  digitalWrite(GPIO_ONBOARD_LED, HIGH);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    taskYIELD();
  }

  system_state.local_ip = WiFi.localIP();
  system_state.subnet_mask = WiFi.subnetMask();

  printf("i: Connected to %s\n", WIFI_SSID);
  printf("v: MAC address: %s\n", WiFi.macAddress().c_str());
  printf("v: IP address: %s\n", system_state.local_ip.toString().c_str());
  printf("v: Subnet mask: %s\n", system_state.subnet_mask.toString().c_str());

  udp.begin(HOST_UDP_PORT);
  printf("i: UDP listening port: %u\n", HOST_UDP_PORT);

  server.begin();
  printf("i: TCP listening port: %u\n", HOST_TCP_PORT);

  printf("i: Peripherals initialized.\n");
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

/* == TASKS (THREAD ENTRY POINTS) == */

/* == MAIN == */

void setup()
{
  BaseType_t x_rc; // The BaseType_t return code.

  auto app_cpu = xPortGetCoreID();
  printf("v: User code running on CPU %d\n", app_cpu);

  action_init_periph();
  action_init_data_structures();
}

void loop()
{
  printf("i: Setup done. Main thread quitting.\n");
  vTaskDelete(nullptr);
}
