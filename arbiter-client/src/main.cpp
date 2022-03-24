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
#include <WiFiClient.h>
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
static const std::uint16_t CLIENT_UDP_PORT = 10010u;

static const std::uint8_t GPIO_ONBOARD_LED = LED_BUILTIN;
static const std::uint8_t GPIO_MAIN_BUTTON = 0u;

/* == GLOBAL OBJECTS == */

static WiFiClient client{};
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

static std::uint8_t incoming_queue_storage[QUEUE_SIZE * sizeof(WirelessHostMessage)];
static StaticQueue_t incoming_queue{};
static QueueHandle_t incoming_queue_handle = nullptr; // Incoming packet queue.
static std::uint8_t outgoing_queue_storage[QUEUE_SIZE * sizeof(WirelessClientMessage)];
static StaticQueue_t outgoing_queue{};
static QueueHandle_t outgoing_queue_handle = nullptr; // Outgoing packet queue.

static TaskHandle_t task_wifi_read_handle = nullptr;
static TaskHandle_t task_wifi_send_handle = nullptr;
static TaskHandle_t task_message_processor_handle = nullptr;
static TaskHandle_t task_button_debounced_handle = nullptr;

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

/* == INTERRUPT HANDLERS == */

/**
 * \brief Button press detection.
 */
static void IRAM_ATTR handler_main_button()
{
  system_state.main_button_irq_issued = true;
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
  pinMode(GPIO_MAIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(GPIO_MAIN_BUTTON, handler_main_button, FALLING);

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

  udp.begin(CLIENT_UDP_PORT);
  printf("i: UDP listening port: %u\n", CLIENT_UDP_PORT);

  printf("i: Peripherals initialized.\n");
}

/**
 * \brief Handshake with Host.
 */
static void action_handshake()
{
  printf("i: Handshaking with Host.\n");

  std::uint32_t rc = 0; // The return code.

  while (true)
  {
    printf("v: Probing for Host...\n");
    WirelessClientMessage msg{};
    msg.type = WirelessClientMessage_Type_HANDSHAKE_REQUEST;

    std::uint8_t msg_buffer[BUFFER_SIZE];
    std::memset(msg_buffer, 0, BUFFER_SIZE * sizeof(std::uint8_t));
    // Pack message.
    auto stream = pb_ostream_from_buffer(msg_buffer, sizeof(msg_buffer));
    rc = pb_encode(&stream, WirelessClientMessage_fields, &msg);
    assert(rc);
    // Broadcast probe via UDP.
    rc = udp.beginPacket(helper_get_broadcast_address(static_cast<uint32_t>(system_state.local_ip),
                                                      static_cast<uint32_t>(system_state.subnet_mask)),
                         HOST_UDP_PORT);
    assert(rc);
    rc = udp.write(msg_buffer, stream.bytes_written);
    assert(rc == stream.bytes_written);
    rc = udp.endPacket();
    assert(rc);
    printf("v: Waiting for handshake response...\n");
    // Forever loop until we receive any packet.
    uint32_t packetSize = 0;
    while (packetSize == 0)
    {
      packetSize = udp.parsePacket();
      taskYIELD();
    }

    std::uint8_t response_buffer[BUFFER_SIZE];
    std::memset(response_buffer, 0, BUFFER_SIZE * sizeof(std::uint8_t));

    udp.readBytes(response_buffer, sizeof(response_buffer));
    // Unpack response.
    auto istream = pb_istream_from_buffer(response_buffer, packetSize);
    WirelessHostMessage response{};
    rc = pb_decode(&istream, WirelessHostMessage_fields, &response);

    if (rc && response.type == WirelessHostMessage_Type_HANDSHAKE_ASSIGNMENT)
    {
      // HACK: We need to reverse the endianness of local IP to match that of received IP.
      std::uint32_t reversed_local_ip = helper_reverse_bytes(static_cast<std::uint32_t>(system_state.local_ip));
      if (response.content.assignment.address_ipv4 == reversed_local_ip)
      {
        // Assignment answered.
        system_state.host_ip = udp.remoteIP();
        system_state.id = response.content.assignment.id;

        printf("v: Got handshake assignment from Host.\n");
        printf("i: Assigned ID: %d\n", system_state.id);
        printf("v: Host: %s\n", system_state.host_ip.toString().c_str());
        break;
      }
      else
      {
        // The Host has bugs so that it mangles up IP addresses of this
        // Client. Ignoring the packet.
        printf("w: Assigned and local IP mismatch (%u != %u). Packet ignored.\n",
               response.content.assignment.address_ipv4,
               reversed_local_ip);
      }
    }
    else
    {
      // Either the packet is corrupt or another broadcast packet is received.
      // We just try again.
      printf("v: Irrelevant UDP packet received. \n");
    }
  }

  {
    // Now we need to confirm the handshake.
    WirelessClientMessage confirmation{};
    confirmation.type = WirelessClientMessage_Type_HANDSHAKE_ASSIGNMENT_CONFIRMATION;
    confirmation.which_content = WirelessClientMessage_assignment_tag;
    confirmation.content.assignment.address_ipv4 = static_cast<uint32_t>(system_state.local_ip);
    confirmation.content.assignment.id = system_state.id;
    std::uint8_t confirmation_buffer[BUFFER_SIZE];
    auto confirmation_stream = pb_ostream_from_buffer(confirmation_buffer, sizeof(confirmation_buffer));
    rc = pb_encode(&confirmation_stream, WirelessClientMessage_fields, &confirmation);
    assert(rc);

    // Send the confirmation.
    rc = udp.beginPacket(system_state.host_ip, HOST_UDP_PORT);
    assert(rc);
    rc = udp.write(confirmation_buffer, confirmation_stream.bytes_written);
    assert(rc == confirmation_stream.bytes_written);
    rc = udp.endPacket();
    assert(rc);
    printf("i: Handshake done.\n");
  }

  {
    // Establish persistent TCP connection.
    printf("i: Establishing TCP connection...\n");
    while (!client.connect(system_state.host_ip, HOST_TCP_PORT, 200))
    {
      taskYIELD();
    }
    printf("i: TCP connection established.\n");
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

/**
 * \brief Respond Ping with a Pong.
 */
static void action_pong()
{
  // std::uint32_t rc = 0; // The return code.
  BaseType_t x_rc = 0; // The BaseType_t return code.

  WirelessClientMessage pong_msg{};
  pong_msg.type = WirelessClientMessage_Type_PONG;

  // Send message.
  x_rc = xQueueSendToBack(outgoing_queue_handle, &pong_msg, portMAX_DELAY);
  assert(x_rc == pdTRUE);
  printf("v: Responding to Ping message.\n");
}

/* == TASKS (THREAD ENTRY POINTS) == */

/**
 * \brief Read TCP messages via WiFi from Host.
 */
static void task_wifi_read(void *_)
{
  uint32_t rc = 0; // The return code.
  BaseType_t x_rc; // The BaseType_t return code.

  while (true)
  {
    // Wait for a packet.
    while (!client.available())
    {
      taskYIELD();
    }

    // Read the packet.
    std::uint8_t buffer[BUFFER_SIZE];
    uint8_t packetSize = client.readBytes(buffer, sizeof(buffer));
    assert(packetSize);
    auto istream = pb_istream_from_buffer(buffer, packetSize);
    WirelessHostMessage msg{};
    rc = pb_decode(&istream, WirelessHostMessage_fields, &msg);
    if (!rc)
    {
      printf("w: Failed to decode message.\n");
      continue;
    }

    // Send the message to the queue.
    x_rc = xQueueSendToBack(incoming_queue_handle, &msg, portMAX_DELAY);
    if (x_rc != pdPASS)
    {
      printf("w: Incoming message emplacement failed.\n");
    }
    else
    {
      printf("v: Incoming message enqueued.\n");
    }
  }
}

/**
 * \brief Send TCP messages via WiFi to Host.
 */
static void task_wifi_send(void *_)
{
  uint32_t rc = 0; // The return code.
  BaseType_t x_rc; // The BaseType_t return code.

  while (true)
  {
    // Wait for a message.
    WirelessClientMessage msg{};
    x_rc = xQueueReceive(outgoing_queue_handle, &msg, portMAX_DELAY);
    assert(x_rc == pdPASS);

    // Encode the message.
    std::uint8_t buffer[BUFFER_SIZE];
    auto stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    rc = pb_encode(&stream, WirelessClientMessage_fields, &msg);
    assert(rc);

    // Send the message.
    rc = client.write(buffer, stream.bytes_written);
    assert(rc == stream.bytes_written);

    printf("v: Outgoing message sent.\n");
  }
}

/**
 * \brief Process each incoming message.
 */
static void task_message_processor(void *_)
{
  // std::uint32_t rc = 0; // The return code.
  BaseType_t x_rc; // The BaseType_t return code.

  while (true)
  {
    WirelessHostMessage incoming_message{};
    x_rc = xQueueReceive(incoming_queue_handle, &incoming_message, portMAX_DELAY);
    assert(x_rc == pdPASS);
    printf("i: Processing incoming message...\n");

    switch (incoming_message.type)
    {
    case WirelessHostMessage_Type_PING:
      action_pong();
      break;
    default:
      break;
    }
  }
}

/**
 * \brief Button press detection with debounce.
 */
static void task_button_debounced(void *_)
{
  // std::uint32_t rc = 0; // The return code.
  BaseType_t x_rc;      // The BaseType_t return code.

  while (true)
  {
    if (!system_state.main_button_irq_issued)
    {
      taskYIELD(); // Wait for the button to be pressed.
    }

    WirelessClientMessage outgoing_msg{};
    outgoing_msg.type = WirelessClientMessage_Type_ARBITRATION_REQUEST;
    x_rc = xQueueSendToBack(outgoing_queue_handle, &outgoing_msg, portMAX_DELAY);
    assert(x_rc == pdPASS);

    system_state.main_button_irq_issued = false;
    
    printf("v: Main button pressed.\n");
    delay(100);
  }
}

/* == MAIN == */

void setup()
{
  BaseType_t x_rc; // The BaseType_t return code.

  auto app_cpu = xPortGetCoreID();
  printf("v: User code running on CPU %d\n", app_cpu);

  action_init_periph();
  action_handshake();
  action_init_data_structures();

  x_rc = xTaskCreatePinnedToCore(task_wifi_read,
                                 "wifi_read",
                                 2048,
                                 nullptr,
                                 1,
                                 &task_wifi_read_handle,
                                 app_cpu);
  assert(x_rc == pdPASS);
  assert(task_wifi_read_handle);
  x_rc = xTaskCreatePinnedToCore(task_wifi_send,
                                 "wifi_send",
                                 2048,
                                 nullptr,
                                 1,
                                 &task_wifi_send_handle,
                                 app_cpu);
  assert(x_rc == pdPASS);
  assert(task_wifi_send_handle);
  x_rc = xTaskCreatePinnedToCore(task_message_processor,
                                 "message_processor",
                                 2048,
                                 nullptr,
                                 1,
                                 &task_message_processor_handle,
                                 app_cpu);
  assert(x_rc == pdPASS);
  assert(task_message_processor_handle);
  x_rc = xTaskCreatePinnedToCore(task_button_debounced,
                                 "button_debounced",
                                 2048,
                                 nullptr,
                                 1,
                                 &task_button_debounced_handle,
                                 app_cpu);

  // Indicate that we have finished initialization.
  digitalWrite(GPIO_ONBOARD_LED, LOW);
}

void loop()
{
  printf("i: Setup done. Main thread quitting.\n");
  vTaskDelete(nullptr);
}
