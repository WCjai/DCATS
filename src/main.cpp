#include "ardupilotmega/mavlink.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "Arduino.h"
#include <esp_now.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include <WiFi.h>

#define BUFFER_SIZE 1024

int baudrate = 57600;

int Rtos_delay = 90;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t system_id = 11; // Your i.e. Arduino sysid
uint8_t component_id = 162; // Your i.e. Arduino compid
uint8_t type = MAV_TYPE_FLARM;
uint8_t autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
uint8_t received_sysid; // Pixhawk sysid
uint8_t received_compid; // Pixhawk compid
static uint8_t tx_buf[BUFFER_SIZE] = { 0 };
const uart_port_t CONFIG_UART_PORT_NUM = UART_NUM_2;


uint16_t vel_x ,vel_y ,vel_z, len;
uint32_t time_boot_ms, latitude, longitude ,altitude;
uint8_t buffer[MAVLINK_MAX_PACKET_LEN]; // Buffer to hold encoded GPS data

static const char *TAG = "GPS_LOG";



void Stream() {
    delay(2000);
    int flag = 1;
    ESP_LOGE("AutoPilot", "Sending Heartbeats...");
    
    mavlink_message_t msghb;
    mavlink_heartbeat_t heartbeat;
    uint8_t bufhb[MAVLINK_MAX_PACKET_LEN];
    
    // Pack and send a heartbeat message
    mavlink_msg_heartbeat_pack(system_id, component_id, &msghb, type, autopilot, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_STANDBY);
    uint16_t lenhb = mavlink_msg_to_send_buffer(bufhb, &msghb);
    
    delay(1000);
    
    // Send the heartbeat using UART
    uart_write_bytes(CONFIG_UART_PORT_NUM, (const char *)bufhb, lenhb);
    ESP_LOGE("AutoPilot", "Heartbeats sent! Now will check for received heartbeats to record sysid and compid...");

    // Looping until we get the required data
    while (flag == 1) {
        delay(1);
        size_t available_bytes = 0;
        
        // Check how many bytes are available in the UART buffer
        uart_get_buffered_data_len(CONFIG_UART_PORT_NUM, &available_bytes);
        
        // If there are bytes available, process them
        while (available_bytes > 0) {
            mavlink_message_t msgpx;
            mavlink_status_t statuspx;
            uint8_t ch;
            
            // Read one byte from the UART buffer
            int len = uart_read_bytes(CONFIG_UART_PORT_NUM, &ch, 1, 20 / portTICK_RATE_MS);
            
            if (len > 0) {
                // Try to parse the MAVLink message
                if (mavlink_parse_char(MAVLINK_COMM_0, ch, &msgpx, &statuspx)) {
                    ESP_LOGE("AutoPilot", "Message Parsing Done!");
                    
                    // Check if the message is a heartbeat message
                    if (msgpx.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        mavlink_heartbeat_t packet;
                        mavlink_msg_heartbeat_decode(&msgpx, &packet);
                        
                        // Record the system ID and component ID from the heartbeat
                        received_sysid = msgpx.sysid;   // Pixhawk system ID
                        received_compid = msgpx.compid; // Pixhawk component ID
                        
                        // Log the received IDs
                        ESP_LOGE("AutoPilot", "Received SysID: %d", received_sysid);
                        ESP_LOGE("AutoPilot", "Received CompID: %d", received_compid);
                        ESP_LOGE("AutoPilot", "sysid and compid successfully recorded");
                        
                        flag = 0; // Exit the loop after successful reception
                    }
                }
            }
            
            // Update available bytes after reading
            uart_get_buffered_data_len(CONFIG_UART_PORT_NUM, &available_bytes);
        }
    }

    ESP_LOGE("AutoPilot", "Now sending request for data stream...");
    delay(2000);
    mavlink_message_t msgds;
    uint8_t bufds[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_request_data_stream_pack(system_id, component_id, &msgds, received_sysid, received_compid, MAV_DATA_STREAM_ALL , 0x05, 1);
    uint16_t lends = mavlink_msg_to_send_buffer(bufds, &msgds);
    delay(1000);
    uart_write_bytes(CONFIG_UART_PORT_NUM, (const char *)bufds, lends);
    ESP_LOGE("AutoPilot", "Request sent! Now you are ready to recieve datas...");

}

void ReadGPS(uint32_t *lat, uint32_t *lon, uint32_t *alt, uint16_t *velx, uint16_t *vely, uint16_t *velz, uint32_t *time) {
  size_t available_bytes = 0;
  uart_get_buffered_data_len(CONFIG_UART_PORT_NUM, &available_bytes);
  uint16_t x, y, z;
  uint32_t t , la, lo, al;
  if (available_bytes > 0) {
    int len = uart_read_bytes(CONFIG_UART_PORT_NUM, tx_buf, sizeof(tx_buf), pdMS_TO_TICKS(Rtos_delay));
    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status1;
      for (int i = 0; i < len; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_1, tx_buf[i], &msg, &status1)) {
          uint8_t send_buf[MAVLINK_MAX_PACKET_LEN];
          int send_len = mavlink_msg_to_send_buffer(send_buf, &msg);
          switch (msg.msgid) {
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                mavlink_global_position_int_t data;
                mavlink_msg_global_position_int_decode(&msg, &data);
                la = data.lat;
                lo = data.lon;
                al = data.relative_alt;
                x = data.vx;
                y = data.vy;
                z = data.vz;
                t = data.time_boot_ms;
          }
        }
      } 

    }
  }

  *lat = la;
  *lon = lo;
  *alt = al;
  *velx = x;
  *vely = y;
  *velz = z;
  *time = t;
  return;
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    mavlink_message_t msg;
    mavlink_status_t status;
    
    // Parse the incoming MAVLink data
    for (int i = 0; i < len; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_1, incomingData[i], &msg, &status)) {
            // If the message is a GPS message, decode it
            if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                mavlink_global_position_int_t decoded_gps;
                mavlink_msg_global_position_int_decode(&msg, &decoded_gps);

                // Print the received GPS data
                ESP_LOGE(TAG, "Received GPS Data:");
                ESP_LOGE(TAG, "Lat: %ld, Lon: %ld, Alt: %ld", decoded_gps.lat, decoded_gps.lon, decoded_gps.alt);
                ESP_LOGE(TAG, "VelX: %d, VelY: %d, VelZ: %d", decoded_gps.vx, decoded_gps.vy, decoded_gps.vz);
                ESP_LOGE(TAG, "Time Boot: %ld", decoded_gps.time_boot_ms);
            }
        }
    }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Message Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void encodedGPS(uint8_t *buffer, uint16_t *len) {
    // Read the GPS data (assuming these variables are globally defined or accessible)
    ReadGPS(&latitude, &longitude, &altitude, &vel_x, &vel_y, &vel_z, &time_boot_ms);

    // Create MAVLink message and GPS structure
    mavlink_message_t gps;
    mavlink_global_position_int_t gpspos;

    // Fill GPS position data
    gpspos.time_boot_ms = time_boot_ms;
    gpspos.lat = latitude;
    gpspos.lon = longitude;
    gpspos.alt = altitude;
    gpspos.relative_alt = 0;
    gpspos.vx = vel_x;
    gpspos.vy = vel_y;
    gpspos.vz = vel_z;
    gpspos.hdg = 0;

    // Encode GPS position into MAVLink message
    mavlink_msg_global_position_int_encode(received_sysid, received_compid, &gps, &gpspos);

    // Pack the MAVLink message into the provided buffer
    *len = mavlink_msg_to_send_buffer(buffer, &gps);
}

void setup() {
    uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(CONFIG_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_UART_PORT_NUM, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_UART_PORT_NUM, 2 * BUFFER_SIZE, 2 * BUFFER_SIZE, 0, NULL, 0));

    Stream();  // Sending heartbeats and setting up communication with the autopilot

    // Initialize ESP-NOW
    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_max_tx_power(40);  // Max transmission power for longer range

    esp_err_t init_result = esp_now_init();
    if (init_result != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW Initialization failed");
        return;
    } else {
        ESP_LOGI(TAG, "ESP-NOW Initialized successfully");
    }

    // Register the callbacks
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    // Add broadcast peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 1;  // Ensure the channel is the same across all devices
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add broadcast peer");
        return;
    } else {
        ESP_LOGI(TAG, "Broadcast peer added successfully");
    }
}


void loop() {
    // Encode the GPS data and send it via ESP-NOW
    encodedGPS(buffer, &len);
    esp_err_t result = esp_now_send(broadcastAddress, buffer, len);
    if (result == ESP_OK) {
        //ESP_LOGE(TAG, "GPS message sent successfully via ESP-NOW");
    } else {
        ESP_LOGE(TAG, "Error sending GPS message");
    }

    //delay(100);  // 10 Hz sending rate
}