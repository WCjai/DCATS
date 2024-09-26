#include "ardupilotmega/mavlink.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "Arduino.h"
#include <esp_now.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include <WiFi.h>

#define BUFFER_SIZE 256

int baudrate = 57600;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t system_id = 11; // Your system ID
uint8_t component_id = 162; // Your component ID
uint8_t type = MAV_TYPE_FLARM;
uint8_t autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
uint8_t received_sysid = 0; // Pixhawk sysid
uint8_t received_id = 0; // Pixhawk sysid
static uint8_t received_compid = 0; // Pixhawk compid
static uint8_t tx_buf[BUFFER_SIZE] = { 0 };
const uart_port_t CONFIG_UART_PORT_NUM = UART_NUM_2;

uint16_t vel_x, vel_y, vel_z, len;
uint32_t time_boot_ms, latitude, longitude, altitude;
uint8_t buffer[MAVLINK_MAX_PACKET_LEN]; // Buffer to hold GPS data

static const char *TAG = "GPS_LOG";

#define MAX_DEVICES 10  // Maximum number of unique MAC addresses you want to track


// GPS message structure
typedef struct gps_message {
    uint32_t time_boot_ms;
    uint8_t sysid;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t relative_alt;
    int16_t vx;
    int16_t vy;
    int16_t vz;
} gps_message;

// Structure to hold MAC and associated GPS data
typedef struct device_data {
    uint8_t mac[6];  // MAC address (6 bytes)
    gps_message gps_data;  // Associated GPS data
} device_data;

device_data devices[MAX_DEVICES];  // Array to hold MAC and associated data
int device_count = 0;  // Current count of devices in the table

gps_message myData;
gps_message receivedData;


int findDeviceIndex(const uint8_t *mac) {
    for (int i = 0; i < device_count; i++) {
        if (memcmp(devices[i].mac, mac, 6) == 0) {
            return i;  // Return index if MAC address is found
        }
    }
    return -1;  // MAC not found
}

void addOrUpdateDevice(const uint8_t *mac, const gps_message *data) {
    int index = findDeviceIndex(mac);

    if (index != -1) {
        // MAC address found, update existing data
        devices[index].gps_data = *data;
    } else {
        // MAC address not found, add new entry if space is available
        if (device_count < MAX_DEVICES) {
            memcpy(devices[device_count].mac, mac, 6);  // Copy MAC address
            devices[device_count].gps_data = *data;  // Copy GPS data
            device_count++;
        } else {
            ESP_LOGE(TAG, "Device table is full. Can't add new device.");
        }
    }
}

void printDeviceTable() {
    ESP_LOGI(TAG, "Device Table:");
    for (int i = 0; i < device_count; i++) {
        ESP_LOGE(TAG, "MAC: %02X:%02X:%02X:%02X:%02X:%02X | Lat: %ld, Lon: %ld, Alt: %ld, VelX: %d, VelY: %d, VelZ: %d, Time Boot: %ld",
                 devices[i].mac[0], devices[i].mac[1], devices[i].mac[2],
                 devices[i].mac[3], devices[i].mac[4], devices[i].mac[5],
                 devices[i].gps_data.lat, devices[i].gps_data.lon, devices[i].gps_data.alt,
                 devices[i].gps_data.vx, devices[i].gps_data.vy, devices[i].gps_data.vz,
                 devices[i].gps_data.time_boot_ms);
    }
}


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

    while (flag == 1) {
        delay(1);
        size_t available_bytes = 0;
        uart_get_buffered_data_len(CONFIG_UART_PORT_NUM, &available_bytes);
        
        while (available_bytes > 0) {
            mavlink_message_t msgpx;
            mavlink_status_t statuspx;
            uint8_t ch;
            
            int len = uart_read_bytes(CONFIG_UART_PORT_NUM, &ch, 1, 20 / portTICK_RATE_MS);
            if (len > 0) {
                if (mavlink_parse_char(MAVLINK_COMM_0, ch, &msgpx, &statuspx)) {
                    ESP_LOGE("AutoPilot", "Message Parsing Done!");
                    if (msgpx.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        received_sysid = msgpx.sysid;
                        received_id = received_sysid;
                        received_compid = msgpx.compid;
                        ESP_LOGE("AutoPilot", "Received SysID: %d, CompID: %d", received_sysid, received_compid);
                        flag = 0;
                    }
                }
            }
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
    ESP_LOGE("AutoPilot", "Request sent!");
    
 
}

void ReadGPS(uint32_t *lat, uint32_t *lon, uint32_t *alt, uint16_t *velx, uint16_t *vely, uint16_t *velz, uint32_t *time) {
    size_t available_bytes = 0;
    uart_get_buffered_data_len(CONFIG_UART_PORT_NUM, &available_bytes);
    uint16_t x = 0, y = 0, z = 0;
    uint32_t t = 0, la = 0, lo = 0, al = 0;
    if (available_bytes > 0) {
        int len = uart_read_bytes(CONFIG_UART_PORT_NUM, tx_buf, available_bytes, 20 / portTICK_RATE_MS);
        if (len > 0) {
            mavlink_message_t msg;
            mavlink_status_t status1;
            for (int i = 0; i < len; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_1, tx_buf[i], &msg, &status1)) {
                    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
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
}

// Callback when data is received via ESP-NOW
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(gps_message)) {
        gps_message receivedData;
        memcpy(&receivedData, incomingData, sizeof(receivedData));

        // Add or update the device data in the table
        addOrUpdateDevice(mac, &receivedData);

        // Print the updated device table
        printDeviceTable();
    } else {
        ESP_LOGE(TAG, "Invalid data length received");
    }
}


void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Message Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
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
    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_max_tx_power(40);  // Max transmission power for longer range

    esp_err_t init_result = esp_now_init();
    if (init_result != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW Initialization failed");
        return;
    }
    ESP_LOGE(TAG, "ESP-NOW Initialized successfully");

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    esp_now_del_peer(NULL);

    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add broadcast peer");
        return;
    }
    ESP_LOGE(TAG, "Broadcast peer added successfully");
}

void loop() {
    // Read GPS data
    ReadGPS(&latitude, &longitude, &altitude, &vel_x, &vel_y, &vel_z, &time_boot_ms);

    
    // Check if GPS data is valid
    if (latitude != 0 && longitude != 0 && time_boot_ms != 0) {
        myData.lat = latitude;
        myData.lon = longitude;
        myData.alt = altitude;
        myData.vx = vel_x;
        myData.vy = vel_y;
        myData.vz = vel_z;
        myData.time_boot_ms = time_boot_ms;
        myData.sysid = 2;
        
        // Send the data over ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "GPS message sent successfully via ESP-NOW");
        } else {
            ESP_LOGE(TAG, "Error sending GPS message");
        }
    } else {
        ESP_LOGI(TAG, "Waiting for valid GPS data...");
    }

    delay(100);  // Control sending rate (10 Hz)
}
