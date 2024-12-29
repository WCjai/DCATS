#include <Arduino.h>
#include "ardupilotmega/mavlink.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include <esp_now.h>
#include <WiFi.h>
#include <cmath>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
const double EARTH_RADIUS = 6371000.0; // Earth's radius in meters
#define MAX_DEVICES 10  // Maximum number of unique MAC addresses you want to track
#define SAFE_VELOCITY_LIMIT 200  // cm/s (2 meters/second)
#define TIMEOUT_PERIOD 10000  // Timeout period in milliseconds (10 seconds)
SemaphoreHandle_t xMutex;  // Mutex to protect shared data

/*uart config*/
#define BUFFER_SIZE 1024
int baudrate = 115200;
const uart_port_t CONFIG_UART_PORT_NUM = UART_NUM_2;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


/*Autopilot sent config*/

uint8_t component_id = MAV_COMP_ID_OBSTACLE_AVOIDANCE;       // ESP32 component ID
uint8_t received_sysid = 0;
uint8_t received_compid = 0;
uint8_t system_id = 2;        // ESP32 system ID

/*Array to hold mission items*/
typedef struct {
    uint16_t seq;     // Mission sequence number
    int32_t lat;      // Latitude
    int32_t lon;      // Longitude
    float alt;        // Altitude
} mission_item_t;
mission_item_t mission_items[100];
uint16_t mission_item_count = 0;   // stores recived total count

struct mission_current {
    uint16_t seq;
    uint16_t total;
};

mission_current mission;

/* DCAT */
struct Agent {
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    uint16_t hdg; // heading
    int32_t target_lat;
    int32_t target_lon;
    int32_t target_alt;
};
Agent myData;


//dcat 
//repulsive
const double REPULSION_ZONE_RADIUS = 50.0;   // Repulsion zone radius in meters
const double REPULSION_GAIN = 0.2; // Repulsion gain factor
const double ANISOTROPY_FACTOR = 2.0;  // Anisotropy parameter
const double A = 0.5;  // Anisotropy parameter
//friction
const double R = 25.0;                       // friction between drones
const double FRICTION_THRESHOLD_VELOCITY = 0.8; // Minimum friction velocity in m/s
const double FRICTION_GAIN = 0.6; // Friction gain factor
const double a = 0.15; // Acceleration limit

//self-driving
#define AVOID_RADIUS 1500  // cm (10 meters)
#define PLANNING_TIME 1.0  // seconds
const double MAX_SPEED = 4.0; // Max speed in m/s for self-drive velocity
const double MIN_SAFE_DISTANCE = 1000.0; // in cm (5 meters)



// Structure to hold MAC and associated GPS data
typedef struct device_data {
    uint8_t mac[6];  // MAC address (6 bytes)
    Agent gps_data;  // Associated GPS data
    uint32_t last_update_time;  // Timestamp of the last received data
} device_data;

device_data devices[MAX_DEVICES];  // Array to hold MAC and associated data
int device_count = 0;  // Current count of devices in the table

/*target handeller*/
uint16_t target_switch_index = 1;
const double target_switch_tolerance = 0.2;  // Example: 5 meters tolerance
bool is_first_target_set = false;



/*log*/
static const char *ESP_NOW = "COMM";
static const char *DCAT = "DCATS";
static const char *Vfri = "friction_velocity";
static const char *vrep = "repulsive_velocity";


// MAVLink message type for the companion computer
const uint8_t mav_type = MAV_TYPE_ONBOARD_CONTROLLER;   // Companion computer type
const uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;   // This is not an autopilot
const uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;  // No specific mode
const uint32_t custom_mode = 0;
const uint8_t system_status = MAV_STATE_ACTIVE;  // The system is active


/*----------Mavlink thing -----------*/

void sendHeartbeat(uint8_t target_sysid) {
    delay(2000);
    mavlink_message_t msghb;
    uint8_t bufhb[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(target_sysid, component_id, &msghb, mav_type, autopilot_type, base_mode, custom_mode, system_status);
    uint16_t lenhb = mavlink_msg_to_send_buffer(bufhb, &msghb);
    delay(1000);
    uart_write_bytes(CONFIG_UART_PORT_NUM, (const char *)bufhb, lenhb);
    ESP_LOGE("AutoPilot", "Heartbeat sent: system_id=%d, component_id=%d, length=%d", system_id, component_id, lenhb);
}


void listenHeartbeat() {
    delay (1000);
    bool flag = true;
    while (flag) {
        delay(1);
        size_t available_bytes = 0;
        uart_get_buffered_data_len(CONFIG_UART_PORT_NUM, &available_bytes);
   
        if (available_bytes > 0) {
            mavlink_message_t msg;
            mavlink_status_t status;
            uint8_t tx_buf[available_bytes];  
            int num_bytes = uart_read_bytes(CONFIG_UART_PORT_NUM, tx_buf, available_bytes, 20 / portTICK_RATE_MS);
          
            // Loop through each byte and attempt to parse MAVLink messages
            for (int i = 0; i < num_bytes; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, tx_buf[i], &msg, &status)) {
                    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        received_sysid = msg.sysid;
                        received_compid = msg.compid;
                        ESP_LOGE("AutoPilot", "Received heartbeat: sysid=%d, compid=%d\n", received_sysid, received_compid);
                        flag = false;  
                        break;  
                    }
                }
            }
        }
    }
}

void streamRequest(uint8_t target_sysid, uint8_t target_compid) {
    ESP_LOGE("AutoPilot","Now sending request for data stream...");
    delay(2000);
    mavlink_message_t msgds;
    uint8_t bufds[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_request_data_stream_pack(system_id, component_id, &msgds, target_sysid, target_compid, MAV_DATA_STREAM_ALL , 0x05, 1);
    uint16_t lends = mavlink_msg_to_send_buffer(bufds, &msgds);
    delay(1000);
    uart_write_bytes(CONFIG_UART_PORT_NUM, (const char *)bufds, lends);
    ESP_LOGE("AutoPilot", "Request sent!");
}



void sendVelocityCommand(uint8_t target_sysid, uint8_t target_compid, float vx, float vy, float vz) {
    mavlink_message_t msg;
    uint16_t type_mask = (1 << 0) | (1 << 1) | (1 << 2) |  // Ignore positions x, y, z
                         (1 << 3) | (1 << 4) | (1 << 5) |  // Ignore accelerations
                         (1 << 6) | (1 << 7);              // Ignore yaw and yaw rate

    mavlink_msg_set_position_target_local_ned_pack(
        system_id,                     // Sender system id
        component_id,                  // Sender component id
        &msg,                          // Message
        millis(),                      // Timestamp (in milliseconds since boot)
        target_sysid,                  // Target system ID (drone's sysid)
        target_compid,                 // Target component ID (drone's compid)
        MAV_FRAME_LOCAL_NED,           // Coordinate frame (local NED)
        4039,                     // Type mask to ignore position, yaw, etc.
        0, 0, 0,                       // Position x, y, z (ignored)
        vx, vy, vz,                    // Velocity x, y, z (NED frame)
        0, 0, 0,                       // Acceleration x, y, z (ignored)
        0,                             // Yaw (ignored)
        0                              // Yaw rate (ignored)
    );

    // Serialize the message and send it
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    uart_write_bytes(CONFIG_UART_PORT_NUM, (const char *)buf, len);
}


void sendTrajectoryVelocityCommand(float vx, float vy, float vz) {
    mavlink_trajectory_representation_waypoints_t trajectory_msg;

    // Set the timestamp (in microseconds since system boot)
    trajectory_msg.time_usec = micros();  

    // Initialize all other fields to NaN, since we are only using velocity
    for (int i = 0; i < 5; i++) {
        trajectory_msg.pos_x[i] = NAN;
        trajectory_msg.pos_y[i] = NAN;
        trajectory_msg.pos_z[i] = NAN;
        trajectory_msg.vel_x[i] = NAN;
        trajectory_msg.vel_y[i] = NAN;
        trajectory_msg.vel_z[i] = NAN;
        trajectory_msg.acc_x[i] = NAN;
        trajectory_msg.acc_y[i] = NAN;
        trajectory_msg.acc_z[i] = NAN;
        trajectory_msg.pos_yaw[i] = NAN;
        trajectory_msg.vel_yaw[i] = NAN;
        trajectory_msg.command[i] = 0;  // No MAV_CMD command
    }

    // Set velocities for the first waypoint
    trajectory_msg.vel_x[0] = vx;  // X velocity (north)
    trajectory_msg.vel_y[0] = vy;  // Y velocity (east)
    trajectory_msg.vel_z[0] = vz;  // Z velocity (down)
    trajectory_msg.command[0] = UINT16_MAX;

    // Pack the MAVLink message
    mavlink_message_t msg;
    mavlink_msg_trajectory_representation_waypoints_pack(
        received_sysid,                    // Sender system id
        component_id,                 // Sender component id
        &msg,                         // MAVLink message
        trajectory_msg.time_usec,     // Timestamp
        trajectory_msg.valid_points,  // Number of valid waypoints
        trajectory_msg.pos_x,         // X positions (ignored)
        trajectory_msg.pos_y,         // Y positions (ignored)
        trajectory_msg.pos_z,         // Z positions (ignored)
        trajectory_msg.vel_x,         // X velocities
        trajectory_msg.vel_y,         // Y velocities
        trajectory_msg.vel_z,         // Z velocities
        trajectory_msg.acc_x,         // X accelerations (ignored)
        trajectory_msg.acc_y,         // Y accelerations (ignored)
        trajectory_msg.acc_z,         // Z accelerations (ignored)
        trajectory_msg.pos_yaw,       // Yaw positions (ignored)
        trajectory_msg.vel_yaw,       // Yaw velocities (ignored)
        trajectory_msg.command        // Command (ignored)
    );

    // Serialize the message and send it
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    uart_write_bytes(CONFIG_UART_PORT_NUM, (const char*)buf, len);
}


void handleGlobalPosInt (mavlink_message_t* msg) {
    mavlink_global_position_int_t data;
    mavlink_msg_global_position_int_decode(msg, &data);
    myData.lat = data.lat;
    myData.lon = data.lon;
    myData.alt = data.alt;
    myData.hdg = data.hdg;
    myData.vx  = data.vx;
    myData.vy  = data.vy;
    myData.vz  = data.vz;                    
}

void handleTargetPosInt (mavlink_message_t* msg) {
    mavlink_position_target_global_int_t target_data;
    mavlink_msg_position_target_global_int_decode(msg, &target_data);
    myData.target_lat = target_data.lat_int;
    myData.target_lon = target_data.lon_int;
    myData.target_alt = (int32_t)(target_data.alt * 1000); 
    ESP_LOGE("DCATS", "Target: %d lat, %d lon, %d alt ", myData.target_lat, myData.target_lon, myData.target_alt);                  
}

void handleMissionCurrent (mavlink_message_t* msg) {
    mavlink_mission_current_t data;
    mavlink_msg_mission_current_decode(msg, &data);
    mission.seq = data.seq;
    mission.total = data.total;               
}

void enableMessageInterval(uint32_t interval_us, uint8_t target_sysid, uint8_t target_compid) {
    mavlink_command_long_t cmd;

    // Set target system and component ID (replace these with actual values)
    cmd.target_system = target_sysid;      // Target system ID
    cmd.target_component = target_compid;  // Target component ID

    // Set the command to MAV_CMD_SET_MESSAGE_INTERVAL
    cmd.command = MAV_CMD_SET_MESSAGE_INTERVAL;

    // Param 1: Set the message ID to the desired message (TRAJECTORY_REPRESENTATION_WAYPOINTS is 332)
    cmd.param1 = MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS;

    // Param 2: Set the interval in microseconds
    cmd.param2 = interval_us;

    // Param 3 to Param 7: Unused (set to 0 or NaN)
    cmd.param3 = 0;
    cmd.param4 = 0;
    cmd.param5 = 0;
    cmd.param6 = 0;
    cmd.param7 = 0;

    // Pack the MAVLink command message
    mavlink_message_t msg;
    mavlink_msg_command_long_encode(system_id, component_id, &msg, &cmd);

    // Serialize the message and send it
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    uart_write_bytes(CONFIG_UART_PORT_NUM, (const char*)buf, len);
}

/* Mavlink thing end below starts capturing mavlink*/
void readMavlinkMessages(void *pvParameters) {
    while (true) {  // Task loop
        size_t available_bytes = 0;
        uart_get_buffered_data_len(CONFIG_UART_PORT_NUM, &available_bytes);

        if (available_bytes > 0) {
            mavlink_message_t msg;
            mavlink_status_t status;
            uint8_t tx_buf[available_bytes]; 

            // Read the available bytes from UART
            int num_bytes = uart_read_bytes(CONFIG_UART_PORT_NUM, tx_buf, available_bytes, 20 / portTICK_RATE_MS);
            
            // Check if bytes were successfully read
            if (num_bytes > 0) {
                // Loop through the received bytes and parse MAVLink messages
                for (int i = 0; i < num_bytes; i++) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, tx_buf[i], &msg, &status)) {
                        // Handle the parsed message based on its ID
                        switch (msg.msgid) {
                            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                                handleGlobalPosInt(&msg);
                                break;
                            case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
                                handleTargetPosInt(&msg);
                                break;
                            case MAVLINK_MSG_ID_MISSION_CURRENT:
                                handleMissionCurrent(&msg);
                                break;
                            default:
                                ESP_LOGI("AutoPilot", "Unhandled MAVLink message ID: %d", msg.msgid);
                                break;
                        }
                    }
                }
            } else {
                ESP_LOGE("UART", "Error reading UART bytes, error code: %d", num_bytes);
            }
        } else {
            ESP_LOGI("UART", "No data available in UART buffer");
        }

        // Task delay to yield CPU and avoid busy-waiting
       
        vTaskDelay(pdMS_TO_TICKS(1));  // Adjust delay as necessary
    }
}


/*----------Target handeller---------*/

double toRadians(int32_t degE7) {
    return degE7 * 1e-7 * M_PI / 180.0;
}


// Function to calculate distance between two GPS points in 3D
double calculateDistance3D(int32_t lat1, int32_t lon1, int32_t alt1, int32_t lat2, int32_t lon2, int32_t alt2) {
    // Convert latitudes and longitudes to radians
    double lat1Rad = toRadians(lat1);
    double lon1Rad = toRadians(lon1);
    double lat2Rad = toRadians(lat2);
    double lon2Rad = toRadians(lon2);

    // Haversine formula for 2D distance
    double dLat = lat2Rad - lat1Rad;
    double dLon = lon2Rad - lon1Rad;

    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1Rad) * cos(lat2Rad) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    // 2D distance (horizontal) in meters
    double distance2D = EARTH_RADIUS * c;

    // Altitude difference in meters
    double altDiff = (alt2 - alt1) / 1000.0;  // Convert mm to meters

    // Total distance in 3D
    return sqrt(distance2D * distance2D + altDiff * altDiff);
}



/*----------communication AND storage---------*/

int findDeviceIndex(const uint8_t *mac) {
    for (int i = 0; i < device_count; i++) {
        if (memcmp(devices[i].mac, mac, 6) == 0) {
            return i;  // Return index if MAC address is found
        }
    }
    return -1;  // MAC not found
}

// Function to add or update device information
void addOrUpdateDevice(const uint8_t *mac, const Agent *data, uint32_t current_time) {
    int index = findDeviceIndex(mac);  // Find the device by MAC

    if (index != -1) {
        // MAC address found, update the existing entry
        devices[index].gps_data = *data;
        devices[index].last_update_time = current_time;
    } else {
        // MAC address not found, add new entry if space is available
        if (device_count < MAX_DEVICES) {
            memcpy(devices[device_count].mac, mac, 6);  // Copy MAC address
            devices[device_count].gps_data = *data;     // Copy GPS data
            devices[device_count].last_update_time = current_time;
            device_count++;
        } else {
            ESP_LOGE(DCAT, "Device table is full. Can't add new device.");
        }
    }
}
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    //ESP_LOGE(ESP_NOW, "Message Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// ESP-NOW callback for receiving data
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(Agent)) {
        Agent receivedData;
        memcpy(&receivedData, incomingData, sizeof(receivedData));

        uint32_t current_time = millis();  // Current time
        ESP_LOGE(ESP_NOW, "Received Data from %02X:%02X:%02X:%02X:%02X:%02X:", 
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        ESP_LOGE(ESP_NOW, "Lat: %d, Lon: %d, Alt: %d", receivedData.lat, receivedData.lon, receivedData.alt);

        // Protect access to shared data with mutex
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            // Add or update the device in the table
            ESP_LOGE("ESP_NOW", "Lat: %d, Lon: %d, Alt: %d", receivedData.lat, receivedData.lon, receivedData.alt);
            addOrUpdateDevice(mac, &receivedData, current_time);
            xSemaphoreGive(xMutex);  // Release the mutex
        }

    } else {
        ESP_LOGE(ESP_NOW, "Invalid data length received");
    }
}

void removeStaleDevices(uint32_t current_time) {
    // Protect access to the shared devices array using a mutex
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        for (int i = 0; i < device_count; i++) {
            // Check if the device is stale
            if (current_time - devices[i].last_update_time > TIMEOUT_PERIOD) {
                ESP_LOGE(DCAT, "Removing stale device: %02X:%02X:%02X:%02X:%02X:%02X",
                         devices[i].mac[0], devices[i].mac[1], devices[i].mac[2],
                         devices[i].mac[3], devices[i].mac[4], devices[i].mac[5]);
                
                // Shift the remaining devices up
                for (int j = i; j < device_count - 1; j++) {
                    devices[j] = devices[j + 1];
                }
                device_count--;
                i--;  // Adjust index after removal
            }
        }
        xSemaphoreGive(xMutex);  // Release the mutex after the operation
    }
}

// Convert device_data to Agent
Agent convertToAgent(const device_data& device) {
    return Agent{
        device.gps_data.lat,
        device.gps_data.lon,
        device.gps_data.alt,
        device.gps_data.vx,
        device.gps_data.vy,
        device.gps_data.vz,
        device.gps_data.hdg

    };
}

/*--------------DCATS--------------*/
void gpsToNED(const Agent& currentAgent, const Agent& otherAgent, double& x, double& y, double& z) {
    double lat_cur = currentAgent.lat / 1e7 * DEG_TO_RAD;
    double lon_cur = currentAgent.lon / 1e7 * DEG_TO_RAD;
    double alt_cur = currentAgent.alt / 1000.0; // Convert mm to meters

    double lat_other = otherAgent.lat / 1e7 * DEG_TO_RAD;
    double lon_other = otherAgent.lon / 1e7 * DEG_TO_RAD;
    double alt_other = otherAgent.alt / 1000.0; // Convert mm to meters

    // Compute NED coordinates relative to the current agent (North, East, Down)
    x = EARTH_RADIUS * (lat_other - lat_cur); // North (X) is latitude difference
    y = EARTH_RADIUS * cos(lat_cur) * (lon_other - lon_cur); // East (Y) is longitude difference
    z = alt_cur - alt_other; // Altitude difference (positive downwards)
}

double computeDistance(double x1, double y1, double z1, double x2, double y2, double z2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
}

void headingToUnitVector(uint16_t hdg, double& dx, double& dy) {
    double heading_rad = (hdg / 100.0) * DEG_TO_RAD; // Convert centidegrees to radians
    dx = cos(heading_rad);  // X component (North direction)
    dy = sin(heading_rad);  // Y component (East direction)
}



void computeRepulsiveVelocityWithHeading(const Agent& currentAgent, const std::vector<Agent>& neighbors, double& rep_vx, double& rep_vy, double& rep_vz) {
    rep_vx = 0.0;
    rep_vy = 0.0;
    rep_vz = 0.0;

    for (const auto& neighbor : neighbors) {
        // Convert positions to NED relative to the current agent
        double x_j, y_j, z_j;
        gpsToNED(currentAgent, neighbor, x_j, y_j, z_j); // Neighbor's NED position

        // Compute distance between current agent and neighbor
        double distance = computeDistance(0, 0, 0, x_j, y_j, z_j); // Current agent's NED position is always (0, 0, 0)
        
        // Apply repulsion if within the repulsion zone
        if (distance < REPULSION_ZONE_RADIUS && distance > 0) {
            double force_magnitude = REPULSION_GAIN * (REPULSION_ZONE_RADIUS - distance);

            // Normalize the direction vector (relative position)
            double dx = x_j / distance;
            double dy = y_j / distance;

            // Get unit vectors from headings of current agent and neighbor
            double agent_dx, agent_dy, neighbor_dx, neighbor_dy;
            headingToUnitVector(currentAgent.hdg, agent_dx, agent_dy);
            headingToUnitVector(neighbor.hdg, neighbor_dx, neighbor_dy);

            // Calculate angle φ (between relative position and current agent's target direction)
            double phi = acos(dx * agent_dx + dy * agent_dy); // Angle between position vector and target direction

            // Determine if agents are moving in the same or opposite directions
            double dot_product_velocities = agent_dx * neighbor_dx + agent_dy * neighbor_dy;
            bool same_direction = dot_product_velocities > cos(M_PI / 3); // Less than 60° apart

            // Calculate anisotropic adjustment angle ρ based on φ and direction similarity
            double rho;
            if (phi <= M_PI / 2 && same_direction) {
                rho = (1 - ANISOTROPY_FACTOR) * phi;
            } else if (phi > M_PI / 2 && same_direction) {
                rho = M_PI + (1 - ANISOTROPY_FACTOR) * (phi - M_PI);
            } else { // Opposite directions
                rho = (1 - ANISOTROPY_FACTOR / 2) * (phi - M_PI) + M_PI;
            }

            // Rotate the repulsion direction by ρ to get the anisotropic repulsion vector
            double adjusted_dx = cos(rho) * dx - sin(rho) * dy;
            double adjusted_dy = sin(rho) * dx + cos(rho) * dy;

            // Apply the repulsive force using the adjusted direction
            rep_vx += force_magnitude * adjusted_dx;
            rep_vy += force_magnitude * adjusted_dy;
        }
    }
}


void computeRepulsiveVelocityWithHeading_v2(const Agent& currentAgent, const std::vector<Agent>& neighbors, double& rep_vx, double& rep_vy, double& rep_vz) {
    rep_vx = 0.0;
    rep_vy = 0.0;
    rep_vz = 0.0;

    for (const auto& neighbor : neighbors) {
        // Convert positions to NED relative to the current agent
        double x_j, y_j, z_j;
        gpsToNED(currentAgent, neighbor, x_j, y_j, z_j); // Neighbor's NED position

        // Compute distance between current agent and neighbor
        double distance = computeDistance(0, 0, 0, x_j, y_j, z_j); // Current agent's NED position is always (0, 0, 0)
        
        // Apply repulsion if within the repulsion zone
        if (distance < REPULSION_ZONE_RADIUS && distance > 0) {
            double force_magnitude = REPULSION_GAIN * (REPULSION_ZONE_RADIUS - distance);

            // Normalize the direction vector
            double dx = (0 - x_j) / distance;
            double dy = (0 - y_j) / distance;
            
            // Adjust based on heading
            double agent_dx, agent_dy, neighbor_dx, neighbor_dy;
            headingToUnitVector(currentAgent.hdg, agent_dx, agent_dy);
            headingToUnitVector(neighbor.hdg, neighbor_dx, neighbor_dy);
            
            // Compute phi: angle between r_ij (relative position) and current agent's direction
            double phi = acos(dx * agent_dx + dy * agent_dy);

            // Anisotropic repulsion adjustment (rho)
            double rho;
            if (phi <= M_PI / 2 && neighbor_dx * agent_dx + neighbor_dy * agent_dy > cos(M_PI / 3)) { // Parallel directions
                rho = (1 - A) * phi;
            } else if (phi > M_PI / 2 && neighbor_dx * agent_dx + neighbor_dy * agent_dy > cos(M_PI / 3)) { // Parallel but behind
                rho = M_PI + (1 - A) * (phi - M_PI);
            } else {  // Opposite directions
                rho = (1 - A / 2) * (phi - M_PI) + M_PI;
            }
            
            // Apply repulsive force using rho adjustment
            double heading_factor = cos(rho);  // Use cosine of rho as an adjustment

            rep_vx += force_magnitude * heading_factor * dx;
            rep_vy += force_magnitude * heading_factor * dy;
        }
    }
}

void computeRepulsiveVelocityWithHeading_v3(const Agent& currentAgent, const std::vector<Agent>& neighbors, double& rep_vx, double& rep_vy, double& rep_vz) {
    rep_vx = 0.0;
    rep_vy = 0.0;
    rep_vz = 0.0;

    for (const auto& neighbor : neighbors) {
        // Convert positions to NED relative to the current agent
        double x_j, y_j, z_j;
        gpsToNED(currentAgent, neighbor, x_j, y_j, z_j); // Neighbor's NED position

        // Compute distance between current agent and neighbor
        double distance = computeDistance(0, 0, 0, x_j, y_j, z_j); // Current agent's NED position is always (0, 0, 0)
        
        // Apply repulsion if within the repulsion zone
        if (distance < REPULSION_ZONE_RADIUS && distance > 0) {
            // Nonlinear decay for force magnitude, with additional scaling if below MIN_SAFE_DISTANCE
            double force_magnitude = REPULSION_GAIN * pow((REPULSION_ZONE_RADIUS - distance) / REPULSION_ZONE_RADIUS, 2);
            if (distance < MIN_SAFE_DISTANCE) {
                force_magnitude *= 1.5; // Increase force when too close to the obstacle
            }

            // Normalize the direction vector
            double dx = (0 - x_j) / distance;
            double dy = (0 - y_j) / distance;
            
            // Adjust based on heading
            double agent_dx, agent_dy, neighbor_dx, neighbor_dy;
            headingToUnitVector(currentAgent.hdg, agent_dx, agent_dy);
            headingToUnitVector(neighbor.hdg, neighbor_dx, neighbor_dy);
            
            // Compute phi: angle between r_ij (relative position) and current agent's direction
            double phi = acos(dx * agent_dx + dy * agent_dy);

            // Anisotropic repulsion adjustment (rho)
            double rho;
            if (phi <= M_PI / 2 && neighbor_dx * agent_dx + neighbor_dy * agent_dy > cos(M_PI / 3)) { // Parallel directions
                rho = (1 - A) * phi;
            } else if (phi > M_PI / 2 && neighbor_dx * agent_dx + neighbor_dy * agent_dy > cos(M_PI / 3)) { // Parallel but behind
                rho = M_PI + (1 - A) * (phi - M_PI);
            } else {  // Opposite directions
                rho = (1 - A / 2) * (phi - M_PI) + M_PI;
            }
            
            // Apply repulsive force using rho adjustment
            double heading_factor = cos(rho);  // Use cosine of rho as an adjustment

            // Calculate final repulsive velocity components
            rep_vx += force_magnitude * heading_factor * dx;
            rep_vy += force_magnitude * heading_factor * dy;
        }
    }
}



double computeRelativeVelocity(const Agent& agent1, const Agent& agent2) {
    // Calculate the relative velocity (in m/s)
    double vx_diff = (agent1.vx - agent2.vx) / 100.0; // Convert cm/s to m/s
    double vy_diff = (agent1.vy - agent2.vy) / 100.0;
    double vz_diff = (agent1.vz - agent2.vz) / 100.0;
    
    return sqrt(vx_diff * vx_diff + vy_diff * vy_diff + vz_diff * vz_diff);
}

double computeBrakingFunction1(double distance, double rho, double R, double a) {
    double d_offset = distance - R;
    if (d_offset < 0) {
        return 0;
    } else if (d_offset <= a / (rho * rho)) {
        return rho * d_offset;
    } else {
        return sqrt(2 * a * d_offset) - a / rho;
    }
}

void computeFrictionVelocity_v1(const Agent& currentAgent, const std::vector<Agent>& neighbors, double rho, double& friction_vx, double& friction_vy, double& friction_vz) { 
    friction_vx = 0.0;
    friction_vy = 0.0;
    friction_vz = 0.0;

    for (const auto& neighbor : neighbors) {
        // Convert positions to NED relative to the current agent
        double x_j, y_j, z_j;
        gpsToNED(currentAgent, neighbor, x_j, y_j, z_j); // Neighbor's NED position

        // Compute distance between current agent and neighbor
        double distance = computeDistance(0, 0, 0, x_j, y_j, z_j); // Current agent's NED position is always (0, 0, 0)
        //ESP_LOGE(TAG, "Distance between agents: %.2f m", distance);

        // Only apply friction if the distance is less than R
        if (distance < R) {
            // Compute relative velocity
            double v_ij = computeRelativeVelocity(currentAgent, neighbor);

            // Compute friction threshold velocity
            double v_friction_threshold = std::max(FRICTION_THRESHOLD_VELOCITY, computeBrakingFunction1(distance, rho, R, a));
            ESP_LOGE(Vfri, "Relative Velocity: %.2f m/s, Friction Threshold: %.2f m/s", v_ij, v_friction_threshold);

            // If relative velocity exceeds the threshold, apply friction
            if (v_ij > v_friction_threshold) {
                //ESP_LOGE(Vfri, "Applying friction force between Agent 1 and Neighbor: %.2f m/s^2", v_ij);

                double friction_force = FRICTION_GAIN * (v_ij - v_friction_threshold);

                // Normalize the relative velocity vector and apply friction
                double vx_diff = (currentAgent.vx - neighbor.vx) / 100.0; // Convert to m/s
                double vy_diff = (currentAgent.vy - neighbor.vy) / 100.0;
                double vz_diff = (currentAgent.vz - neighbor.vz) / 100.0;
                double velocity_magnitude = sqrt(vx_diff * vx_diff + vy_diff * vy_diff + vz_diff * vz_diff);

                //ESP_LOGE(Vfri, "Velocity Differences - X: %.2f, Y: %.2f, Z: %.2f", vx_diff, vy_diff, vz_diff);
                //ESP_LOGE(Vfri, "Velocity Magnitude: %.2f m/s", velocity_magnitude);

                if (velocity_magnitude > 0) {
                    friction_vx += friction_force * (vx_diff / velocity_magnitude);
                    friction_vy += friction_force * (vy_diff / velocity_magnitude);
                    friction_vz += friction_force * (vz_diff / velocity_magnitude);
                }
            }
        }
    }
}
bool shouldApplyFriction(const Agent& currentAgent, const Agent& neighbor) {
    // Compute angular conditions (theta, etc.)
    // These are the checks described in the paper:
    // - Is the neighbor coming toward the agent?
    // - Is the neighbor between the agent and its target?
    // - Would aligning velocities help or hinder?

    // For simplicity, let’s assume we’re checking if the neighbor is coming toward the agent.
    // Compute direction vectors
    double agent_heading_x, agent_heading_y;
    headingToUnitVector(currentAgent.hdg, agent_heading_x, agent_heading_y);

    double rel_x = neighbor.lon - currentAgent.lon;
    double rel_y = neighbor.lat - currentAgent.lat;
    double rel_distance = sqrt(rel_x * rel_x + rel_y * rel_y);

    // Compute angle between the current agent’s heading and the relative position
    double angle = acos((agent_heading_x * rel_x + agent_heading_y * rel_y) / rel_distance);

    // Check if the angle is within the allowed range
    return angle <= M_PI / 4; // Using the +/- pi/4 threshold for "coming towards" check
}

double computeFrictionMaxVelocity(double distance) {
    double p_friction = 0.6;  // Example gain
    double a_friction = 0.15;  // Acceleration limit
    double R_friction = 25.0;  // Distance threshold for friction
    double v_friction = 0.8;  // Minimum friction velocity

    if (distance < 0) return 0;

    if (distance <= R_friction) {
        return p_friction * (distance - R_friction);
    } else if (distance > R_friction && distance < (a_friction / (p_friction * p_friction))) {
        return sqrt(2 * a_friction * (distance - R_friction) - (a_friction * a_friction) / (p_friction * p_friction));
    } else {
        return v_friction;
    }
}


void computeFrictionVelocity_v2(const Agent& currentAgent, const std::vector<Agent>& neighbors, double& friction_vx, double& friction_vy, double& friction_vz) {
    friction_vx = 0.0;
    friction_vy = 0.0;
    friction_vz = 0.0;

    for (const auto& neighbor : neighbors) {
        // Convert positions to NED relative to the current agent
        double x_j, y_j, z_j;
        gpsToNED(currentAgent, neighbor, x_j, y_j, z_j); // Neighbor's NED position

        // Compute distance between current agent and neighbor
        double distance = computeDistance(0, 0, 0, x_j, y_j, z_j); // Current agent's NED position is always (0, 0, 0)

        // Compute relative velocity between current agent and neighbor
        double vx_rel = currentAgent.vx - neighbor.vx;
        double vy_rel = currentAgent.vy - neighbor.vy;
        double vz_rel = currentAgent.vz - neighbor.vz;
        double v_rel_mag = sqrt(vx_rel * vx_rel + vy_rel * vy_rel + vz_rel * vz_rel);

        // Check friction conditions
        // 1. Neighbor is coming towards current agent
        // 2. Neighbor lies between current agent and target
        // 3. Alignment check
        if (shouldApplyFriction(currentAgent, neighbor)) {
            // Compute friction threshold (using braking curve)
            double v_frictmax = computeFrictionMaxVelocity(distance);
            
            // Apply friction if relative velocity is greater than the threshold
            if (v_rel_mag > v_frictmax) {
                double friction_force = v_rel_mag - v_frictmax;

                // Normalize the direction vector of the relative velocity
                double vx_dir = vx_rel / v_rel_mag;
                double vy_dir = vy_rel / v_rel_mag;
                double vz_dir = vz_rel / v_rel_mag;

                // Apply friction in the opposite direction of the relative velocity
                friction_vx += friction_force * vx_dir;
                friction_vy += friction_force * vy_dir;
                friction_vz += friction_force * vz_dir;
            }
        }
    }
}

// Helper function for braking function D(d, R, p, a)
double computeBrakingFunction(double d, double R, double p, double a) {
    if (d - R < 0) return 0;
    else if (d - R <= a / (p * p)) return p * (d - R);
    else return sqrt(2 * a * (d - R) - a * a / (p * p));
}

void computeFrictionVelocity(const Agent& currentAgent, const std::vector<Agent>& neighbors, double& friction_vx, double& friction_vy, double& friction_vz) {
    friction_vx = 0.0;
    friction_vy = 0.0;
    friction_vz = 0.0;

    const double friction_gain = 2.0;         // Linear gain for friction
    const double friction_max_distance = 50.0; // Maximum distance for applying friction (in meters)
    const double max_deceleration = 1.0;      // Maximum deceleration (m/s^2)
    const double r_friction = 30.0;            // Distance threshold for braking function (meters)

    for (const Agent& neighbor : neighbors) {
        // Convert positions to NED relative to the current agent
        double x_neigh, y_neigh, z_neigh;
        gpsToNED(currentAgent, neighbor, x_neigh, y_neigh, z_neigh);

        // Calculate relative position and velocity
        double d_rel = sqrt(x_neigh * x_neigh + y_neigh * y_neigh + z_neigh * z_neigh);
        if (d_rel > friction_max_distance) continue;  // Skip neighbors beyond friction range

        double vx_rel = neighbor.vx / 100.0 - currentAgent.vx / 100.0; // Convert cm/s to m/s
        double vy_rel = neighbor.vy / 100.0 - currentAgent.vy / 100.0;
        double vz_rel = neighbor.vz / 100.0 - currentAgent.vz / 100.0;

        double v_rel = sqrt(vx_rel * vx_rel + vy_rel * vy_rel + vz_rel * vz_rel);
        double v_friction_max = std::max(v_rel, computeBrakingFunction(d_rel, r_friction, friction_gain, max_deceleration));

        // Condition checking for applying friction based on the three conditions
        bool apply_friction = false;

        // Condition 1: Neighbor coming towards the agent (angle between r_ij and v_j within ±π/4)
        double dx = x_neigh / d_rel;
        double dy = y_neigh / d_rel;
        double neighbor_v_magnitude = sqrt(neighbor.vx * neighbor.vx + neighbor.vy * neighbor.vy + neighbor.vz * neighbor.vz);
        double vx_unit = neighbor.vx / neighbor_v_magnitude;
        double vy_unit = neighbor.vy / neighbor_v_magnitude;
        double angle1 = acos(dx * vx_unit + dy * vy_unit);
        if (angle1 <= M_PI / 4) apply_friction = true;

        // Condition 2: Neighbor lies between agent and target (angle between r_ij and agent’s target within ±2π/3)
        double tx, ty, tz;
        gpsToNED(currentAgent, {currentAgent.target_lat, currentAgent.target_lon, currentAgent.target_alt, 0, 0, 0, 0}, tx, ty, tz);
        double target_magnitude = sqrt(tx * tx + ty * ty + tz * tz);
        double tx_unit = tx / target_magnitude;
        double ty_unit = ty / target_magnitude;
        double angle2 = acos(dx * tx_unit + dy * ty_unit);
        if (angle2 <= 2 * M_PI / 3) apply_friction = true;

        // Condition 3: Neighbor’s velocity would drive agent away from its goal (angle between v_j and target direction within ±π/2)
        double angle3 = acos(vx_unit * tx_unit + vy_unit * ty_unit);
        if (angle3 <= M_PI / 2) apply_friction = true;

        // Apply friction adjustment if any condition is satisfied
        if (apply_friction) {
            double friction_magnitude = (v_rel - v_friction_max);
            if (friction_magnitude > 0) {
                double friction_factor = friction_magnitude / v_rel;
                friction_vx += friction_factor * vx_rel;
                friction_vy += friction_factor * vy_rel;
                friction_vz += friction_factor * vz_rel;
            }
        }
    }
}



bool checkDangerCriteria(const Agent& currentAgent, const Agent& neighbor, double& x_j, double& y_j, double& z_j) {
    // Convert positions to NED relative to the current agent
    gpsToNED(currentAgent, neighbor, x_j, y_j, z_j); // Neighbor's NED position
    double distance = computeDistance(0, 0, 0, x_j, y_j, z_j); // Distance between current agent and neighbor

    // D1: Are they on a collision course based on relative velocity?
    double rel_vx, rel_vy, rel_vz;
    rel_vx = (currentAgent.vx - neighbor.vx) / 100.0; // cm/s to m/s
    rel_vy = (currentAgent.vy - neighbor.vy) / 100.0;
    rel_vz = (currentAgent.vz - neighbor.vz) / 100.0;
    double relative_velocity = sqrt(rel_vx * rel_vx + rel_vy * rel_vy + rel_vz * rel_vz);
    
    // D2: Ensure relative velocity is within safe limit (handleable by drone's acceleration)
    if (relative_velocity > MAX_SPEED) return true;

    // D3: Ensure that the current agent is partially responsible (moving toward the neighbor)
    if (rel_vx * x_j + rel_vy * y_j + rel_vz * z_j > 0) return true;

    // D4: Time to collision is dangerously close
    if (distance / relative_velocity < 5.0) return true; // Example threshold of 5 seconds for collision

    return false;
}

/*triggers self-driving when neighbours hits radius*/


void computeSelfDriveVelocity_v1(const Agent& currentAgent, const std::vector<Agent>& neighbors, double& self_drive_vx, double& self_drive_vy, double& self_drive_vz) {
    double x_target, y_target, z_target;

    // Convert target position to NED relative to the current position
    Agent targetAgent = {currentAgent.target_lat, currentAgent.target_lon, currentAgent.target_alt, 0, 0, 0, 0};
    gpsToNED(currentAgent, targetAgent, x_target, y_target, z_target);

    // Compute the direction toward the target
    double distance_to_target = sqrt(x_target * x_target + y_target * y_target + z_target * z_target);

    // Normalize direction and set the velocity toward the target (with max speed)
    if (distance_to_target > 0) {
        self_drive_vx = (x_target / distance_to_target) * MAX_SPEED;
        self_drive_vy = (y_target / distance_to_target) * MAX_SPEED;
        self_drive_vz = (z_target / distance_to_target) * MAX_SPEED;
    } else {
        self_drive_vx = 0.0;
        self_drive_vy = 0.0;
        self_drive_vz = 0.0;
    }

    // Now, integrate obstacle avoidance with MIN_SAFE_DISTANCE
    for (const Agent& neighbor : neighbors) {
        double x_neigh, y_neigh, z_neigh;
        gpsToNED(currentAgent, neighbor, x_neigh, y_neigh, z_neigh);  // Convert neighbor's position to NED

        // Relative velocity
        double vx_rel = currentAgent.vx - neighbor.vx;
        double vy_rel = currentAgent.vy - neighbor.vy;
        double vz_rel = currentAgent.vz - neighbor.vz;

        // Relative position
        double d_rel = sqrt(x_neigh * x_neigh + y_neigh * y_neigh + z_neigh * z_neigh);

        // D1: Check if the neighbor is within the avoidance radius and the relative velocity would lead to a collision
        if (d_rel < AVOID_RADIUS) {
            double theta = asin(fmin(1.0, AVOID_RADIUS / d_rel));
            if (vx_rel * x_neigh + vy_rel * y_neigh + vz_rel * z_neigh < cos(theta) * d_rel) {
                // D2: Check if relative velocity is too large for safe avoidance
                if (vx_rel * x_neigh + vy_rel * y_neigh + vz_rel * z_neigh > SAFE_VELOCITY_LIMIT) {
                    // D3: The current agent is moving toward the neighbor
                    if (vx_rel * x_neigh + vy_rel * y_neigh + vz_rel * z_neigh > 0) {
                        // D4: The time to closest approach is shorter than the planned time
                        double t_approach = d_rel / sqrt(vx_rel * vx_rel + vy_rel * vy_rel + vz_rel * vz_rel);
                        if (t_approach < PLANNING_TIME) {
                            // Collision threat detected, apply avoidance by adjusting the velocity

                            // Use a stronger adjustment factor if below the MIN_SAFE_DISTANCE
                            //double adjustment_factor = (d_rel < MIN_SAFE_DISTANCE) ? 0.5 : 0.2;
                            double adjustment_factor = 0.1;

                            // Apply the avoidance adjustment
                            self_drive_vx -= adjustment_factor * x_neigh;
                            self_drive_vy -= adjustment_factor * y_neigh;
                            self_drive_vz -= adjustment_factor * z_neigh;
                        }
                    }
                }
            }
        }
    }

}

void computeSelfDriveVelocity_v2(const Agent& currentAgent, const std::vector<Agent>& neighbors, double& self_drive_vx, double& self_drive_vy, double& self_drive_vz) {
    double x_target, y_target, z_target;

    // Convert target position to NED relative to the current position
    Agent targetAgent = {currentAgent.target_lat, currentAgent.target_lon, currentAgent.target_alt, 0, 0, 0, 0};
    gpsToNED(currentAgent, targetAgent, x_target, y_target, z_target);

    // Compute the direction toward the target
    double distance_to_target = sqrt(x_target * x_target + y_target * y_target + z_target * z_target);

    // Initial velocity towards the target
    if (distance_to_target > 0) {
        self_drive_vx = (x_target / distance_to_target) * MAX_SPEED;
        self_drive_vy = (y_target / distance_to_target) * MAX_SPEED;
        self_drive_vz = (z_target / distance_to_target) * MAX_SPEED;
    } else {
        self_drive_vx = 0.0;
        self_drive_vy = 0.0;
        self_drive_vz = 0.0;
    }

    // Iterative refinement of self-driving velocity to avoid threats
    bool threat_detected;
    const int max_iterations = 10;
    int iteration_count = 0;

    do {
        threat_detected = false;
        iteration_count++;

        for (const Agent& neighbor : neighbors) {
            double x_neigh, y_neigh, z_neigh;
            gpsToNED(currentAgent, neighbor, x_neigh, y_neigh, z_neigh);

            // Relative velocity
            double vx_rel = self_drive_vx - neighbor.vx / 100.0;
            double vy_rel = self_drive_vy - neighbor.vy / 100.0;
            double vz_rel = self_drive_vz - neighbor.vz / 100.0;

            // Relative position
            double d_rel = sqrt(x_neigh * x_neigh + y_neigh * y_neigh + z_neigh * z_neigh);

            // D1-D4 Danger Criteria Checks
            if (d_rel < AVOID_RADIUS) {
                double theta = asin(fmin(1.0, AVOID_RADIUS / d_rel));
                if (vx_rel * x_neigh + vy_rel * y_neigh + vz_rel * z_neigh < cos(theta) * d_rel) {
                    if (vx_rel * x_neigh + vy_rel * y_neigh + vz_rel * z_neigh > SAFE_VELOCITY_LIMIT) {
                        if (vx_rel * x_neigh + vy_rel * y_neigh + vz_rel * z_neigh > 0) {
                            double t_approach = d_rel / sqrt(vx_rel * vx_rel + vy_rel * vy_rel + vz_rel * vz_rel);
                            if (t_approach < PLANNING_TIME) {
                                // Collision threat detected, apply avoidance adjustments iteratively
                                threat_detected = true;

                                // Geometric Criteria C1-C5 Adjustment
                                // C1: Ensure no collision course
                                double adjustment_factor = 0.1; // Adjust iteratively based on threat
                                if (iteration_count > 5) adjustment_factor = 0.05;

                                double new_vx = self_drive_vx - adjustment_factor * x_neigh;
                                double new_vy = self_drive_vy - adjustment_factor * y_neigh;
                                double new_vz = self_drive_vz - adjustment_factor * z_neigh;

                                // C2: Keep on the same side to avoid oscillation
                                double side_check = (new_vx * x_neigh + new_vy * y_neigh);
                                if (side_check < 0) { // Wrong side, reverse correction
                                    new_vx = -new_vx;
                                    new_vy = -new_vy;
                                }

                                // C3: Avoid speeding up in danger
                                double current_speed = sqrt(self_drive_vx * self_drive_vx + self_drive_vy * self_drive_vy + self_drive_vz * self_drive_vz);
                                double new_speed = sqrt(new_vx * new_vx + new_vy * new_vy + new_vz * new_vz);
                                if (new_speed > current_speed) {
                                    new_vx *= (current_speed / new_speed);
                                    new_vy *= (current_speed / new_speed);
                                    new_vz *= (current_speed / new_speed);
                                }

                                // C4: Ensure ability to stop
                                if (new_speed > SAFE_VELOCITY_LIMIT) {
                                    new_vx *= (SAFE_VELOCITY_LIMIT / new_speed);
                                    new_vy *= (SAFE_VELOCITY_LIMIT / new_speed);
                                    new_vz *= (SAFE_VELOCITY_LIMIT / new_speed);
                                }

                                // C5: Maximize alignment to original direction
                                double alignment_factor = ((x_target * new_vx) + (y_target * new_vy) + (z_target * new_vz)) / (distance_to_target * new_speed);
                                new_vx *= alignment_factor;
                                new_vy *= alignment_factor;
                                new_vz *= alignment_factor;

                                // Apply new velocity
                                self_drive_vx = new_vx;
                                self_drive_vy = new_vy;
                                self_drive_vz = new_vz;

                                break; // Exit loop to re-evaluate with updated velocity
                            }
                        }
                    }
                }
            }
        }
    } while (threat_detected && iteration_count < max_iterations);
}

void computeSelfDriveVelocity(const Agent& currentAgent, const std::vector<Agent>& neighbors, double& self_drive_vx, double& self_drive_vy, double& self_drive_vz) {
    double x_target, y_target, z_target;

    // Convert target position to NED relative to the current position
    Agent targetAgent = {currentAgent.target_lat, currentAgent.target_lon, currentAgent.target_alt, 0, 0, 0, 0};
    gpsToNED(currentAgent, targetAgent, x_target, y_target, z_target);

    // Initial velocity towards the target
    double distance_to_target = sqrt(x_target * x_target + y_target * y_target + z_target * z_target);
    if (distance_to_target > 0) {
        self_drive_vx = (x_target / distance_to_target) * MAX_SPEED;
        self_drive_vy = (y_target / distance_to_target) * MAX_SPEED;
        self_drive_vz = (z_target / distance_to_target) * MAX_SPEED;
    } else {
        self_drive_vx = 0.0;
        self_drive_vy = 0.0;
        self_drive_vz = 0.0;
    }

    bool threat_detected;
    const int max_iterations = 10;
    int iteration_count = 0;

    do {
        threat_detected = false;
        iteration_count++;

        for (const Agent& neighbor : neighbors) {
            double x_neigh, y_neigh, z_neigh;
            gpsToNED(currentAgent, neighbor, x_neigh, y_neigh, z_neigh);

            // Relative position and velocity
            double vx_rel = self_drive_vx - neighbor.vx / 100.0;
            double vy_rel = self_drive_vy - neighbor.vy / 100.0;
            double vz_rel = self_drive_vz - neighbor.vz / 100.0;
            double d_rel = sqrt(x_neigh * x_neigh + y_neigh * y_neigh + z_neigh * z_neigh);

            // Adjust SAFE_VELOCITY_LIMIT based on distance to obstacle
            double dynamic_safe_velocity = SAFE_VELOCITY_LIMIT * (d_rel / AVOID_RADIUS);
            
            // D1-D4 Danger Criteria
            if (d_rel < AVOID_RADIUS) {
                double theta = asin(fmin(1.0, AVOID_RADIUS / d_rel));
                if (vx_rel * x_neigh + vy_rel * y_neigh + vz_rel * z_neigh < cos(theta) * d_rel) {
                    if (vx_rel * x_neigh + vy_rel * y_neigh + vz_rel * z_neigh > dynamic_safe_velocity) {
                        if (vx_rel * x_neigh + vy_rel * y_neigh + vz_rel * z_neigh > 0) {
                            double t_approach = d_rel / sqrt(vx_rel * vx_rel + vy_rel * vy_rel + vz_rel * vz_rel);
                            if (t_approach < PLANNING_TIME) {
                                threat_detected = true;

                                // Adjust velocity based on C1-C5
                                double adjustment_factor = (iteration_count > 5) ? 0.05 : 0.1;
                                double new_vx = self_drive_vx - adjustment_factor * x_neigh;
                                double new_vy = self_drive_vy - adjustment_factor * y_neigh;
                                double new_vz = self_drive_vz - adjustment_factor * z_neigh;

                                // C2: Side consistency
                                double side_check = (new_vx * x_neigh + new_vy * y_neigh);
                                if (side_check < 0) {
                                    new_vx = -new_vx;
                                    new_vy = -new_vy;
                                }

                                // C3 & C4: Speed control
                                double current_speed = sqrt(self_drive_vx * self_drive_vx + self_drive_vy * self_drive_vy + self_drive_vz * self_drive_vz);
                                double new_speed = sqrt(new_vx * new_vx + new_vy * new_vy + new_vz * new_vz);
                                new_vx *= fmin(1.0, current_speed / new_speed);
                                new_vy *= fmin(1.0, current_speed / new_speed);
                                new_vz *= fmin(1.0, current_speed / new_speed);

                                // C5: Alignment to target
                                double alignment_factor = ((x_target * new_vx) + (y_target * new_vy) + (z_target * new_vz)) / (distance_to_target * new_speed);
                                new_vx *= alignment_factor;
                                new_vy *= alignment_factor;
                                new_vz *= alignment_factor;

                                // Apply updated velocity
                                self_drive_vx = new_vx;
                                self_drive_vy = new_vy;
                                self_drive_vz = new_vz;

                                break;
                            }
                        }
                    }
                }
            }
        }
    } while (threat_detected && iteration_count < max_iterations);
}


void computeTotalVelocity(Agent& currentAgent, const std::vector<Agent>& neighbors, double& total_vx, double& total_vy, double& total_vz) {
    double rep_vx, rep_vy, rep_vz;
    double friction_vx, friction_vy, friction_vz;
    double self_drive_vx, self_drive_vy, self_drive_vz;

    // Compute repulsive velocity with heading adjustment
    computeRepulsiveVelocityWithHeading(currentAgent, neighbors, rep_vx, rep_vy, rep_vz);

    // Compute friction velocity
    //computeFrictionVelocity(currentAgent, neighbors, 0.5, friction_vx, friction_vy, friction_vz);
    computeFrictionVelocity(currentAgent, neighbors, friction_vx, friction_vy, friction_vz);
    // Compute self-drive velocity toward the target
    computeSelfDriveVelocity(currentAgent, neighbors, self_drive_vx, self_drive_vy, self_drive_vz);
    //computeSelfdrivingvelocity(currentAgent, neighbors, self_drive_vx, self_drive_vy, self_drive_vz);
 
    
    // Combine the velocities (Repulsive + Friction + Self-Drive) in NED
    total_vx = rep_vx + self_drive_vx + friction_vx; // North (X)
    total_vy = rep_vy + self_drive_vy + friction_vy; // East (Y)
    total_vz = rep_vz + self_drive_vz + friction_vz; // Down (Z)
    ESP_LOGE("DCATS", "Total Velocity (X, Y, Z): %.2f m/s, %.2f m/s, %.2f m/s", total_vx, total_vy, total_vz);

}

/*-----------------*/

// Function to send a MAVLink heartbeat message
void send_heartbeat(void* arg) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(received_sysid, component_id, &msg, mav_type, autopilot_type, base_mode, custom_mode, system_status);

    // Send the message via serial
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t length = mavlink_msg_to_send_buffer(buffer, &msg);
    uart_write_bytes(CONFIG_UART_PORT_NUM, (const char *)buffer, length);
}


void setup_timers() {
    // Create and start timer for heartbeat (1Hz)
    esp_timer_handle_t heartbeat_timer;
    const esp_timer_create_args_t heartbeat_timer_args = {
        .callback = &send_heartbeat,
        .name = "heartbeat_sender"
    };
    esp_timer_create(&heartbeat_timer_args, &heartbeat_timer);
    esp_timer_start_periodic(heartbeat_timer, 1000000);  // 1 second (1Hz)
}

void staleDeviceTask(void *pvParameters) {
    while (true) {
        uint32_t current_time = millis();  // Get current time
        removeStaleDevices(current_time);  // Remove stale devices
       
        vTaskDelay(pdMS_TO_TICKS(1000));   // Check for stale devices every 1 second
    }
}

void sendDataTask(void *pvParameters) {
    while (true) {
        // Send data via ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
        if (result == ESP_OK) {
            ESP_LOGI("SendTask", "Data sent successfully");
        } else {
            ESP_LOGE("SendTask", "Error sending data: %s", esp_err_to_name(result));
        }

        // Wait for a specified time before sending again (e.g., every 100ms)
       
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void velocityControlTask(void *pvParameters) {
    const int frequencyHz = 40;      // Target frequency
    const int periodMs = 1000 / frequencyHz;  // Period in milliseconds (25 ms)

    while (true) {
        // Compute total velocity
        Agent agent2 = {-353634043, 1491651470, 82357, 4000 , 0, 0, 0};
        std::vector<Agent> neighbors = {agent2};

        // for (int i = 0; i < device_count; ++i) {
        //     neighbors.push_back(convertToAgent(devices[i]));
        // }
        double total_vx, total_vy, total_vz;
        computeTotalVelocity(myData, neighbors, total_vx, total_vy, total_vz);

        // Calculate the 3D distance between the current position and the target
        double distance = calculateDistance3D(myData.lat, myData.lon, myData.alt, myData.target_lat, myData.target_lon, myData.target_alt);

        // Define a buffer distance for deceleration start
        double deceleration_start_distance = 10;  // Start decelerating 3 times the tolerance distance before

        // Gradually reduce velocity as the distance decreases
        if (distance <= target_switch_tolerance) {
            // If the distance is less than or equal to the target tolerance, stop completely
            total_vx = 0.0;
            total_vy = 0.0;
            total_vz = 0.0;
        } else if (distance <= deceleration_start_distance) {
            // Compute a scaling factor based on the distance and deceleration start distance
            double reduction_factor = (distance - target_switch_tolerance) / (deceleration_start_distance - target_switch_tolerance);
            
            // Gradually reduce the velocity by applying the reduction factor
            total_vx *= reduction_factor;
            total_vy *= reduction_factor;
            total_vz *= reduction_factor;
        }

        // Send the updated velocity command
        sendTrajectoryVelocityCommand(total_vx, total_vy, total_vz);
       

        // Delay to maintain 40Hz loop frequency (25 ms)
        vTaskDelay(pdMS_TO_TICKS(periodMs));  // Delay for 25 ms (40Hz frequency)
    }
}


void setup() {
    // UART Configuration
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

    // WiFi and ESP-NOW Configuration
    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_max_tx_power(40);  // Max transmission power for longer range
    
    esp_err_t init_result = esp_now_init();
    if (init_result != ESP_OK) {
        ESP_LOGE(ESP_NOW, "ESP-NOW Initialization failed");
        return;
    }
    ESP_LOGE(ESP_NOW, "ESP-NOW Initialized successfully");
    
    // Register ESP-NOW Callbacks
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
    esp_now_del_peer(NULL);

 
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);  // Use the valid broadcast address
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(ESP_NOW, "Failed to add broadcast peer");
        return;
    }
    ESP_LOGE(ESP_NOW, "Broadcast peer added successfully");

    // Create Mutex for synchronization
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
        ESP_LOGE("Semaphore", "Mutex creation failed!");
        return;  // Exit the setup function if the semaphore couldn't be created
    }

    // Additional application-specific setups
    listenHeartbeat();
    sendHeartbeat(received_sysid);
    streamRequest(received_sysid, received_compid); 
    setup_timers();
    enableMessageInterval(1000000, received_sysid, received_compid);

    // Create FreeRTOS Tasks
    xTaskCreate(readMavlinkMessages, "Read Mavlink", 4096, NULL, 5, NULL);  // Higher priority
    xTaskCreate(staleDeviceTask, "Stale Device Task", 4096, NULL, 5, NULL); 
    xTaskCreate(sendDataTask, "Send Data Task", 4096, NULL, 5, NULL); 
    xTaskCreate(velocityControlTask, "Velocity Control Task", 4096, NULL, 5, NULL);
}


void loop() {

            
}