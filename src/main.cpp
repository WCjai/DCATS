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
const double EARTH_RADIUS = 6371000.0; // Earth's radius in meters
#define MAX_DEVICES 10  // Maximum number of unique MAC addresses you want to track
#define TIMEOUT_PERIOD 10000  // Timeout period in milliseconds (10 seconds)


/*uart config*/
#define BUFFER_SIZE 256
int baudrate = 115200;
const uart_port_t CONFIG_UART_PORT_NUM = UART_NUM_2;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


/*Autopilot sent config*/
uint8_t system_id = 255;        // ESP32 system ID
uint8_t component_id = 0;       // ESP32 component ID
uint8_t received_sysid = 0;
uint8_t received_compid = 0;


/*Array to hold mission items*/
typedef struct {
    uint16_t seq;     // Mission sequence number
    int32_t lat;      // Latitude
    int32_t lon;      // Longitude
    float alt;        // Altitude
} mission_item_t;
mission_item_t mission_items[100];
uint16_t mission_item_count = 0;   // stores recived total count


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

const double REPULSION_ZONE_RADIUS = 20.0;   // Repulsion zone radius in meters
const double R = 10.0;                       // friction between drones
const double SELFDRIVING_ZONE_RADIUS = 50.0;// self-driving zone


const double REPULSION_GAIN = 0.1; // Repulsion gain factor
const double FRICTION_THRESHOLD_VELOCITY = 2.0; // Minimum friction velocity in m/s
const double FRICTION_GAIN = 0.1; // Friction gain factor
const double a = 0.3; // Acceleration limit
const double MAX_SPEED = 5.0; // Max speed in m/s for self-drive velocity

bool is_guided = false;
bool is_auto = false;

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
const double target_switch_tolerance = 3.0;  // Example: 5 meters tolerance
bool is_first_target_set = false;



/*log*/
static const char *ESP_NOW = "COMM";
static const char *DCAT = "DCATS";
static const char *Vfri = "friction_velocity";
static const char *vrep = "repulsive_velocity";


/*----------Mavlink thing -----------*/

void sendHeartbeat() {
    delay(2000);
    mavlink_message_t msghb;
    uint8_t bufhb[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(system_id, component_id, &msghb, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_GENERIC, MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);
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

void requestMissionList(uint8_t target_sysid, uint8_t target_compid) {
    mavlink_message_t msg;
    mavlink_msg_mission_request_list_pack(system_id, component_id, &msg, target_sysid, target_compid, MAV_MISSION_TYPE_MISSION);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    delay(1000);
    uart_write_bytes(CONFIG_UART_PORT_NUM, (const char *)buf, len);
    ESP_LOGE("AutoPilot","Requested mission list.");
}

void requestMissionItemInt(uint8_t target_sysid, uint8_t target_compid, uint16_t seq) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the mission request message
    mavlink_msg_mission_request_int_pack(system_id, component_id, &msg, target_sysid, target_compid, seq, MAV_MISSION_TYPE_MISSION);
    
    // Convert MAVLink message to buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Write the buffer to UART
    uart_write_bytes(CONFIG_UART_PORT_NUM, (const char *)buf, len);
    
    // Log the mission item request
    ESP_LOGE("AutoPilot", "Requested mission item %d", seq);
}

void changeModeToAuto(uint8_t target_sysid, uint8_t target_compid) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // Set base mode and custom mode for AUTO
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint32_t custom_mode = 3;  // This is typically the value for AUTO mode in ArduPilot systems

    // Pack the COMMAND_LONG message to set the mode
    mavlink_msg_command_long_pack(system_id, component_id, &msg, target_sysid, target_compid, 
                                  MAV_CMD_DO_SET_MODE, 0, 
                                  base_mode, custom_mode, 0, 0, 0, 0, 0);

    // Send the message over UART
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    uart_write_bytes(CONFIG_UART_PORT_NUM, (const char *)buf, len);
    is_auto = true;
    is_guided = false;

    ESP_LOGE("AutoPilot","Sent mode change command to AUTO mode.");
}

void ModeToAuto(uint8_t target_sysid, uint8_t target_compid) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // Set base mode and custom mode for AUTO
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint32_t custom_mode = 3;  // This is typically the value for AUTO mode in ArduPilot systems

    // Pack the COMMAND_LONG message to set the mode
    mavlink_msg_command_long_pack(system_id, component_id, &msg, target_sysid, target_compid, 
                                  MAV_CMD_DO_SET_MODE, 0, 
                                  base_mode, custom_mode, 0, 0, 0, 0, 0);

    // Send the message over UART
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    uart_write_bytes(CONFIG_UART_PORT_NUM, (const char *)buf, len);
    ESP_LOGE("AutoPilot","Sent mode change command to AUTO mode.");
}

void changeModeToGuided(uint8_t target_sysid, uint8_t target_compid) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // Set base mode and custom mode for GUIDED
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint32_t custom_mode = 4;  // This is typically the value for GUIDED mode in most autopilot systems (check your autopilot's documentation for the correct value)

    // Pack the COMMAND_LONG message to set the mode
    mavlink_msg_command_long_pack(system_id, component_id, &msg, target_sysid, target_compid, 
                                  MAV_CMD_DO_SET_MODE, 0, 
                                  base_mode, custom_mode, 0, 0, 0, 0, 0);

    // Send the message over UART
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    //mavlinkSerial.write(buf, len);
    uart_write_bytes(CONFIG_UART_PORT_NUM, (const char *)buf, len);
    is_guided = true;
    is_auto = false;

    ESP_LOGE("AutoPilot","Sent mode change command to GUIDED mode.");
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

void handleMissionCount(mavlink_message_t* msg) {
    mavlink_mission_count_t mission_count;

    // Decode the mission count message
    mavlink_msg_mission_count_decode(msg, &mission_count);
    
    ESP_LOGE("AutoPilot", "Number of mission items: %d", mission_count.count);

    if (mission_count.count == 0) {
        ESP_LOGE("AutoPilot", "No mission exists.");
    } else {
        // Store mission item count and start requesting mission items
        mission_item_count = mission_count.count;

        // Request each mission item
        for (uint16_t i = 0; i < mission_item_count; i++) {
            requestMissionItemInt(received_sysid, received_compid, i);
        }
    }
}

void handleMissionItemInt(mavlink_message_t* msg) {
    mavlink_mission_item_int_t mission_item;

    // Decode the mission item message
    mavlink_msg_mission_item_int_decode(msg, &mission_item);

    // Ensure the mission item sequence is within bounds
    if (mission_item.seq < mission_item_count) {
        // Store the mission item
        mission_items[mission_item.seq].seq = mission_item.seq;
        mission_items[mission_item.seq].lat = mission_item.x;
        mission_items[mission_item.seq].lon = mission_item.y;
        mission_items[mission_item.seq].alt = mission_item.z;

        ESP_LOGE("AutoPilot", "Mission item received: seq=%d, lat=%d, lon=%d, alt=%f", 
                 mission_item.seq, mission_item.x, mission_item.y, mission_item.z);
    } else {
        ESP_LOGE("AutoPilot", "Received mission item with out-of-bounds sequence: seq=%d", mission_item.seq);
    }
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

/* Mavlink thing end below starts capturing mavlink*/
void readMavlinkMessages() {
    size_t available_bytes = 0;
    uart_get_buffered_data_len(CONFIG_UART_PORT_NUM, &available_bytes);

    if (available_bytes > 0) {
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t tx_buf[available_bytes]; 
        int num_bytes = uart_read_bytes(CONFIG_UART_PORT_NUM, tx_buf, available_bytes, 20 / portTICK_RATE_MS);

        // Loop through the received bytes and parse MAVLink messages
        for (int i = 0; i < num_bytes; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, tx_buf[i], &msg, &status)) {
                // Handle the parsed message based on its ID
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_MISSION_COUNT:
                        handleMissionCount(&msg);
                        break;
                    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
                        handleMissionItemInt(&msg);
                        break;
                    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                        handleGlobalPosInt (&msg);
                        break;
                    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
                        handleTargetPosInt (&msg);
                        break;
                    default:
                        ESP_LOGI("AutoPilot", "Unhandled MAVLink message ID: %d", msg.msgid);
                        break;
                }
            }
        }
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

void switchTargetPos (uint16_t switch_index){
    if (switch_index >= mission_item_count) {
         myData.target_lat = mission_items[mission_item_count].lat;
         myData.target_lon = mission_items[mission_item_count].lon;
         myData.target_alt = (mission_items[mission_item_count].alt + mission_items[0].alt)*1000;
         ESP_LOGE("LOG","No more target, setting %d seq as target", mission_item_count);
         return;
    }
    myData.target_lat = mission_items[switch_index].lat;
    myData.target_lon = mission_items[switch_index].lon;
    myData.target_alt = (mission_items[switch_index].alt + mission_items[0].alt)*1000;
    ESP_LOGE("LOG","switching target");
}
void targetAssigner() {
    //first assign first target 
    if (!is_first_target_set) {
        if (mission_items[1].lat !=0 && mission_items[1].lat && mission_items[1].alt !=0 ) {
            myData.target_lat = mission_items[target_switch_index].lat;
            myData.target_lon = mission_items[target_switch_index].lon;
            myData.target_alt = (mission_items[target_switch_index].alt + mission_items[0].alt)*1000;
            is_first_target_set = true;
        } else {
            myData.target_lat = 0;
            myData.target_lon = 0;
            myData.target_alt = 0;
        }
        
    }
    if (is_first_target_set) {
        
        double distance = calculateDistance3D(myData.lat, myData.lon, myData.alt, myData.target_lat, myData.target_lon, myData.target_alt);
        //ESP_LOGE("TAG","dis %f", distance);
        if (distance <= target_switch_tolerance ) {
            target_switch_index++;
            switchTargetPos(target_switch_index);
        }

    }

}

void switchAutotarget() {
    if (myData.target_lat !=0 && myData.target_lon !=0  && myData.target_alt !=0 ) {
        double distance = calculateDistance3D(myData.lat, myData.lon, myData.alt, myData.target_lat, myData.target_lon, myData.target_alt);
        if (distance <= target_switch_tolerance ) {
            ModeToAuto(received_sysid, received_compid);
        }
    }
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

void addOrUpdateDevice(const uint8_t *mac, const Agent *data, uint32_t current_time) {
    int index = findDeviceIndex(mac);

    if (index != -1) {
        // MAC address found, update existing data
        devices[index].gps_data = *data;
        devices[index].last_update_time = current_time;  // Update last update time
    } else {
        // MAC address not found, add new entry if space is available
        if (device_count < MAX_DEVICES) {
            memcpy(devices[device_count].mac, mac, 6);  // Copy MAC address
            devices[device_count].gps_data = *data;  // Copy GPS data
            devices[device_count].last_update_time = current_time;  // Set last update time
            device_count++;
        } else {
            ESP_LOGE(DCAT, "Device table is full. Can't add new device.");
        }
    }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    //ESP_LOGE(ESP_NOW, "Message Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(Agent)) {
        Agent receivedData;
        memcpy(&receivedData, incomingData, sizeof(receivedData));

        // Add or update the device data in the table with the current time
        uint32_t current_time = millis();
        addOrUpdateDevice(mac, &receivedData, current_time);

        // Print the updated device table
        //printDeviceTable();
    } else {
        ESP_LOGE(ESP_NOW, "Invalid data length received");
    }
}

void removeStaleDevices(uint32_t current_time) {
    for (int i = 0; i < device_count; i++) {
        // Check if the last update time exceeds the timeout period
        if (current_time - devices[i].last_update_time > TIMEOUT_PERIOD) {
            ESP_LOGE(DCAT, "Removing stale device: %02X:%02X:%02X:%02X:%02X:%02X",
                     devices[i].mac[0], devices[i].mac[1], devices[i].mac[2],
                     devices[i].mac[3], devices[i].mac[4], devices[i].mac[5]);
            // Shift the remaining devices up
            for (int j = i; j < device_count - 1; j++) {
                devices[j] = devices[j + 1];
            }
            device_count--;
            i--;  // Recheck the current index after removal
        }
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

            // Normalize the direction vector
            double dx = (0 - x_j) / distance;
            double dy = (0 - y_j) / distance;
            
            // Adjust based on heading
            double agent_dx, agent_dy, neighbor_dx, neighbor_dy;
            headingToUnitVector(currentAgent.hdg, agent_dx, agent_dy);
            headingToUnitVector(neighbor.hdg, neighbor_dx, neighbor_dy);
            
            // Compute angle between current agent's heading and repulsion direction
            double dot_product = dx * agent_dx + dy * agent_dy;
            double heading_factor = fabs(dot_product); // This can be adjusted further based on your needs
            
            // Apply heading factor to the repulsive force
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

double computeBrakingFunction(double distance, double rho, double R, double a) {
    double d_offset = distance - R;
    if (d_offset < 0) {
        return 0;
    } else if (d_offset <= a / (rho * rho)) {
        return rho * d_offset;
    } else {
        return sqrt(2 * a * d_offset) - a / rho;
    }
}

void computeFrictionVelocity(const Agent& currentAgent, const std::vector<Agent>& neighbors, double rho, double& friction_vx, double& friction_vy, double& friction_vz) { 
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
            double v_friction_threshold = std::max(FRICTION_THRESHOLD_VELOCITY, computeBrakingFunction(distance, rho, R, a));
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
bool triggerselfdrive(const Agent& currentAgent, const std::vector<Agent>& neighbors) {

    for (const auto& neighbor : neighbors) {
        // Convert positions to NED relative to the current agent
        double x_j, y_j, z_j;
        gpsToNED(currentAgent, neighbor, x_j, y_j, z_j); // Neighbor's NED position

        // Compute distance between current agent and neighbor
        double distance = computeDistance(0, 0, 0, x_j, y_j, z_j); // Current agent's NED position is always (0, 0, 0)
        
        // Apply repulsion if within the repulsion zone
        if (distance < SELFDRIVING_ZONE_RADIUS && distance > 0) {
            return true;
        }
    }
    return false;
}

void computeSelfDriveVelocity(const Agent& currentAgent, const std::vector<Agent>& neighbors, double& self_drive_vx, double& self_drive_vy, double& self_drive_vz) {
    double x_target, y_target, z_target;

    // Convert target position to NED relative to the current position
    Agent targetAgent = {currentAgent.target_lat, currentAgent.target_lon, currentAgent.target_alt, 0, 0, 0, 0};
    if (currentAgent.target_lat != 0 && currentAgent.target_lon != 0) {
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

            // Now modify the self-drive velocity based on neighbors' positions and danger criteria (D1-D4)
            for (const auto& neighbor : neighbors) {
                double x_j, y_j, z_j;
                if (checkDangerCriteria(currentAgent, neighbor, x_j, y_j, z_j)) {
                    // Apply geometric constraints to adjust velocity (C1-C5)

                    // C1: Ensure the velocity does not point into the dangerous direction
                    double dot_product = (self_drive_vx * x_j + self_drive_vy * y_j + self_drive_vz * z_j);
                    if (dot_product > 0) { // Move away from the neighbor
                        double norm_factor = sqrt(x_j * x_j + y_j * y_j + z_j * z_j);
                        self_drive_vx -= (x_j / norm_factor) * MAX_SPEED;
                        self_drive_vy -= (y_j / norm_factor) * MAX_SPEED;
                        self_drive_vz -= (z_j / norm_factor) * MAX_SPEED;
                    }

                }
    
            }


    } else {
            self_drive_vx = 0.0;
            self_drive_vy = 0.0;
            self_drive_vz = 0.0;

    }

}


void computeTotalVelocity(const Agent& currentAgent, const std::vector<Agent>& neighbors, double& total_vx, double& total_vy, double& total_vz) {
    double rep_vx, rep_vy, rep_vz;
    double friction_vx, friction_vy, friction_vz;
    double self_drive_vx, self_drive_vy, self_drive_vz;

    // Compute repulsive velocity with heading adjustment
    computeRepulsiveVelocityWithHeading(currentAgent, neighbors, rep_vx, rep_vy, rep_vz);

    // Compute friction velocity
    computeFrictionVelocity(currentAgent, neighbors, 0.5, friction_vx, friction_vy, friction_vz);
    
    // Compute self-drive velocity toward the target
    computeSelfDriveVelocity(currentAgent, neighbors, self_drive_vx, self_drive_vy, self_drive_vz);
 
    
    // Combine the velocities (Repulsive + Friction + Self-Drive) in NED
    total_vx = rep_vx + self_drive_vx + friction_vx; // North (X)
    total_vy = rep_vy + self_drive_vy + friction_vy; // East (Y)
    total_vz = rep_vz + self_drive_vz + friction_vz; // Down (Z)
    //ESP_LOGE("DCATS", "repulsive Velocity (X, Y, Z): %.2f m/s, %.2f m/s, %.2f m/s", rep_vx, rep_vy, rep_vz);
    //ESP_LOGE("DCATS", "friction Velocity (X, Y, Z): %.2f m/s, %.2f m/s, %.2f m/s", friction_vx, friction_vy, friction_vz);
    //ESP_LOGE("DCATS", "Self-drive Velocity (X, Y, Z): %.2f m/s, %.2f m/s, %.2f m/s", self_drive_vx, self_drive_vy, self_drive_vz);
   
    ESP_LOGE("DCATS", "Total Velocity (X, Y, Z): %.2f m/s, %.2f m/s, %.2f m/s", total_vx, total_vy, total_vz);
}

/*-----------------*/
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
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
    esp_now_del_peer(NULL);

    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(ESP_NOW, "Failed to add broadcast peer");
        return;
    }
    ESP_LOGE(ESP_NOW, "Broadcast peer added successfully");
    sendHeartbeat();
    listenHeartbeat();
    streamRequest(received_sysid, received_compid);
    requestMissionList(received_sysid, received_compid); 

}


void loop() {
    readMavlinkMessages();
    switchAutotarget();
    // if (target_switch_index != mission_item_count) {
    //    // targetAssigner();
    // }
    //ESP_LOGE("DCATS", "Target: %d lat, %d lon, %d alt ", myData.target_lat, myData.target_lon, myData.target_alt);
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    uint32_t current_time = millis();
    removeStaleDevices(current_time);
    std::vector<Agent> neighbors;

    for (int i = 0; i < device_count; ++i) {
        neighbors.push_back(convertToAgent(devices[i]));
    }

    double total_vx, total_vy, total_vz;
    
    if (triggerselfdrive(myData, neighbors)) {
        computeTotalVelocity(myData, neighbors, total_vx, total_vy, total_vz);
        while (!is_guided)
        {
            changeModeToGuided(received_sysid, received_compid);
        }
        sendVelocityCommand(received_sysid, received_compid, total_vx, total_vy, total_vz);  // Send desired velocities
        
    } else {
        while (!is_auto)
        {
            changeModeToAuto(received_sysid, received_compid);
        }
        
    }

         
}