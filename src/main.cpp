#include "ardupilotmega/mavlink.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "Arduino.h"
#include <esp_now.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include <WiFi.h>
#include <cmath>
#include <vector>
#define BUFFER_SIZE 256
#define MAX_DEVICES 10  // Maximum number of unique MAC addresses you want to track
#define TIMEOUT_PERIOD 10000  // Timeout period in milliseconds (10 seconds)

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

uint16_t hdg;
int16_t vel_x, vel_y, vel_z;
uint16_t len;
int32_t latitude, longitude, altitude, target_lat, target_lon, target_alt;

bool enable_selfdrive = false;



uint8_t buffer[MAVLINK_MAX_PACKET_LEN]; // Buffer to hold GPS data

const double EARTH_RADIUS = 6371000.0; // Earth's radius in meters
//const double DEG_TO_RAD = M_PI / 180.0; // Conversion factor from degrees to radians

const double REPULSION_ZONE_RADIUS = 100.0; // Repulsion zone radius in meters
const double REPULSION_GAIN = 0.1; // Repulsion gain factor

const double FRICTION_THRESHOLD_VELOCITY = 2.0; // Minimum friction velocity in m/s
const double FRICTION_GAIN = 0.1; // Friction gain factor
const double R = 50; // Offset value
const double a = 0.3; // Acceleration limit

const double MAX_SPEED = 5.0; // Max speed in m/s for self-drive velocity


static const char *TAG = "GPS_LOG";


// GPS message structure
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


// Structure to hold MAC and associated GPS data
typedef struct device_data {
    uint8_t mac[6];  // MAC address (6 bytes)
    Agent gps_data;  // Associated GPS data
    uint32_t last_update_time;  // Timestamp of the last received data
} device_data;

device_data devices[MAX_DEVICES];  // Array to hold MAC and associated data
int device_count = 0;  // Current count of devices in the table

Agent myData;
Agent receivedData;
Agent agent1;

// Convert lat/lon/alt to NED (North, East, Down) relative to the current agent
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

// Compute Euclidean distance between two points in NED space
double computeDistance(double x1, double y1, double z1, double x2, double y2, double z2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
}

// Compute relative velocity between two agents in NED frame
double computeRelativeVelocity(const Agent& agent1, const Agent& agent2) {
    // Calculate the relative velocity (in m/s)
    double vx_diff = (agent1.vx - agent2.vx) / 100.0; // Convert cm/s to m/s
    double vy_diff = (agent1.vy - agent2.vy) / 100.0;
    double vz_diff = (agent1.vz - agent2.vz) / 100.0;
    
    //ESP_LOGE(TAG, "Agent 1 Velocity (vx, vy, vz): %.2f, %.2f, %.2f", agent1.vx / 100.0, agent1.vy / 100.0, agent1.vz / 100.0);
    //ESP_LOGE(TAG, "Agent 2 Velocity (vx, vy, vz): %.2f, %.2f, %.2f", agent2.vx / 100.0, agent2.vy / 100.0, agent2.vz / 100.0);

    return sqrt(vx_diff * vx_diff + vy_diff * vy_diff + vz_diff * vz_diff);
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


// Compute braking function for friction based on distance and other parameters
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

// Compute frictional velocity for an agent based on its neighbors
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
            ESP_LOGE(TAG, "Relative Velocity: %.2f m/s, Friction Threshold: %.2f m/s", v_ij, v_friction_threshold);

            // If relative velocity exceeds the threshold, apply friction
            if (v_ij > v_friction_threshold) {
                ESP_LOGE(TAG, "Applying friction force between Agent 1 and Neighbor: %.2f m/s^2", v_ij);

                double friction_force = FRICTION_GAIN * (v_ij - v_friction_threshold);

                // Normalize the relative velocity vector and apply friction
                double vx_diff = (currentAgent.vx - neighbor.vx) / 100.0; // Convert to m/s
                double vy_diff = (currentAgent.vy - neighbor.vy) / 100.0;
                double vz_diff = (currentAgent.vz - neighbor.vz) / 100.0;
                double velocity_magnitude = sqrt(vx_diff * vx_diff + vy_diff * vy_diff + vz_diff * vz_diff);

                ESP_LOGE(TAG, "Velocity Differences - X: %.2f, Y: %.2f, Z: %.2f", vx_diff, vy_diff, vz_diff);
                ESP_LOGE(TAG, "Velocity Magnitude: %.2f m/s", velocity_magnitude);

                if (velocity_magnitude > 0) {
                    friction_vx += friction_force * (vx_diff / velocity_magnitude);
                    friction_vy += friction_force * (vy_diff / velocity_magnitude);
                    friction_vz += friction_force * (vz_diff / velocity_magnitude);
                }
            }
        }
    }
}



// Convert heading (in centidegrees) to a unit vector (dx, dy)
void headingToUnitVector(uint16_t hdg, double& dx, double& dy) {
    double heading_rad = (hdg / 100.0) * DEG_TO_RAD; // Convert centidegrees to radians
    dx = cos(heading_rad);  // X component (North direction)
    dy = sin(heading_rad);  // Y component (East direction)
}



// Modify the repulsive force based on relative heading
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

     //ESP_LOGE("DCATS", "repulse Velocity (X, Y, Z): %.2f m/s, %.2f m/s, %.2f m/s", rep_vx, rep_vy, rep_vz);
}

// Compute the self-drive velocity toward the target position
// Compute the self-drive velocity toward the target while avoiding dangerous neighbors
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

                    // We could also handle C2-C5 similarly by ensuring safe modifications to the velocity
                    // Example: ensuring we don't increase velocity in dangerous regions (C3)
                }
            }


    } else {
            self_drive_vx = 0.0;
            self_drive_vy = 0.0;
            self_drive_vz = 0.0;

    }

    //std::cout << "Self-drive velocity: vx=" << self_drive_vx << " vy=" << self_drive_vy << " vz=" << self_drive_vz << std::endl;
    //ESP_LOGE("DCATS", "Self-drive Velocity (X, Y, Z): %.2f m/s, %.2f m/s, %.2f m/s", self_drive_vx, self_drive_vy, self_drive_vz);
}

// Combine repulsive, friction, and self-drive velocities for total velocity adjustment
void computeTotalVelocity(const Agent& currentAgent, const std::vector<Agent>& neighbors, double& total_vx, double& total_vy, double& total_vz) {
    double rep_vx, rep_vy, rep_vz;
    double friction_vx, friction_vy, friction_vz;
    double self_drive_vx, self_drive_vy, self_drive_vz =0;

    // Compute repulsive velocity with heading adjustment
    computeRepulsiveVelocityWithHeading(currentAgent, neighbors, rep_vx, rep_vy, rep_vz);

    // Compute friction velocity
    computeFrictionVelocity(currentAgent, neighbors, 0.5, friction_vx, friction_vy, friction_vz);
    
    
    computeSelfDriveVelocity(currentAgent, neighbors, self_drive_vx, self_drive_vy, self_drive_vz);
    // Compute self-drive velocity toward the target
    

    // Combine the velocities (Repulsive + Friction + Self-Drive) in NED
    total_vx = rep_vx + self_drive_vx + friction_vx; // North (X)
    total_vy = rep_vy + self_drive_vy + friction_vy; // East (Y)
    total_vz = rep_vz + self_drive_vz + friction_vz; // Down (Z)
    //ESP_LOGE("DCATS", "repulsive Velocity (X, Y, Z): %.2f m/s, %.2f m/s, %.2f m/s", rep_vx, rep_vy, rep_vz);
    //ESP_LOGE("DCATS", "friction Velocity (X, Y, Z): %.2f m/s, %.2f m/s, %.2f m/s", friction_vx, friction_vy, friction_vz);
    //ESP_LOGE("DCATS", "Self-drive Velocity (X, Y, Z): %.2f m/s, %.2f m/s, %.2f m/s", self_drive_vx, self_drive_vy, self_drive_vz);
    ESP_LOGE("DCATS", "Total Velocity (X, Y, Z): %.2f m/s, %.2f m/s, %.2f m/s", total_vx, total_vy, total_vz);
}


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
            ESP_LOGE(TAG, "Device table is full. Can't add new device.");
        }
    }
}

// Function to remove stale entries from the device table
void removeStaleDevices(uint32_t current_time) {
    for (int i = 0; i < device_count; i++) {
        // Check if the last update time exceeds the timeout period
        if (current_time - devices[i].last_update_time > TIMEOUT_PERIOD) {
            ESP_LOGE(TAG, "Removing stale device: %02X:%02X:%02X:%02X:%02X:%02X",
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


void printDeviceTable() {
    ESP_LOGE(TAG, "Device Table:");
    for (int i = 0; i < device_count; i++) {
        ESP_LOGE(TAG, "MAC: %02X:%02X:%02X:%02X:%02X:%02X | Lat: %ld, Lon: %ld, Alt: %ld, VelX: %d, VelY: %d, VelZ: %d, heading: %ld, Last Update: %ld",
                 devices[i].mac[0], devices[i].mac[1], devices[i].mac[2],
                 devices[i].mac[3], devices[i].mac[4], devices[i].mac[5],
                 devices[i].gps_data.lat, devices[i].gps_data.lon, devices[i].gps_data.alt,
                 devices[i].gps_data.vx, devices[i].gps_data.vy, devices[i].gps_data.vz,
                 devices[i].gps_data.hdg, devices[i].last_update_time );
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

void ReadGPS(int32_t *lat, int32_t *lon, int32_t *alt, int16_t *velx, int16_t *vely, int16_t *velz, uint16_t *hdg) {
    size_t available_bytes = 0;
    uart_get_buffered_data_len(CONFIG_UART_PORT_NUM, &available_bytes);
    uint16_t h = 0;
    int16_t x = 0, y = 0, z = 0;
    uint32_t  la = 0, lo = 0, al = 0;
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
                        h = data.hdg;
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
    *hdg = h;
}

void ReadGPSandTarget(int32_t *lat, int32_t *lon, int32_t *alt, int16_t *velx, int16_t *vely, int16_t *velz, uint16_t *hdg, 
             int32_t *target_lat, int32_t *target_lon, int32_t *target_alt) {
    size_t available_bytes = 0;
    uart_get_buffered_data_len(CONFIG_UART_PORT_NUM, &available_bytes);
    uint16_t h = 0;
    int16_t x = 0, y = 0, z = 0;
    int32_t la = 0, lo = 0, al = 0;
    int32_t t_lat = 0, t_lon = 0, t_alt = 0;
    float _alt = 0;
    
    if (available_bytes > 0) {
        int len = uart_read_bytes(CONFIG_UART_PORT_NUM, tx_buf, available_bytes, 20 / portTICK_RATE_MS);
        delay(100);  // Control sending rate (10 Hz)
        if (len > 0) {
            mavlink_message_t msg;
            mavlink_status_t status1;
            for (int i = 0; i < len; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_1, tx_buf[i], &msg, &status1)) {
                    // Handle GLOBAL_POSITION_INT message
                    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                        mavlink_global_position_int_t data;
                        mavlink_msg_global_position_int_decode(&msg, &data);
                        la = data.lat;
                        lo = data.lon;
                        al = data.alt;
                        x = data.vx;
                        y = data.vy;
                        z = data.vz;
                        h = data.hdg;
                    }
                    // Handle POSITION_TARGET_GLOBAL_INT message
                    else if (msg.msgid == MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT) {
                        mavlink_position_target_global_int_t target_data;
                        mavlink_msg_position_target_global_int_decode(&msg, &target_data);
       
                             t_lat = target_data.lat_int;
                             t_lon = target_data.lon_int;
                             t_alt = (int32_t)(target_data.alt * 1000);
                             ESP_LOGE(TAG, "Target Latitude: %d, Longitude: %d", t_lat, t_lon);
                       

                    }
                }
            }
        }
    }
    // Assigning the global position values to output variables
    *lat = la;
    *lon = lo;
    *alt = al;
    *velx = x;
    *vely = y;
    *velz = z;
    *hdg = h;

    // Assigning the target position values to output variables
    *target_lat = t_lat;
    *target_lon = t_lon;
    *target_alt = t_alt;
}



// Callback when data is received via ESP-NOW
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
        ESP_LOGE(TAG, "Invalid data length received");
    }
}


void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
   // ESP_LOGE(TAG, "Message Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
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

void loop() {
    int32_t latitude, longitude, altitude;
    int16_t vel_x, vel_y, vel_z;
    uint16_t hdg;
    int32_t target_lat, target_lon, target_alt;

    // Read GPS data
    ReadGPSandTarget(&latitude, &longitude, &altitude, &vel_x, &vel_y, &vel_z, &hdg, &target_lat, &target_lon, &target_alt);

    // Check if GPS data is valid
    if (latitude != 0 && longitude != 0) {
        myData.lat = latitude;
        myData.lon = longitude;
        myData.alt = altitude;
        myData.vx = vel_x;
        myData.vy = vel_y;
        myData.vz = vel_z;
        myData.hdg = hdg;
        myData.target_lat = target_lat;
        myData.target_lon = target_lon;
        myData.target_alt = target_alt;

        // Send the data over ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
        if (result == ESP_OK) {
            //ESP_LOGE(TAG, "GPS message sent successfully via ESP-NOW");
        } else {
            ESP_LOGE(TAG, "Error sending GPS message");
        }
    } else {
        //ESP_LOGE(TAG, "Invalid GPS data, skipping this iteration");
    }

    // Periodically check and remove stale devices
    uint32_t current_time = millis();
    removeStaleDevices(current_time);

    // Initialize a vector for neighboring agents
    std::vector<Agent> neighbors;

    // Log device count to ensure it’s non-zero
    //ESP_LOGE("NEIGHBOR", "Device count: %d", device_count);

    // Check if devices array contains valid data
    for (int i = 0; i < device_count; ++i) {
        // Log the MAC address and check if data is valid before adding to neighbors
        // ESP_LOGE("NEIGHBOR", "Checking device %d, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
        //          i,
        //          devices[i].mac[0], devices[i].mac[1], devices[i].mac[2],
        //          devices[i].mac[3], devices[i].mac[4], devices[i].mac[5]);

        // Add only valid devices to the neighbors vector
        neighbors.push_back(convertToAgent(devices[i]));
    }

    // Log neighbor count to ensure neighbors are added correctly
    //ESP_LOGE("NEIGHBOR", "Total neighbors: %d", neighbors.size());

    // If neighbors are present, print their values
    // if (!neighbors.empty()) {
    //     for (size_t i = 0; i < neighbors.size(); ++i) {
    //         ESP_LOGE("NEIGHBOR", "Neighbor %d Values:", i + 1);
    //         ESP_LOGE("NEIGHBOR", "Latitude: %d", neighbors[i].lat);
    //         ESP_LOGE("NEIGHBOR", "Longitude: %d", neighbors[i].lon);
    //         ESP_LOGE("NEIGHBOR", "Altitude: %d", neighbors[i].alt);
    //         ESP_LOGE("NEIGHBOR", "Velocity X: %d", neighbors[i].vx);
    //         ESP_LOGE("NEIGHBOR", "Velocity Y: %d", neighbors[i].vy);
    //         ESP_LOGE("NEIGHBOR", "Velocity Z: %d", neighbors[i].vz);
    //         ESP_LOGE("NEIGHBOR", "Heading: %d", neighbors[i].hdg);
    //         ESP_LOGE("NEIGHBOR", "---------");
    //     }
    // } else {
    //     ESP_LOGE("NEIGHBOR", "No neighbors found.");
    // }

    double total_vx, total_vy, total_vz;
    computeTotalVelocity(myData, neighbors, total_vx, total_vy, total_vz);

    delay(100);  // Delay to control loop rate (10 Hz)
}