/*
*
*   ===================================================
*       CropDrop Bot (CB) Theme [eYRC 2025-26]
*   ===================================================
*
*  This script is intended to be an Boilerplate for 
*  Task 2a of CropDrop Bot (CB) Theme [eYRC 2025-26].
*
*  Filename:		Task2a.c
*  Created:		    19/08/2025
*  Last Modified:	12/09/2025
*  Author:		    Team members Name
*  Team ID:		    [ CB_xxxx ]
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using ICT (NMEICT)
*
**********************************************

*/
// Platform-specific includes for Windows compatibility
#ifdef _WIN32
    #define WINVER 0x0600
    #define _WIN32_WINNT 0x0600
    #include <winsock2.h>
    #include <ws2tcpip.h>
#endif

#include "coppeliasim_client.h"  // Include our header
#include <sys/time.h>
#include <math.h>
#include <string.h>

// Global client instance for socket communication
SocketClient client;

// ----------------------
// Forward declarations
// ----------------------
void* control_loop(void* arg);

/**
 * @brief Establishes connection to the CoppeliaSim server
 */
int connect_to_server(SocketClient* c, const char* ip, int port) {
#ifdef _WIN32
    // Initialize Winsock on Windows
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        printf("WSAStartup failed\n");
        return 0;
    }
#endif
    
    // Create TCP socket
    c->sock = socket(AF_INET, SOCK_STREAM, 0);
    if (c->sock < 0) {
        printf("Socket creation failed\n");
        return 0;
    }

    // Setup server address structure
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &serv_addr.sin_addr);

    // Attempt to connect to server
    if (connect(c->sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("Connection failed\n");
        CLOSESOCKET(c->sock);
#ifdef _WIN32
        WSACleanup();
#endif
        return 0;
    }

    c->running = true;

    // Start the receive thread to handle incoming sensor data
#ifdef _WIN32
    c->recv_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)receive_loop, c, 0, NULL);
#else
    pthread_create(&c->recv_thread, NULL, receive_loop, c);
#endif

    return 1;
}

/**
 * @brief Get current time in seconds
 */
double get_current_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
}

/*
---- line follower --------
*/
float Kp = 1.2, Ki = 0.04, Kd = 0.012;
float integral = 0, prevError = 0;
float dt = 0.01;
unsigned long prevTime = 0;

char colorDetected = 'N';
bool destination = false;
float threshold_max = 0.5f;
float threshold_min = 0.2f;
bool boxPicked = false;

float position(float sensor[],int s) {
    float weight[] = {-2, -1, 0, +1, +2};
    float sumSen = 0;
    float sumSenWeg = 0;

    for(int i =0; i<s; i++) {
        float inverted = 1.0f - sensor[i];  // for the inversion of line detection from white to black
        sumSen += inverted;
        sumSenWeg += (inverted*weight[i]);
    }

    if(sumSen == 0) return 0.0f;
    return sumSenWeg/sumSen ;
}

float normalize(float x,float oldMin,float oldMax,float newMin,float newMax) {
return ((x - oldMin) / (oldMax - oldMin)) * (newMax - newMin) + newMin;
}

char detectColor(float r,float g,float b) {
    char color = 'N';
    if(r > 0.0 || g > 0.0 || b > 0.0) {
        if(g > 0.5 && r > 0.1 && b == 0) {
            color = 'R';
        }
        else if(g == 0 && r > 0 && b > 0) {
            color = 'G';
        }
        else if(r > 0 && g == 0 && b == 0) {
            color = 'B';
        }
    }
    return color;
}

bool isBlack(SocketClient* c) {
    float ir1 = c->line_sensors[0];  // left_corner sensor
    float ir2 = c->line_sensors[1];  // left sensor
    float ir3 = c->line_sensors[2];  // middle sensor (center)
    float ir4 = c->line_sensors[3];  // right sensor
    float ir5 = c->line_sensors[4];

    if(ir1 < 0.4 && ir2 < 0.4 && ir3 < 0.4 && ir4 < 0.4 && ir5 < 0.4) {
        return true;
    }
    return false;
}

int thersholdreading(SocketClient *c,float max,float min) {
    int counter = 0;
    for(int i =0; i<5; i++) {
        if(c->line_sensors[i] != 0 && c->line_sensors[i] < max && c->line_sensors[i] > min) {
            counter++;
        }
    }
    return counter;
}

char chooseDirection(char boxColor) {
    char dir;
    if(boxColor == 'R') {
        dir = 'L';
    }
    else if(boxColor == 'G') {
        dir = 'R';   
    }
    else if(boxColor == 'B') {
        dir = 'F';
    }
    return dir;   
}

/**
 * @brief Main control loop thread for robot behavior
 */
void* control_loop(void* arg) {
    SocketClient* c = (SocketClient*)arg;
    bool junction = false;
    
    float baseSpeed = 5.0f;
    float setPoint = 0.0;
    
    while (c->running) {
        // ========================================
        // LINE FOLLOWING SENSORS (IR SENSORS)
        // ========================================
        // These sensors detect black lines on white surface
        // Values typically range from 0.0 to 1.0
        // Lower values indicate darker surface (line detected)
        // Higher values indicate lighter surface (no line)

        /*
           ---- line follower --------
        */
        float x = position(c->line_sensors,5);
        x = normalize(x,-2.0f,2.0f,-5.0f,5.0f);
        float error = setPoint - x;

        integral += (error*dt);

        float derivative = (error - prevError)/dt;

        float correction = Kp * error + Ki*integral + Kd * derivative;
        prevError = error;

        /*
            color detction and pick + drop box
        */
        // ========================================
        // PROXIMITY SENSOR (ULTRASONIC/DISTANCE)
        // ========================================
        // Detects obstacles or objects in front of the robot
        // Value represents distance in meters
        // Smaller values indicate closer obstacles
        // Use this for obstacle avoidance or box detection

        float proximity = c->proximity_distance;
        // if(proximity != 1) {
        //     printf("Proximity sensor distance: %.3f meters\n", proximity);
        // }
        
        // ========================================
        // COLOR SENSOR (RGB VALUES)
        // ========================================
        // Detects color of surface beneath the sensor
        // RGB values range from 0.0 to 1.0
        // Use these values to identify colored boxes or markers
        float r = c->color_r;
        float g = c->color_g;
        float b = c->color_b;

        char newColor = detectColor(r,g,b);

        if(newColor != 'N') {
            colorDetected = newColor;
        }
        
        // ========================================
        // ACTUATOR COMMANDS
        // ========================================
        // PicboxDetectedk and drop box operations
        // Implement your logic for when to pick/drop based on sensor readings

        // if(r > 0 || g > 0 || b > 0) {
        //     printf("[%.3f,%.3f,%.3f]\n",r,g,b);
        // }
        
        /*
            now detect for second junction mean the palce where the box is placed
        */
        int readingCount = thersholdreading(c,threshold_max, threshold_min);
        if(readingCount > 3) {
            destination = true;
        }
        
        /*
        ---- line follower --------
        */
       float leftChange = baseSpeed - correction;
       float rightChange = baseSpeed + correction;
       
       if(proximity <= 0.2 && proximity>0.05 && !boxPicked) {
            set_motor(c, 2.8f, 2.8f);
            SLEEP(200);  
            set_motor(c, 0, 0);
            SLEEP(200);
            pick_box(c);
            integral = 0;
            prevError = 0;
            boxPicked = true;
        }
        /*
            Detection of path according to the color 
        */
        // detect the junction 
        else if(isBlack(c) && !junction && boxPicked) {
            printf("Node is detected\n");
            SLEEP(10);
            set_motor(c,0,0);
            
            printf("color of Box: %c\n",colorDetected);
            char dir = chooseDirection(colorDetected);
            printf("Turn Direction: %c \n",dir);

            if(dir == 'L') {
                rightChange = 5.0f;
                leftChange = 1.0f;
            }
            else if(dir == 'R') {
                rightChange = 1.0f;
                leftChange = 5.0f;
            }
            else {
                rightChange = 5.0f;
                leftChange = 5.0f;
            }
            
            if(leftChange > 255) leftChange = 255;
            if(leftChange < -255) leftChange = -255;
            if(rightChange > 255) rightChange = 255;
            if(rightChange < -255) rightChange = -255;

            set_motor(c,leftChange,rightChange);

            double startTime = get_current_time();
            double duration = (dir == 'F') ? 0.2 : 1.5 ;

            while ((get_current_time() - startTime) < duration) {
                SLEEP(5);
            }

            set_motor(c,0,0);
            SLEEP(100);

            integral = 0;
            prevError = 0;
            SLEEP(20);
            junction = true;
        }
        else if(destination && boxPicked) {
            set_motor(c,0,0);
            drop_box(c);
        }
        else {
            if(leftChange < 0) leftChange = 0;
            if(leftChange > 255) leftChange = 255;
            if(rightChange < 0) rightChange = 0;
            if(rightChange > 255) rightChange = 255;
    
            set_motor(c, leftChange, rightChange);
        }

        SLEEP(50); 
    }
    return NULL;
}

/**
 * @brief Main function - Entry point of the program
 */
int main() {
    // Attempt to connect to CoppeliaSim server
    if (!connect_to_server(&client, "127.0.0.1", 50002)) {
        printf("Failed to connect to CoppeliaSim server. Make sure:\n");
        printf("1. CoppeliaSim is running\n");
        printf("2. The simulation scene is loaded\n");
        printf("3. The ZMQ remote API is enabled on port 50002\n");
        return -1;
    }
    
    printf("Successfully connected to CoppeliaSim server!\n");
    printf("Starting control thread...\n");
    
    // Start the control thread for robot behavior
#ifdef _WIN32
    HANDLE control_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)control_loop, &client, 0, NULL);
#else
    pthread_t control_thread;
    pthread_create(&control_thread, NULL, control_loop, &client);
#endif

    // Main loop: Display sensor data continuously
    printf("Monitoring sensor data... (Press Ctrl+C to exit)\n");
    while (1) {
        SLEEP(100);  // Update display every 100ms
    }

    // Cleanup
    printf("Disconnecting...\n");
    disconnect(&client);
    return 0;
}





















































