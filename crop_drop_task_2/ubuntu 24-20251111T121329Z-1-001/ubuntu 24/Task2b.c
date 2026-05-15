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
from task 2a for pick and drop one box and follow black line
*/
// float Kp = 1.2, Ki = 0.04, Kd = 0.012;   // parameter for black line 

float Kp = 1.45, Ki = 0.01, Kd = 0.012; // parameter for white line
float baseSpeed_white = 3.0f;

float Kp_b = 1.5 , Ki_b = 0.03, Kd_b = 0.01;  //parameter for black line
float baseSpeed_black = 4.0f;

float Kp_corner = 3.8 , Ki_corner = 0.0, Kd_corner = 0.01;
float baseSpeed_corner = 2.0f;

float Kp_concave = 10.0;
float Kd_concave = 0.0;
float baseSpeed_concave = 1.8f;


int CORNER_HOLD_TIME = 35;
int CORNER_HOLD_TIME_CONCAVE = 20;


bool junction = false;
bool black = false;
float setPoint = 0.0;

float integral = 0, prevError = 0;
float dt = 0.01;
unsigned long prevTime = 0;

bool boxPicked = false; // exp
int color_counter = 0; // exp
char color[3] = {'N','N','N'};
bool destination = false;
int corner_hold = 0;
int concave_hold = 0;


bool detectConcaveTurn(SocketClient* c) {
    float ir1 = c->line_sensors[0];
    float ir2 = c->line_sensors[1];
    float ir3 = c->line_sensors[2];
    float ir4 = c->line_sensors[3];
    float ir5 = c->line_sensors[4];

    // EARLY concave entry detection
    bool concaveRight = (ir3 > 0.70  &&  ir5 > 0.30  &&  ir4 > 0.10);
    bool concaveLeft = (ir3 > 0.70 && ir1 > 0.30 && ir2 > 0.10);

    return concaveRight || concaveLeft;
}


bool detectSharpCorner(SocketClient *c) {
    // Detects sharp 90-degree corners
    float ir1 = c->line_sensors[0];
    float ir2 = c->line_sensors[1];
    float ir3 = c->line_sensors[2];
    float ir4 = c->line_sensors[3];
    float ir5 = c->line_sensors[4];
    
    // Left sharp corner: both left sensors see line
    bool leftCorner = (ir1 > 0.3 && ir2 > 0.3 && ir3 < 0.25);
    
    // Right sharp corner: both right sensors see line
    bool rightCorner = (ir4 > 0.3 && ir5 > 0.3 && ir3 < 0.25);
    
    return leftCorner || rightCorner;
}


bool isJunction(SocketClient* c) {
    int count = 0;
    for(int i=0; i<5; i++) {
        if(c->line_sensors[i] > 0.02 && c->line_sensors[i] < 0.3){
            count++;
        }
    }
    return count == 5 ? true : false;
}

bool blackLine(SocketClient *c) {
    int first = c->line_sensors[2];
    int sec = 0;

    for(int i=0; i<5; i++) {
        if(c->line_sensors[i] > 0.7) {
            sec++;
        }
    }
    return (first < 0.08 && sec > 3) ? true : false;
}

float position(float sensor[],int s,bool black_line) {
    float weight[] = {-2, -1, 0, +1, +2};
    float sumSen = 0;
    float sumSenWeg = 0;

    if(black_line) {
        Kp = Kp_b, Ki = Ki_b, Kd = Kd_b; 
    }

    for(int i =0; i<s; i++) {
        float sensorValue = 0.0;
        if(black_line) {
            sensorValue = 1.0f - sensor[i];  // for the inversion of line detection from white to black
        }else if(!black_line){
            sensorValue = sensor[i];
        }
        sumSen += sensorValue;
        sumSenWeg += (sensorValue*weight[i]);
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
        if(g > 0.5 && r > 0.1 && b < 0.02) {
            color = 'R';
        }
        else if(g < 0.01 && r >= 0.1 && b > 0.3) {
            color = 'G';
        }
        else if(r > 0.1 && g < 0.08 && b < 0.08) {
            color = 'B';
        }
    }
    return color;
}


int thersholdreading(SocketClient *c) {
    int counter = 0;
    for(int i =0; i<5; i++) {
        if((c->line_sensors[i] > 0.45 && c->line_sensors[i] < 0.55) || (c->line_sensors[i] > 0.25 && c->line_sensors[i] < 0.35)) {
            counter++;
        }
    }
    return counter;
}


char chooseDirection(char boxColor) {
    char dir;
    if(boxColor == 'R' || boxColor == 'G') {
        dir = 'L';
    }
    else if(boxColor == 'B'){
        dir = 'R';   
    }
    return dir;   
}


bool validColor(char color[],char c) {
    for(int i=0; i<3; i++) {
        if(color[i] == c) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Main control loop thread for robot behavior
 */
void* control_loop(void* arg) {
    SocketClient* c = (SocketClient*)arg;
    float baseSpeed = baseSpeed_white;


    while (c->running) {
        
        float ir1 = c->line_sensors[0];  // left_corner sensor
        float ir2 = c->line_sensors[1];  // left sensor
        float ir3 = c->line_sensors[2];  // middle sensor (center)
        float ir4 = c->line_sensors[3];  // right sensor
        float ir5 = c->line_sensors[4];  // right_corner sensor
        
        // if(color[0] != 'N') {
            // printf("destination %d\n",destination);
            // printf("Junction %d \n",junction);
            // }
            
        // if(black) {
            printf("Line sensor readings: [%.2f, %.2f, %.2f, %.2f, %.2f]\n", ir1, ir2, ir3, ir4, ir5);
        // }
        

        // if(detectSharpCorner(c) && corner_hold == 0) {
        //     printf("Sharp Corner Detected \n");
        // }


        // if(detectConcaveTurn(c) && corner_hold == 0) {
        //     printf("concave Corner Detected \n");
        // }
        
        /* ----  pid calculation  ----------*/
        bool line = blackLine(c);
        // printf("Black line detected: %d",line);
        if(line && color_counter == 0 && !black) {
            black = true;
            baseSpeed = baseSpeed_black;
            integral = 0;
            prevError = 0;
            printf("Switched to Black line %d\n",line);
        }
            

        if(detectConcaveTurn(c) && concave_hold == 0 && corner_hold == 0) {
            concave_hold = CORNER_HOLD_TIME_CONCAVE;
            integral = 0.0;
            prevError = 0.0;
            // printf("Concave Turn Activated\n");
        }
        else if(detectSharpCorner(c) && corner_hold == 0 && concave_hold == 0) {
            corner_hold = CORNER_HOLD_TIME;
            integral = 0.0;
            prevError = 0.0;
            // printf("Sharp Corner Activated\n");
        }


        float current_kp,current_ki,current_kd;
        
        // printf("kp: %.2f, kd: %.2f,[ki: %.2f,baseSpeed: %.2f]\n",Kp,Kd,Ki,baseSpeed);
        if(concave_hold > 0) {
            current_kp = Kp_concave;
            current_ki = 0.0;
            current_kd = Kd_concave;
            baseSpeed = baseSpeed_concave;
            concave_hold--;
        }else if(corner_hold > 0) {
            current_kp = Kp_corner;
            current_ki = Ki_corner;
            current_kd = Kd_corner;
            baseSpeed = baseSpeed_corner;
            corner_hold--;
        }
        else if(black) {
            current_kd = Kd_b;
            current_ki = Ki_b;
            current_kp = Kp_b;
            baseSpeed = baseSpeed_black;
        }
        else {
            current_kd = Kd;
            current_ki = Ki;
            current_kp = Kp;
            baseSpeed = baseSpeed_white;
        }


        float x = position(c->line_sensors,5,black);
        x = normalize(x,-2.0f,2.0f,-1.0f,1.0f);
        float error = setPoint - x;

        integral += (error*dt);

        if(integral > 15.0f) integral = 15.0f;
        if(integral < -15.0f) integral = -15.0f;
    
 
        float derivative = (error - prevError)/dt;

        float correction = current_kp * error + current_ki*integral + current_kd * derivative;
        prevError = error;


        
       /* distance from box 0 to 1*/
        float proximity = c->proximity_distance;
        // printf("Proximity sensor distance: %.3f meters\n", proximity);

        
        // Example: Send color commands (implement your color detection logic here)
        // These are placeholder calls - implement actual color detection
        // send_color(&client, "green")
        /* detect color */
        float r = c->color_r;
        float g = c->color_g;
        float b = c->color_b;
        if(r>0 || g > 0 || b>0 && color_counter < 3) {
            // printf("Color RGB raw values: (%.3f, %.3f, %.3f)\n", r, g, b);
            char co = detectColor(r,g,b);
            if(color[color_counter] == 'N' && validColor(color,co)) {
                color[color_counter] = co;
                if(co == 'R') {
                    send_color(c,"red");
                }
                else if(co == 'G') {
                    send_color(c,"green");
                }
                else if(co == 'B') {
                    send_color(c,"blue");
                }
                color_counter++;
            }
            // printf("{%c,%c,%c}, color counter: %d\n",color[0],color[1],color[2],color_counter);
        }

        

        // pick_box(c);
        // drop_box(c);

        int readingCount = thersholdreading(c);
        if(readingCount > 4) {
            destination = true;
        }
        
        float leftChange = baseSpeed - correction;
        float rightChange = baseSpeed + correction;
       
        if(proximity <= 0.2 && proximity>0.05 && !boxPicked && color_counter == 3) {
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
        else 
        if(isJunction(c) && !junction && boxPicked && black) {
            printf("Node is detected\n");
            SLEEP(10);
            set_motor(c,0,0);
            
            char dir = chooseDirection(color[2]);
            printf("Turn Direction: %c \n",dir);

            if(dir == 'L') {
                rightChange = 5.0f;
                leftChange = 1.0f;
            }
            else if(dir == 'R') {
                rightChange = 1.0f;
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
            junction = true;
        }
        else if(destination && boxPicked && black && junction) {
            set_motor(c,0,0);
            drop_box(c);
            break;
        }
        else {
            if(leftChange < 0) leftChange = 0;
            if(leftChange > 255) leftChange = 255;
            if(rightChange < 0) rightChange = 0;
            if(rightChange > 255) rightChange = 255;
    
            set_motor(c, leftChange, rightChange);
        }

        SLEEP(50);  // Wait 5ms before next iteration
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





















































