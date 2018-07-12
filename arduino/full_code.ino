//import the libraries
#include <WiFi.h>
#include <string.h>
#include <Arduino.h>
#include <mavlink.h>

/* ESP32 declarations */
#define LED_BUILTIN 2 //Blue
int armLED = 12; //Yellow
int throttleLED = 32; //Green
int connectionLED = 26; //White
int buttonPin = 14; //Pushbutton's input pin
int sensorPin = 15; //Pin for an output from the lightsensor
int buttonState = 0; //Variable for reading the pushbutton status
int sensorValue = 0; //Output from the lightsensor

//Reflects connection with the ESP32
static const String PINGING = "PINGING";
//Reflects connection with the drone
static const String HEARTBEATING = "HEARTBEATING";
//Shows drone's state
static const String ARMED = "ARMED";
static const String DISARMED = "DISARMED";
//Sets current flight mode
static const String SET_FLIGHT_MODE_ALTHOLD = "SET_FLIGHT_MODE_ALTHOLD";
static const String SET_FLIGHT_MODE_LOITER = "SET_FLIGHT_MODE_LOITER";
static const String SET_FLIGHT_MODE_STABILIZE = "SET_FLIGHT_MODE_STABILIZE";
static const String SET_FLIGHT_MODE_AUTO_RETURN = "SET_FLIGHT_MODE_AUTO_RETURN";

static const String STABILIZE = "STABILIZE";
static const String ALTHOLD = "ALTHOLD";
static const String LOITER = "LOITER";

//Sets rotors values
static const String SET_ROLL = "SET_ROLL_";//+ int
static const String SET_PITCH = "SET_PITCH_";//+ int
static const String SET_THROTTLE = "SET_THROTTLE_";//+ int
static const String SET_YAW = "SET_YAW_";//+ int
//Sets elementary actions
static const String SET_ARM = "SET_ARM";
static const String SET_DISARM = "SET_DISARM";

//Configuration of the drone
boolean current_arm = false;
String current_mode = STABILIZE;
int current_roll = 0;
int current_pitch = 0;
int current_throttle = 1150; //Min 1150 to run motors
int current_yaw = 0;

HardwareSerial SerialMAV(2); //default pins for 16RX, 17TX

//Define softAP info
const char *ssid = "MYESP32"; //name
const char *password = "testpassword"; //pass

// How often to ping (milliseconds)
static const int PING_TIME = 1000;

// Special messages
static const String PING_MSG = "SOCKET_PING";
static const String CONNECTED_MSG = "SOCKET_CONNECTED";

//Serial messages
static const String LED_ON = "on";
static const String LED_OFF = "off";
static const String SLIDER = "SLIDER_"; //value after "_" is a expected int

//ledc values
int freq = 5000;
int ledChannel = 0;
int resolution = 8;

//Init Server at port 80
WiFiServer server(80);

/* MAVLINK configuration */
int sysid = 255;//GCS                   ///< ID 20 for this airplane. 1 PX, 255 ground station
int compid = 190;//Mission Planner                ///< The component sending the message
int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing

// Define the system type, in this case an airplane -> on-board controller
uint8_t system_type = MAV_TYPE_GENERIC;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

// Hardware definitions
uint8_t system_mode = MAV_MODE_TEST_ARMED; /// /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
uint32_t custom_mode = MAV_MODE_FLAG_SAFETY_ARMED; ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from APM. 60 = one minute.
int num_hbs_past = num_hbs;

//Default Arduino function
void setup() {
  //Initialize led on ESP32
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(armLED, OUTPUT);
  pinMode(connectionLED, OUTPUT);

  pinMode(buttonPin, INPUT);
    
  //Start human-readable console
  Serial.begin(115200);
  //Run softAP
  initSoftAp();
  //Run server
  server.begin();
  //Start drone's serial connection
  //16, 17 corresponds to the ESP32 serial pins
  SerialMAV.begin(57600, SERIAL_8N1, 16, 17);
  //Show inital message
  Serial.println("Started listening...");
}

//Default Arduino function
void loop() {
  //Check for connected devices to the softAP
  checkIfConnected();
  //Looks for a connected client
  WiFiClient client = server.available();
  //Control connected client
  runWifiClient(client);

  /*
   * Code for the arming red button
   */

  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) {
    current_arm = true;
  } else {
    current_arm = false;
  }

  /*
   * Code for the motor controlled via lightsensor
   */

   sensorValue = analogRead(sensorPin);
   //Serial.println(sensorValue);
  
  // Initialize the required buffers
  mavlink_rc_channels_override_t sp;
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  delay(100);

  /*
   * Put the configuration in loop. The config changes async. reacting
   * on app's commands.
   */

  //We have to have the heartbeats to indicate side by side connection
  mav_heartbeat_pack();

  mav_set_mode(current_mode);

  mav_arm_pack(current_arm);

  // ROLL, PITCH, THROTTLE, YAW
  mav_override_rc(current_roll, current_pitch, current_throttle, current_yaw);

  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    // Timing variables
    previousMillisMAVLink = currentMillisMAVLink;

    //SerialMAV.write(buf, len);

    //Mav_Request_Data();
    num_hbs_past++;
    if(num_hbs_past>=num_hbs) {
      // Request streams from APM
      Mav_Request_Data();
      num_hbs_past=0;
    }
  }

  // Check reception buffer
  comm_receive();
}

/* This function gets message from the APM and interprete for Mavlink common messages */
void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  //Indicates no data input
  digitalWrite(LED_BUILTIN, LOW);

  //Shows if the drone is armed
  if (current_arm){
    digitalWrite(armLED, HIGH);
  }else {
    digitalWrite(armLED, LOW);
  }

  //Checks if drone is connected
  while(SerialMAV.available()) {
    uint8_t c = SerialMAV.read();
    //Indicates data receive frequency
    digitalWrite(LED_BUILTIN, HIGH);
        
    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      //Indicates data flow
      Serial.println("+");
      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
            //Serial.println("MAVLINK_MSG_ID_HEARTBEAT");
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            Serial.print("State: "); Serial.println(hb.base_mode == 209 ? "Armed" : "Disarmed");
            Serial.print("Mode: ");
            switch(hb.custom_mode) {
              case 0:
                Serial.println("Stabilize");
              break;
              case 2:
                Serial.println("AltHold");
              break;
              case 3:
                Serial.println("Auto");
              break;
              case 5:
                Serial.println("Loiter");
              break;
              case 7:
                Serial.println("Circle");
              break;
              default:
                Serial.println("Mode not known");
              break;
            }
            //Stablize = 0
            //AltHold = 2
            //Auto = 3
            //Loiter = 5
            //Circle = 7
          }
          break;
        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
             */
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
            //Serial.println("MAVLINK_MSG_ID_SYS_STATUS");
            //Serial.println("Battery (V): ");
            //Serial.println(sys_status.voltage_battery);
          }
          break;
        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
             */
            //mavlink_message_t* msg;
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&msg, &param_value);
            //Serial.println("MAVLINK_MSG_ID_PARAM_VALUE");
          }
          break;
        case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
          {
            /* Message decoding: PRIMITIVE
             *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
             */
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
            //Serial.println("MAVLINK_MSG_ID_RAW_IMU");
          }
          break;
        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
             */
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);
            //Serial.println("MAVLINK_MSG_ID_ATTITUDE");
            //Serial.println("ROLL: ");
            //Serial.println(attitude.roll);
          }
          break;
        case MAVLINK_MSG_ID_SET_MODE: // #11
          {
            mavlink_set_mode_t set_mode;
            mavlink_msg_set_mode_decode(&msg, &set_mode);
            /*
            Serial.println("CUSTOM_MODE: ");
            Serial.println(set_mode.custom_mode);
            Serial.println("TARGET_SYSTEM: ");
            Serial.println(set_mode.target_system);
            Serial.println("BASE_MODE: ");
            Serial.println(set_mode.base_mode);
            */
          }
          break;
          //Not overriden channels
          case MAVLINK_MSG_ID_RC_CHANNELS_RAW: // #35
          {
           /* 
           *  RC (Radio controll) channels are the inputs and outputs for controlling all 
           *  actions called from joystick / mission planner. E.g. arm, throttle, pitch.
           */ 
            mavlink_rc_channels_raw_t chs;
            mavlink_msg_rc_channels_raw_decode(&msg, &chs);

            Serial.print("Roll: ");  Serial.print(chs.chan1_raw);
            Serial.println();
            Serial.print("Pitch: ");  Serial.print(chs.chan2_raw + '\n');
            Serial.println();
            Serial.print("Throttle: ");  Serial.print(chs.chan3_raw + '\n');
            Serial.println();
          }
          break;
          //Overriden channels for radio values
          case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: // #70
          {
            mavlink_rc_channels_override_t ov_chs;
            mavlink_msg_rc_channels_override_decode(&msg, &ov_chs);
  
            Serial.print("Overr. Roll: ");  Serial.print(ov_chs.chan1_raw);
            Serial.println();
            Serial.print("Overr. Pitch: ");  Serial.print(ov_chs.chan2_raw + '\n');
            Serial.println();
            Serial.print("Overr. Throttle: ");  Serial.print(ov_chs.chan3_raw + '\n');
            Serial.println();
          }
          break;
       default:
            /*
            Serial.println("Mavlink message: ");
            Serial.print("#");
            Serial.print(msg.msgid);
            Serial.println("");
            */
          break;
      }
    }
  }
}

void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 1;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_ALL};
  const uint16_t MAVRates[maxStreams] = {0x02};
    
  for (int i=0; i < maxStreams; i++) {
    /*
     * mavlink_msg_request_data_stream_pack(system_id, component_id, 
     *    &msg, 
     *    target_system, target_component, 
     *    MAV_DATA_STREAM_POSITION, 10000000, 1);
     *    
     * mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, 
     *    mavlink_message_t* msg,
     *    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, 
     *    uint16_t req_message_rate, uint8_t start_stop)
     * 
     */
     
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);    
    SerialMAV.write(buf, len);
  }
}

void mav_heartbeat_pack() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
  mavlink_msg_heartbeat_pack(255,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void mav_arm_pack(boolean state) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  //Arm the drone
  //400 stands for MAV_CMD_COMPONENT_ARM_DISARM
  // 1 an 8'th argument is for ARM (0 for DISARM)
  if(state) {
    //ARM
    mavlink_msg_command_long_pack(0xFF, 0xBE, &msg, 1, 1, 400, 1,1.0,0,0,0,0,0,0);
  }else {
    //DISARM
    mavlink_msg_command_long_pack(0xFF, 0xBE, &msg, 1, 1, 400, 1,0.0,0,0,0,0,0,0);
  }
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void mav_set_mode(String value) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  value.trim();

  //SET_MODE
  //Works with 1 at 4'th parameter
  if (value == STABILIZE){
    mavlink_msg_set_mode_pack(0xFF, 0xBE, &msg, 1, 209, 0);
  }

  if (value == ALTHOLD){
    mavlink_msg_set_mode_pack(0xFF, 0xBE, &msg, 1, 209, 2);
  }

  if (value == LOITER){
    mavlink_msg_set_mode_pack(0xFF, 0xBE, &msg, 1, 209, 5);
  }
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);  
}

void mav_override_rc(int roll, int pitch, int throttle, int yaw) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_rc_channels_override_pack(0xFF, 0xBE, &msg, 1, 1, roll, pitch, throttle, yaw, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

/**
 * Section for the Serial connection with an Android app. The app you can find here:
 * 
 * https://play.google.com/store/apps/details?id=com.czterysery.drone.drone
 * 
 * Based on https://github.com/hmartiro/android-arduino-wifi
 * 
 */

void interpreteMsg(String msg){
  //Clean the message before comparing
  msg.trim();
  
  if (msg == LED_ON) {
    Serial.println("LED ON !");
    digitalWrite(connectionLED, HIGH);
  }

  if (msg == LED_OFF) {
    Serial.println("LED OFF !");
    digitalWrite(connectionLED, LOW);
  }

  
  if (stringContains(msg, SLIDER) >= 0){
    msg.replace(SLIDER, "");
  
    int numbVal = msg.toInt();

    int scaledVal = numbVal * 2.5;

    Serial.println(scaledVal);

    ledcWrite(ledChannel, scaledVal);
    delay(7);
  }
}

/**
 * Called whenever a newline-delimited message is received.
 */
void got_message(String msg) {
  interpreteMsg(msg);
  Serial.println("[RX] " + msg);
}

/**
 * Send a message to the given client.
 */
void send_message(WiFiClient client, String msg) {
  client.println(msg);
  Serial.println("[TX] " + msg);
}

/**
 * Function that compares two strings 
 * and returns number of matches. 
 */
int stringContains(String s, String search) {
    int max = s.length() - search.length();
    int lgsearch = search.length();

    for (int i = 0; i <= max; i++) {
        if (s.substring(i, i + lgsearch) == search) return i;
    }

 return -1;
}


int readValueFromMsg(String message){
  
}

/**
 * Functions that configure and runs 
 * WiFi Access Point that devices can connect to
 */
void initSoftAp(){
  Serial.print("Setting Access Point...");
  //Start propagating the network
  Serial.println("Running Access Point");
  //channel = 1, not hided, max_connection = 1
  Serial.println(WiFi.softAP(ssid, password, 1, 0, 1) ? "Ready" : "Failed!");
  Serial.println("WiFi address: ");
  //Shows the ESP32 IP
  Serial.println(WiFi.softAPIP());
}

/**
 * setup analogWrite on esp32
 * ledc is an equivalent of Arduino's analogWrite()
 */
void initPWMLed(){
  //Set pin in output mode
  pinMode(throttleLED, OUTPUT);
  //setup values
  ledcSetup(ledChannel, freq, resolution);
  //attach pin to the channel
  ledcAttachPin(throttleLED, ledChannel);
  delay(10);
}

/**
 * Function that indicates via built-in led, 
 * a connected device 
 */
void checkIfConnected() {
  //Check for the active connection
  if(WiFi.softAPgetStationNum() >= 1){
    digitalWrite(connectionLED, HIGH);
  }else{
    digitalWrite(connectionLED, LOW);
  }
}

/**
 * Function that menages current connection.
 * It is responsible for controlling 
 * data flow and stability of the connection.
 * 
 */

void runWifiClient(WiFiClient client){
  if (client) {
    
    // Buffer to store data
    String rx_buffer = "";
    String tx_buffer = "";
    
    // Last time a ping was sent
    int senttime = millis();

    Serial.println("Connected to client.");
    client.println(CONNECTED_MSG);
    
    while (client.connected()) {
      
      // Read data from the client, and when we hit a newline,
      // invoke the message_received callback and echo the message
      // back to the client.
      if(client.available()) {
        while(client.available()) {
          
          char c = client.read();
          
          if(c == '\n') {
            got_message(rx_buffer);
            send_message(client, rx_buffer);
            rx_buffer = "";
            
          } else {
            rx_buffer += c;
          }
        }
      }
      
      // Send messages from Serial to the connected client
      // Delimited by newlines
      if(Serial.available()) {
        while(Serial.available()) {
          
          char c = Serial.read();
          
          if(c == '\n') {
            send_message(client, tx_buffer);
            tx_buffer = "";
          } else tx_buffer += c;
        }
      }
      
      // Send regular ping messages to keep the connection open
      int now = millis();
      if (now - senttime > PING_TIME) {
        client.println(PING_MSG);
        senttime = now;
      }
    }
    
    // Give the client time to receive data before closing
    Serial.println("Disconnecting from client.");
    delay(100);
    client.flush();
    client.stop();
  }
}
