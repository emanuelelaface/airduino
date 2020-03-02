/*
Temperature/Humidity/Pressure sensor BME280
Pin Connections
Vcc -> 3.3v
GND -> GND
SCL -> A5
SDA -> A4
CBS -> N.C.
SDD -> N.C.
*/

/*
CO2 MH-Z14A sensor
Pin Connections
17 -> Vin
16 -> GND
18 -> Tx
19 -> Rx
*/

/*
RGB LED
Pin Connections
RED -> D2
GREEN -> D3
BLUE -> D4 
*/

#include <WiFiNINA.h> // Library used for the WiFi
#include <WiFiUdp.h> // Library used for UDP connection (NTP)
#include <time.h> // Time Library
#include <Adafruit_BME280.h> // Library used for the Temperature, Humidity, Pressure Sensor

#define ssid "WRITE HERE YOUR WIFI NAME" // WIFI Name (SSID)
#define pass "WRITE HERE YOUR WIFI PASSWORD" // WIFI Password

#define RLED 2 // Pin for RED LED
#define GLED 3 // Pin for GREEN LED
#define BLED 4 // Pin for BLUE LED

#define HISTSIZE 100 // Size of the arrays that will keep the history of measurements

/* The time will be in UTC, I add 1 hour (3600 seconds) because I live in GMT+1 */
#define TIMECORR 3600

/* Data structures used for the NTP request */
unsigned int localPort = 2390; // Local port to bind
IPAddress timeServer(17, 253, 54, 123); // IP of the NTP time server
const int NTP_PACKET_SIZE = 48; // NTP Packet Size
byte packetBuffer[ NTP_PACKET_SIZE]; // Buffer that will contain the NTP Request
WiFiUDP Udp; // UDP Client Structure for NTP request

/* WIFI server variables */
int status = WL_IDLE_STATUS; // WIFI Status
WiFiServer server(80); // Create Web Server structure with on port 80

/* Create structure for Temperature, Humidity, Pressure Sensor */
Adafruit_BME280 bme; // This way to initalize uses I2C mode

/* Commands to request CO2 reading from the CO2 sensor */
const byte requestCO2Reading[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
byte resultCO2[9];

/* Variables to store the readings from the four sensors */
int co2;
float temp;
float hum;
float pres;

/* Arrays to store the history of readings for graphs */
float hist_co2[HISTSIZE];
float hist_temp[HISTSIZE];
float hist_hum[HISTSIZE];
float hist_pres[HISTSIZE];

/* Variables to store the current time and the time history for graph */
time_t now = 0; // This contains time of the reading
struct tm ts; // Used to store the time struct
char str_now[80]; // Used to convert time in a string
unsigned long hist_time[HISTSIZE]; // Array of time used for the graph
unsigned long last_update; // Store the last time the arrays were updated
int counter; // Used to keep track of how many readings there are in a time period to average the value in the arrays
int time_interval = 864; // Time to average the reading. It is such that the HISTSIZE of 100 points is 1 day of readings.

int greenOn; // When is 1 the green led is on
int alarmOn; // When is 1 the blue led is on
int co2_threshold = 1000; // 1000 ppm of CO2 triggers the alarmOn

int i; // Generic index

void setup() {
  /* Initalize the LEDs */
  pinMode(RLED, OUTPUT);
  pinMode(GLED, OUTPUT);
  pinMode(BLED, OUTPUT);
  digitalWrite(RLED, LOW);
  digitalWrite(GLED, LOW);
  digitalWrite(BLED, LOW);

  /* Wait for the wifi on */
  while (status != WL_CONNECTED) {
    digitalWrite(RLED, HIGH); // While it is not connected blink the RED LED
    status = WiFi.begin(ssid, pass);
    delay(500);
    digitalWrite(RLED, LOW);
    delay(500);
  }

  digitalWrite(RLED, LOW); // Once WIFI is connected turn off the RED LED
  /* Initialize the humidity/temperature/pressure sensors */
  bme.begin();
  /* Initialize the serial port for CO2 sensor */
  Serial1.begin(9600); // Arduino IOT 33 has 2 serials, this is Serial 1, the Serial 0 is the one connected to micro USB port!
  delay(500);
  for (i = 0; i < 9; i++) {
    Serial1.write(requestCO2Reading[i]); // Write a blank CO2 request to initalize the sensor
  }
  delay(500);
  while (Serial1.available()) {
    Serial1.read(); // Read the sensor to empty the serial buffer
    delay(100);
  }

  /* Start the Web Server */
  server.begin();

  /* Start the NTP connection */
  Udp.begin(localPort);
  
  greenOn = 3; // Everything is ok, so turn the green led on for 3 loop cycles
  alarmOn = 0; // The alarm is off

  /* Set the variables and the arrays to zero */
  co2=0;
  temp=0;
  hum=0;
  pres=0;
  now = 0;

  for (i=0; i<HISTSIZE; i++) {
    hist_co2[i]=0;
    hist_temp[i]=0;
    hist_hum[i]=0;
    hist_pres[i]=0;
    hist_time[i]=0;
  }
  last_update = getTime(timeServer)+TIMECORR; // Setup the initial time
  counter = 1; // Set the cycle counter to 1
}

/* Start the idle loop */
void loop() {
  /* This manage the on/off of the green LED */
  if (greenOn > 0){
    digitalWrite(GLED, HIGH);
    greenOn = greenOn - 1;
  }
  else{
    digitalWrite(GLED, LOW);
  }

  /* If the CO2 is above the threshold, the alarm is turned on until the level
     returns below the threshold */
  if (co2 >= co2_threshold and alarmOn == 0){
    digitalWrite(BLED, HIGH);
    alarmOn = 1;
  }
  if (co2 < co2_threshold and alarmOn == 1){
    digitalWrite(BLED, LOW);
    alarmOn = 0;
  }

  pres = bme.readPressure() / 100.0F; // Read the pressure and normalize
  hum = bme.readHumidity(); // Read the Humiditu
  temp = bme.readTemperature(); // Read the temperature
  co2 = getCO2(); // Read the CO2
  now = getTime(timeServer)+TIMECORR; // Get the current time 

  /* If one time_interval passed, roll the array of one position*/
  if (now-last_update > time_interval) {
    for (i=0;i<HISTSIZE-1;i++){
      hist_co2[i]=hist_co2[i+1];
      hist_temp[i]=hist_temp[i+1];
      hist_hum[i]=hist_hum[i+1];
      hist_pres[i]=hist_pres[i+1];
      hist_time[i]=hist_time[i+1];
    }
    last_update = now;
    counter = 1;
  }
  /* If the time_interval is not passed, store the average of the values in the last position of the arrays */
  else {
    hist_co2[HISTSIZE-1] = (hist_co2[HISTSIZE-1]*(counter-1) + (float)co2)/counter;
    hist_temp[HISTSIZE-1] = (hist_temp[HISTSIZE-1]*(counter-1) + temp)/counter;
    hist_hum[HISTSIZE-1] = (hist_hum[HISTSIZE-1]*(counter-1) + hum)/counter;
    hist_pres[HISTSIZE-1] = (hist_pres[HISTSIZE-1]*(counter-1) + pres)/counter;
    hist_time[HISTSIZE-1] = now;
    counter += 1;
  }

  WiFiClient client = server.available(); // Check if a client requested the Web Server
  if (client) { // If it requested
    boolean currentLineIsBlank = true;
    while (client.connected()) { // Check that the client is connected
      if (client.available()) { // Check that it is avaialble
        char c = client.read(); // Read the HTTP request
        if (c == '\n' && currentLineIsBlank) { // Until the new line
          /* Create a standard header for the answer serving the JSON as type */
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type: application/json");
          client.println("Access-Control-Allow-Origin: *");
          client.println("Access-Control-Allow-Headers: Authorization, Content-Type");
          client.println("Connection: close");
          client.println(); // End of the header
          
          /* Start the JSON */
          client.print("{");
          
          /* Send pressure */
          client.print("\"pressure\": ");
          client.print("{\"value\": ");
          client.print(pres);
          client.print(", \"unit\":");
          client.print("\"mBar\"");
          client.print("}");
          
          client.print(",");
          
          /* Send humidity */
          client.print("\"humidity\": ");
          client.print("{\"value\": ");
          client.print(hum);
          client.print(", \"unit\":");
          client.print("\"%\"");
          client.print("}");

          client.print(",");

          /* Send temperature */
          client.print("\"temperature\": ");
          client.print("{\"value\": ");
          client.print(temp);
          client.print(", \"unit\":");
          client.print("\"\C\"");
          client.print("}");

          client.print(",");

          /* Send CO2 */ 
          client.print("\"co2\": ");
          client.print("{\"value\": ");
          client.print(co2);
          client.print(", \"unit\":");
          client.print("\"ppm\"");
          client.print("}");

          client.print(",");

          /* Send time as string */
          client.print("\"epoch\": ");
          client.print("{\"value\": \"");
          ts=*localtime(&now);
          /* Format time with the format Mon 2020-03-02 22:27:31 */
          strftime(str_now, sizeof(str_now), "%a %Y-%m-%d %H:%M:%S", &ts); 
          client.print(str_now);
          client.print("\", \"unit\":");
          client.print("\"s\"");
          client.print("}");

          client.print(",");

          /* Send the temperature history as pairs of [time, temperature].
             The time is in UNIX time in milliseconds because the plot tool converts that format authomatically,
             so the 000 are used to mupliply the time from seconds to milliseconds.
             The last point is sent separately because it closes the array in a different way.*/
          client.print("\"hist_temp\": ");
          client.print("{\"value\": ");
          client.print("[");
          for (i=0; i<HISTSIZE-1; i++){
            if (hist_temp[i] != 0){
              client.print("[");
              client.print(hist_time[i]);
              client.print("000, ");
              client.print(hist_temp[i]);
              client.print("], ");
            }
          }
          client.print("[");
          client.print(hist_time[HISTSIZE-1]);
          client.print("000, ");
          client.print(hist_temp[HISTSIZE-1]);
          client.print("]]");
          client.print(", \"unit\":");
          client.print("\"\C\"");
          client.print("}");

          client.print(",");

          /* Send the humidity history as pairs of [time, humidity].
             The time is in UNIX time in milliseconds because the plot tool converts that format authomatically,
             so the 000 are used to mupliply the time from seconds to milliseconds.
             The last point is sent separately because it closes the array in a different way.*/
          client.print("\"hist_hum\": ");
          client.print("{\"value\": ");
          client.print("[");
          for (i=0; i<HISTSIZE-1; i++){
            if (hist_hum[i] != 0){
              client.print("[");
              client.print(hist_time[i]);
              client.print("000,");
              client.print(hist_hum[i]);
              client.print("], ");
            }
          }
          client.print("[");
          client.print(hist_time[HISTSIZE-1]);
          client.print("000,");
          client.print(hist_hum[HISTSIZE-1]);
          client.print("]]");
          client.print(", \"unit\":");
          client.print("\"%\"");
          client.print("}");

          client.print(",");

          /* Send the CO2 history as pairs of [time, co2].
             The time is in UNIX time in milliseconds because the plot tool converts that format authomatically,
             so the 000 are used to mupliply the time from seconds to milliseconds.
             The last point is sent separately because it closes the array in a different way.*/
          client.print("\"hist_co2\": ");
          client.print("{\"value\": ");
          client.print("[");
          for (i=0; i<HISTSIZE-1; i++){
            if (hist_co2[i] != 0){
              client.print("[");
              client.print(hist_time[i]);
              client.print("000,");
              client.print(hist_co2[i]);
              client.print("], ");
            }
          }
          client.print("[");
          client.print(hist_time[HISTSIZE-1]);
          client.print("000,");
          client.print(hist_co2[HISTSIZE-1]);
          client.print("]]");
          client.print(", \"unit\":");
          client.print("\"ppm\"");
          client.print("}");

          client.print(",");
          
          /* Send the pressure history as pairs of [time, pressure].
             The time is in UNIX time in milliseconds because the plot tool converts that format authomatically,
             so the 000 are used to mupliply the time from seconds to milliseconds.
             The last point is sent separately because it closes the array in a different way.*/
          client.print("\"hist_pres\": ");
          client.print("{\"value\": ");
          client.print("[");
          for (i=0; i<HISTSIZE-1; i++){
            if (hist_pres[i] != 0){
              client.print("[");
              client.print(hist_time[i]);
              client.print("000,");
              client.print(hist_pres[i]);
              client.print("], ");
            }
          }
          client.print("[");
          client.print(hist_time[HISTSIZE-1]);
          client.print("000,");
          client.print(hist_pres[HISTSIZE-1]);
          client.print("]]");
          client.print(", \"unit\":");
          client.print("\"mBar\"");
          client.print("}");
         
          client.println("}"); // Close the JSON dictionary
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    delay(1);
    client.stop(); // Ends the communication
  }

  /* Every loop check if the WIFI is connected. If not, turn on the RED LED and try to reconnect */
  status = WiFi.status();
  while (status != WL_CONNECTED) {
    digitalWrite(RLED, HIGH);
    status = WiFi.begin(ssid, pass);
    delay(500);
    digitalWrite(RLED, LOW);
    delay(500);
  }
  delay(1000);
}

/* Function to get the CO2 value from the serial port */
int getCO2() {
  for (i = 0; i < 9; i++) {
    Serial1.write(requestCO2Reading[i]); // Write the bytes sequence for the request
  }
  while (Serial1.available() < 9) {};
  for (i = 0; i < 9; i++) {
    resultCO2[i] = Serial1.read(); // Read the answer
  }
  int high = resultCO2[2];
  int low = resultCO2[3];
  return high * 256 + low; // Convert the answer in a human readable value
}

/* Routine to get the time from the NTP Server */
unsigned long getTime(IPAddress& address) {
  /* Prepare the packet */
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  Udp.beginPacket(address, 123); // NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE); // Send the packet
  Udp.endPacket();
  delay(1000); // Wait one second
  if (Udp.parsePacket()) { // If there is an answer then read and parse the result
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    const unsigned long seventyYears = 2208988800UL;
    unsigned long epoch = secsSince1900 - seventyYears;
    return epoch;
  }
  return 0;
}
