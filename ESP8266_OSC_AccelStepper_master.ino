
/*---------------------------------------------------------------------------------------------

  Open Sound Control (OSC) library for the ESP8266/ESP32

  Example for receiving open sound control (OSC) bundles on the ESP8266/ESP32
  Send integers '0' or '1' to the address "/led" to turn on/off the built-in LED of the esp8266.

  This example code is in the public domain.

--------------------------------------------------------------------------------------------- */
#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>

char ssid[] = "your wifi";          // your network SSID (name)
char pass[] = "your password";                    // your network password

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;
const IPAddress outIp(192,168,0,216);        // remote IP (not needed for receive)
const unsigned int outPort = 9001;          // remote port (not needed for receive)
const unsigned int localPort = 8001;        // local port to listen for UDP packets (here's where we send the packets)



// Set your Static IP address
IPAddress local_IP(192, 168, 0, 101);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional


// POINTER
int myVar = 0;
int *myPointer;


OSCErrorCode error;
//unsigned int ledState = LOW;              // LOW means led is *on*
int ledState = 0;
int redValue = 0;
int greenValue = 0;
int blueValue = 0;

//#ifndef BUILTIN_LED
///#ifdef LED_BUILTIN
//#define BUILTIN_LED LED_BUILTIN
//#else
//#define BUILTIN_LED 13
//#endif
//#endif
static const uint8_t D1   = 5;
static const uint8_t D2   = 4;

static const uint8_t D5   = 14;
static const uint8_t D6   = 12;
static const uint8_t D7   = 13;



//MOTOR
#include <AccelStepper.h>

int pul = 14;
int dir = 12;
int eS1 = 5;
int eS2 = 4;

AccelStepper stepper(1,pul,dir);

int newDest;
//


void setup() {


  pinMode(14, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(14, LOW);
  digitalWrite(12, LOW);

  pinMode(eS1, INPUT);
  pinMode(eS2, INPUT);

  
int distance = 0;
//  pinMode(D1, INPUT);
//  pinMode(D2, INPUT);
//  
//  pinMode(D5, OUTPUT);
//  pinMode(D6, OUTPUT);
//  pinMode(D7, OUTPUT);
  //digitalWrite(D5, ledState);    // turn *on* led

  Serial.begin(115200);

//setup static ip
   if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  
//
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    
  }
  Serial.println("");

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
#ifdef ESP32
  Serial.println(localPort);
#else
  Serial.println(Udp.localPort());
#endif


  myPointer = &myVar;

}


void red(OSCMessage &msg) {
  *myPointer = msg.getInt(0);
  //analogWrite(D5, redValue);
  //Serial.print("/newDest = ");
  //Serial.println(myVar);
}

void green(OSCMessage &msg) {
  greenValue = msg.getInt(0);
  //analogWrite(D6, greenValue);
  Serial.print("/green = ");
  Serial.println(greenValue);
}

void blue(OSCMessage &msg) {
  blueValue = msg.getInt(0);
  //analogWrite(D7, blueValue);
  Serial.print("/blue = ");
  Serial.println(blueValue);
}






void loop() {
  
   // Continuous loop
   while (1)
 {
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(3000);

//ENDSTOP READ
float wholeSum1 = 0;
float wholeSum2 = 0;

  for(int i=0; i<10; i++){
    wholeSum1 += digitalRead(eS1);
    wholeSum2 += digitalRead(eS2);
  }

wholeSum1 /= 10;
wholeSum2 /= 10;

  if (wholeSum1 <= 0.5){
    wholeSum1 = 0;
  }
  else {
    wholeSum1 = 1;
  }

  if (wholeSum2 <= 0.5){
  wholeSum2 = 0;
}
  else {
    wholeSum2 = 1;
  }
  
int endStop1 = int(wholeSum1);
int endStop2 = int(wholeSum2);
//Serial.print(endStop1);
//Serial.println(endStop2);

//ENDSTOP READ




//if (state == 0){
//  stepper.stop();
//
//}
//STEPPER RUN



  OSCBundle bundle;
  int size = Udp.parsePacket();

  if (size > 0) {
    while (size--) {
      bundle.fill(Udp.read());
    }
    if (!bundle.hasError()) {
      bundle.dispatch("/red", red);
      //newDest = myVar;
    } 
    if (!bundle.hasError()) {
      bundle.dispatch("/green", green);
    } 
    if (!bundle.hasError()) {
      bundle.dispatch("/blue", blue);
    } 
    

   //ENDSTOP PROTECTION
     stepper.moveTo(myVar);
int state = 1;
    long cPos = stepper.currentPosition();
    long tPos = stepper.targetPosition();
    long dtg = stepper.distanceToGo();
    if (endStop1 == 0 && cPos-tPos > 0){
      state *= 0;
    }


    if (endStop2 == 0 && cPos-tPos < 0){
      state *= 0;
    }
//ENDSTOP PROTECTION
Serial.println(state);

//STEPPER RUN

if (state > 0){
  stepper.run();
}

if (state <=0){
  stepper.stop();
  stepper.moveTo(cPos); // current position is the new target position
  stepper.setMaxSpeed(0); // flush movement speed data, stops into a new static position

}

       }
       //WHILE(1)
    }
    //VOID LOOP
