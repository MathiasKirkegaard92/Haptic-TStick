#include <Wire.h>
// WiFi ESP32
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <dummy.h>  //for esp32
#include <Thread.h>
#include <ThreadController.h>

#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include "Capsense.h"
#include "DRV2605.h"
#include "AHRS.h"
#include "Haptic.h"

// Debug & calibration definitions
#define DEBUG true
#define CALIB false

///////////////////////////
// Pin definitions
//////////////////////////
const int pressurePin = 33;
const int piezoPin = 32;
const int ledPin = 12;
const int buttonPin = 26;
const int sda1Pin = 21;
const int scl1Pin = 22;
const int sda2Pin  = 5;
const int scl2Pin = 4;
const int pwm1Pin = 14;
const int pwm2Pin = 15;


class Led {
public:
  Led(int the_pin): status(0), pin(the_pin) {pinMode(pin, OUTPUT); digitalWrite(pin, status);};
  int pin, status;
  void update() {digitalWrite(pin, status); status = (status + 1) % 2;};
};

class Button {
public:
  Button(int the_pin, int the_debounceDelay): status(0), pin(the_pin), lastDebounceTime(0), debounceDelay(the_debounceDelay) {pinMode(pin, INPUT);};
  int pin, status, debounceDelay, mode;
  long lastDebounceTime;
  void update() {
    if (!status) {
      status = digitalRead(pin);
    } else if (millis() - lastDebounceTime > debounceDelay) {
      lastDebounceTime = millis();
      status = digitalRead(pin);
      if (!status) {
        mode++;
        mode %= 2;
        Serial.println("released");
      }
    };

  };
};

class AnalogSensor {
public:
  AnalogSensor(int the_pin): data(0), pin(the_pin) {};
  int pin, data;
  void update() {data = analogRead(pin);};
};


///////////////////////////
// WiFi & OSC settings
//////////////////////////

const char *ssid = "SopraninoWiFi-192";           //Ap SSID
const char *password = "password";                //Ap Password
const int numParam = 7;
bool sendOSC = true;
WiFiUDP oscEndpoint;            // A UDP instance to let us send and receive packets over UDP
IPAddress oscEndpointIP;        // remote IP - your computer
int oscEndpointPORT;            // remote port to receive OSC
const unsigned int portLocal = 8888;
static int bufferFromHost[4] = {0, 0, 0, 0};
OSCMessage msg[numParam] = OSCMessage();

typedef struct {char * adress; int length;} oscMsg_t;
oscMsg_t msgMeta[numParam] = {{"/rawcapsense", 2}, {"/rawpressure", 1}, {"/rawpiezo", 1}, {"/q", 4}, {"/rawacc", 3}, {"/latch", 1}, {"/rawgyro", 3}};


///////////////////////////
// Update intervals [ms]
//////////////////////////

int touchInterval = 15;
int AHRSinterval = 30;
int oscInterval  = 30;
int oscSlowInterval = 100;
int analogSensorInterval = 50;
int ledInterval = 500;
int hapticInterval = 10;

///////////////////////////
// Initializations
//////////////////////////

// I2C init - 2 channels
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

// Init 2 instances of drv2605 (haptic driver)
DRV2605 drv1;
DRV2605 drv2;

// Init stereo haptic feedback channel
HapticStereo haptic = HapticStereo(&drv1, &drv2, pwm1Pin, pwm2Pin);

// Init Marg sensor
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
AHRS ahrs = AHRS(&lsm, 1000 / AHRSinterval);

// Init capsense
Capsense capsense(255, 255, &I2Cone);

// Init button
Button latch = Button(buttonPin, 100);

// Init led
Led led = Led(ledPin);

// Init analog sensors
AnalogSensor piezo    = AnalogSensor(piezoPin);
AnalogSensor pressure = AnalogSensor(pressurePin);


// Init thread controller
ThreadController controll = ThreadController();

// Init threads
Thread capsenseThread     = Thread();
Thread hapticThread       = Thread();
Thread oscThread          = Thread();
Thread oscSlowThread      = Thread();
Thread AHRSThread         = Thread();
Thread ledThread          = Thread();
Thread analogSensorThread = Thread();


// Callbacks
void ledCallback() {
  led.update();
}

void hapticCallback() {

  float jerk = ahrs.getAccJerkMag();
  float * orientation = ahrs.getOrientation();
  // Serial.println(orientation[0]);
  haptic.update(jerk, orientation[0], ahrs.getGyroMag(), latch.mode);
  ahrs.update(CALIB);

  //  Send jerk over serial for lower latency
  // Serial.println(jerk * 1000, DEC);
  // Serial.print(" ");

}

void analogSensorCallback() {
  piezo.update();
  pressure.update();
  latch.update();
}

void capsenseCallback() {
  capsense.update();
}


void AHRSCallback() {
}

void oscCallback() {
  if (sendOSC) {
    // AHRS orientation
    OSCMsgSet(msg[3], ahrs.getOrientation(), 3, 0);
    // AHRS jerk
    OSCMsgSet(msg[3], ahrs.getAccJerkMag(), 3);

    OSCMsgSend(msg[3], oscEndpointIP, oscEndpointPORT);
    OSCMsgSend(msg[4], ahrs.getAcc(), 3, oscEndpointIP, oscEndpointPORT);
    OSCMsgSend(msg[6], ahrs.getGyro(), 3, oscEndpointIP, oscEndpointPORT);

    // Capacitive touch
    if (capsense.changed) OSCMsgSend(msg[0], capsense.readTouch(), 2, oscEndpointIP, oscEndpointPORT);
    // Pressure
    OSCMsgSend(msg[1], &pressure.data, 1, oscEndpointIP, oscEndpointPORT);
    float * tmp = ahrs.getOrientation();
    // Serial.println(tmp[0]);
    // Serial.print(",");
    // Serial.print(tmp[1]);
    // Serial.print(",");
    // Serial.println(tmp[2]);
  }

}

void oscSlowCallback() {
  if (sendOSC) {
    // Piezo
    OSCMsgSend(msg[2], &piezo.data, 1, oscEndpointIP, oscEndpointPORT);
//  // Button
    OSCMsgSend(msg[5], &latch.mode, 1, oscEndpointIP, oscEndpointPORT);
    Serial.print("Port :");
    Serial.println(oscEndpointPORT);
    Serial.print("IP :");
    Serial.println(oscEndpointIP);
  }

}


void setup() {

  Serial.begin(115200);

  // OBS:: Put in happticStereo
  drv1.setupLRA(&I2Cone, 21, 22);
  delay(1000); // Wait for other motor to brake
  drv2.setupLRA(&I2Ctwo, 5, 4);

  setupWiFi();

  ahrs.setup();
  capsense.setup(&I2Cone);

  hapticThread.onRun(hapticCallback);
  hapticThread.setInterval(hapticInterval);

  ledThread.onRun(ledCallback);
  ledThread.setInterval(ledInterval);

  analogSensorThread.onRun(analogSensorCallback);
  analogSensorThread.setInterval(analogSensorInterval);

  AHRSThread.onRun(AHRSCallback);
  AHRSThread.setInterval(AHRSinterval);

  capsenseThread.onRun(capsenseCallback);
  capsenseThread.setInterval(touchInterval);

  oscThread.onRun(oscCallback);
  oscThread.setInterval(oscInterval);

  oscSlowThread.onRun(oscSlowCallback);
  oscSlowThread.setInterval(oscSlowInterval);


//Add threads to main controller
  controll.add(&analogSensorThread);
  controll.add(&capsenseThread);
  controll.add(&AHRSThread);
  controll.add(&ledThread);
  controll.add(&oscThread);
  controll.add(&hapticThread);
  controll.add(&oscSlowThread);

// Init osc messages
  for (int k = 0; k < numParam; k++) {
    msg[k].setAddress(msgMeta[k].adress);
    for (int i = 0; i < msgMeta[k].length; i++)
    {
      msg[k].add(0.0f);
    }
  }

  // Wait until open serial connection for calibrationn with MotionCal
  if (CALIB )while (!Serial);
}


void loop() {
  controll.run();
}


//




