#define BLYNK_TEMPLATE_ID "TMPL5VK-L9qgu"
#define BLYNK_TEMPLATE_NAME "Airtime"
#define BLYNK_AUTH_TOKEN "sw6mVV5tRfOHCKw89Ut96-D7JJCdI1EH"
#define VPIN_LOWER_AVERAGE_JUMP V0
#define VPIN_ABOVE_AVERAGE_JUMP V1
#define VPIN_AVERAGE_JUMP V2
#define VPIN_PEAK_ACCEL V4
#define VPIN_TOTAL_JUMP V5
//for blynk app
#define VPIN_WRITE_BUTTON V6

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

const int MPU_ADDR = 0x68;  // I2C address of the MPU-6050
int16_t accelZ;
float gZ;

// Blynk Wi-Fi setup
char ssid[] = "Flavour country";
char pass[] = "Kuroko64";

// Calibration variables
float offsetZ = 0;
bool calibrating = false;
float jumpThreshold = 0;
float averagePeakAccelZ;

// Jump session variables
volatile bool startJumpSession;
volatile bool sessionEnded = false;
float maxAccelZ = 0;
int lowerThanAverageCount = 0;
int averageJumpCount = 0;
int goodJumpCount = 0;
int totalJump = 0;

// BLE setup
BLEServer* pServer = nullptr;
BLECharacteristic* pCalibrateCharacteristic = nullptr;
BLECharacteristic* pNotificationCharacteristic = nullptr;

const char* SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab";
const char* CALIBRATE_CHARACTERISTIC_UUID = "abcd1234-5678-1234-5678-abcdef123456";
const char* NOTIFICATION_CHARACTERISTIC_UUID = "1234abcd-5678-1234-5678-abcdef123456";

// Function prototypes
void enableBLEMode();
void performCalibration();
float readCalibratedAccelerometer();
void jumpSession();
void endSession();
void notifyUser(String message);

// BLE Callbacks
class CalibrationCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String value = pCharacteristic->getValue().c_str();
    if (value == "startCalibration") {
      calibrating = true;
      Serial.println("Calibration started...");
      notifyUser("Calibration started...");
    } else if (value == "startSession") {
      sessionEnded = false;
      startJumpSession = true;
      Serial.println("Jump session started...");
      notifyUser("Jump session started...");
    } else if (value == "endSession") {
      startJumpSession = false;
      sessionEnded = false;
      endSession();
    }
  }
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println("Client connected!");
    notifyUser("Client connected!");
  }
  void onDisconnect(BLEServer* pServer) override {
    Serial.println("Client disconnected!");
    notifyUser("Client disconnected!");
    BLEDevice::startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  Wire.begin(0, 1);

  // Initialize Wi-Fi with timeout
  Serial.print("Attempting Wi-Fi connection...");
  notifyUser("Attempting Wi-Fi connection...");
  unsigned long startTime = millis();
  const unsigned long timeout = 10000;  // 10-second timeout

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < timeout) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Wi-Fi");
    notifyUser("Connected to Wi-Fi");
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
    Blynk.run();
  } else {
    Serial.println("\nWi-Fi connection failed. Switching to BLE Low Power Mode...");
    enableBLEMode();
  }

  // MPU-6050 Setup
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void enableBLEMode() {
  Serial.println("Initializing BLE Low Power Mode...");
  notifyUser("Initializing BLE Low Power Mode...");

  BLEDevice::init("Airtime Tracker");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pCalibrateCharacteristic = pService->createCharacteristic(
    CALIBRATE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE);
  pCalibrateCharacteristic->setCallbacks(new CalibrationCallbacks());

  pNotificationCharacteristic = pService->createCharacteristic(
    NOTIFICATION_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);

  pService->start();
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();

  Serial.println("BLE Low Power Mode Active. Awaiting commands...");
}

 BLYNK_WRITE(VPIN_WRITE_BUTTON){
  String input = param.asStr();
  if(input.equalsIgnoreCase("cali")){
    calibrating = true;
    Serial.println("starting Calibration...");
    return;
  } else if (input.equalsIgnoreCase("start")){
    startJumpSession = true;
    Serial.println("starting Session...");
    return;
  } else if (input.equalsIgnoreCase("end")){
    sessionEnded = true;
    Serial.println("ending Session...");
    return;
  } else {
    Serial.println("error misinput");
  }
 }

void loop() {
  static bool waitingMessageShown = false;

  if (calibrating) {
    performCalibration();
    waitingMessageShown = false;
  } else if (startJumpSession) {
    jumpSession();
    waitingMessageShown = false;
  } else if (sessionEnded) {
    endSession();
  } else if (!waitingMessageShown) {
    Serial.println("WAITING FOR COMMAND");
    notifyUser("WAITING FOR COMMAND");
    waitingMessageShown = true;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Blynk.run();  // Run Blynk only if Wi-Fi is connected
  }

  delay(50);
}

void readAccelerometer() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  accelZ = Wire.read() << 8 | Wire.read();  // Read raw data
  gZ = (accelZ / 4096.0) * 9.8;             // Scale for ±8g range aand putting it in m/s^2
}

float readCalibratedAccelerometer() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  accelZ = Wire.read() << 8 | Wire.read();     // Read raw data
  return ((accelZ / 4096.0) - offsetZ) * 9.8;  // Scale for ±8g range aand putting it in m/s
}

void performCalibration() {
  const int calibrationDuration = 10000;
  unsigned long startTime = millis();
  long sumZ = 0;
  int count = 0;

  Serial.println("Starting calibration, please keep the device still...");
  notifyUser("Starting calibration, please keep the device still...");

  while (millis() - startTime < calibrationDuration) {
    readAccelerometer();
    sumZ += gZ;
    count++;
    delay(100);
  }

  offsetZ = sumZ / count;
  calibrating = false;
  Serial.println("Calibration complete!");
  notifyUser("Calibration complete!");

  String message = "Calibration complete: Offset Z = " + String(offsetZ);
  pNotificationCharacteristic->setValue(message.c_str());
  pNotificationCharacteristic->notify();

  setJumpThreshold();
  setBaseJump();

  message = "Final Calibration complete:";
  calibrating = false;
  pNotificationCharacteristic->setValue(message.c_str());
  pNotificationCharacteristic->notify();
}

void setJumpThreshold() {
  Serial.println("Please perform a single hop...");
  notifyUser("Please perform a single hop...");

  float peakZ = 0;
  bool hopDetected = false;
  unsigned long startHopTime = millis();
  while (millis() - startHopTime < 5000) {
    readAccelerometer();
    float netZ = gZ - offsetZ;
    if (netZ > peakZ) {
      peakZ = netZ;
      hopDetected = true;
    }
    delay(50);
  }

  if (hopDetected) {
    jumpThreshold = peakZ * 0.8;
    Serial.print("Hop detected! Setting jump threshold to: ");
    Serial.println(jumpThreshold);
    notifyUser("Hop detected! Setting jump threshold to: " + String(jumpThreshold));

    String message = "Hop detected. Jump threshold set to: " + String(jumpThreshold);
    pNotificationCharacteristic->setValue(message.c_str());
    pNotificationCharacteristic->notify();
  } else {
    Serial.println("Hop not detected. Please try calibrating again.");
    notifyUser("Hop not detected. Please try calibrating again.");
  }
}

void setBaseJump() {
  Serial.println("Please perform three standing jumps...");
  notifyUser("Please perform three standing jumps...");

  int count = 3;
  float totalPeakAccelZ = 0;

  while (count != 0) {
    float peakAccelZ = 0;
    bool hopDetected = false;

    readAccelerometer();
    float netZ = gZ - offsetZ;

    if (netZ > jumpThreshold) {
      hopDetected = true;
      peakAccelZ = netZ;

      while (hopDetected) {
        readAccelerometer();
        netZ = gZ - offsetZ;

        if (netZ > peakAccelZ) {
          peakAccelZ = netZ;
        }

        if (netZ <= jumpThreshold) {
          hopDetected = false;
          totalPeakAccelZ += peakAccelZ;
          count--;

          Serial.print("Jump detected with peak acceleration: ");
          Serial.println(peakAccelZ);
          notifyUser("Jump detected with peak acceleration: " + String(peakAccelZ));
        }
      }
      delay(150);
    }
  }

  averagePeakAccelZ = totalPeakAccelZ / 3;
  Serial.print("Average peak acceleration from 3 jumps: ");
  Serial.println(averagePeakAccelZ);
  notifyUser("Average peak acceleration from 3 jumps: " + String(averagePeakAccelZ));
}

void jumpSession() {
  static bool jumpDetected = false;  // Tracks whether a jump is currently detected
  static float peakAccelZ = 0;       // Stores the peak acceleration of the jump
  sessionEnded = false;              // Tracks whether the session has ended
  float netZ;

  while (!sessionEnded) {
    // Detect a jump
    while (!jumpDetected) {
      netZ = readCalibratedAccelerometer();  // Read calibrated acceleration

      if (netZ > jumpThreshold) {  // Check for jump threshold
        jumpDetected = true;
        peakAccelZ = netZ;  // Initialize peak acceleration
        notifyUser("Jump detected!");
        Serial.println("Jump detected!");
      }
    }

    // Track peak acceleration during the jump
    while (jumpDetected) {
      netZ = readCalibratedAccelerometer();  // Update calibrated acceleration

      if (netZ > peakAccelZ) {  // Update peak acceleration if higher
        peakAccelZ = netZ;
      }

      // End jump detection when acceleration drops below threshold
      if (netZ <= jumpThreshold) {
        jumpDetected = false;
        classifyAndRecordJump(peakAccelZ);  // Classify and record the jump
        peakAccelZ = 0;                     // Reset for the next jump
      }
    }
  }

  endSession();  // Summarize and clean up after the session ends
}

void classifyAndRecordJump(float peakAccelZ) {
  // Define ranges for jump quality classification
  float lowerBound = averagePeakAccelZ * 0.95;
  float upperBound = averagePeakAccelZ * 1.05;
  float goodUpperBound = averagePeakAccelZ * 1.125;

  String jumpQuality;

  Serial.print("peakAccelZ: ");
  Serial.println(peakAccelZ);
  notifyUser("peakAccelZ: " + String(peakAccelZ));
  Serial.print("avg peakAccelZ: ");
  Serial.println(averagePeakAccelZ);
  notifyUser("avg peakAccelZ: " + String(averagePeakAccelZ));

  // Classify the jump
  if (peakAccelZ < lowerBound) {
    jumpQuality = "Lower than average";
    lowerThanAverageCount++;
    Blynk.virtualWrite(VPIN_LOWER_AVERAGE_JUMP, lowerThanAverageCount);
    totalJump++;
  } else if (peakAccelZ >= lowerBound && peakAccelZ <= upperBound) {
    jumpQuality = "Average";
    averageJumpCount++;
    Blynk.virtualWrite(VPIN_AVERAGE_JUMP, averageJumpCount);
    totalJump++;
  } else if (peakAccelZ > upperBound && peakAccelZ <= goodUpperBound) {
    jumpQuality = "Good";
    goodJumpCount++;
    Blynk.virtualWrite(VPIN_ABOVE_AVERAGE_JUMP, goodJumpCount);
    totalJump++;
  } else {
    Serial.println("error");
  }

  if (peakAccelZ >= maxAccelZ) {
    maxAccelZ = peakAccelZ;
    Blynk.virtualWrite(VPIN_PEAK_ACCEL, maxAccelZ);
  }

  // Send jump quality notification via BLE
  pNotificationCharacteristic->setValue(jumpQuality.c_str());
  pNotificationCharacteristic->notify();

  Serial.print("Jump classified as: ");
  Serial.println(jumpQuality);
  notifyUser("Jump classified as: " + jumpQuality);

  Serial.print("Total jumps: ");
  Serial.println(totalJump);
  Blynk.virtualWrite(VPIN_TOTAL_JUMP, totalJump);
}

void endSession() {
  Serial.println("Session ended!");
  notifyUser("Session ended!");

  // Send final jump stats to the user
  String summary = "Session Summary: \n";
  summary += "Lower than average: " + String(lowerThanAverageCount) + "\n";
  summary += "Average: " + String(averageJumpCount) + "\n";
  summary += "Good: " + String(goodJumpCount) + "\n";
  summary += "Max Accel: " + String(maxAccelZ) + "\n";
  summary += "Total Jump Count: " + String(totalJump) + "\n";

  notifyUser(summary);

  // Reset jump counts
  lowerThanAverageCount = 0;
  averageJumpCount = 0;
  goodJumpCount = 0;
}

void notifyUser(String message) {
  if (pNotificationCharacteristic != nullptr) {            
    pNotificationCharacteristic->setValue(message.c_str());  
    pNotificationCharacteristic->notify();                   
    Serial.println("Notification sent: " + message);         
  }
}