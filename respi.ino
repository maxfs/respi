/**
 *   ///////////////////////////////////////////////////////////////////////////
 * 
 *    Project: Respi - A Teensy 3.1 Arduino-powered respiration monitor.
 *    Author: Max-Ferdinand Suffel
 *    Creation Data: 11-02-2014
 *
 *   ///////////////////////////////////////////////////////////////////////////
 *
 *   Third party code used for the implementation:
 *   
 *   + SdFat library by William Greiman
 *   + I2Cdev device library code and MPU6050_DMP6 example by Jeff Rowberg
 *     ===============================================
 *     I2Cdev device library code is placed under the MIT license
 *     Copyright (c) 2012 Jeff Rowberg
 *
 *     Permission is hereby granted, free of charge, to any person obtaining a copy
 *     of this software and associated documentation files (the "Software"), to deal
 *     in the Software without restriction, including without limitation the rights
 *     to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *     copies of the Software, and to permit persons to whom the Software is
 *     furnished to do so, subject to the following conditions:
 *
 *     The above copyright notice and this permission notice shall be included in
 *     all copies or substantial portions of the Software.
 *
 *     THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *     IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *     FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *     UTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *     LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *     OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *     THE SOFTWARE.
 *     ===============================================
*/

// ================================================================
// ===                    IMPORTED LIBRARIES                    ===
// ================================================================

#include "SPI.h"    // SPI library is needed for the ILI9341_t3 display.
#include <SdFat.h>  // SdFat library is used to access the SD card.
#include <Adafruit_GFX.h>   // Adafruit GFX library is used for render routines.
#include "ILI9341_t3.h" // ILI9341_t3 library defines the display access func's.
#include <math.h>   // sqrt needed for magnitude calculation of acceleration.
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// ================================================================
// ===                 MPU5060 IMU ACCEL+GYRO                   ===
// ================================================================ 

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[128]; // FIFO storage buffer, allows to store approx 3 packets

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// moving average buffer
#define BUF_SIZE 3
struct MovingAverageBuffer {
  float buf[BUF_SIZE];
  size_t i;
  bool initialized;
} ax_buf,ay_buf,az_buf;

void addToMovingAverageBuffer (struct MovingAverageBuffer &m, float val) {
    if (!m.initialized) {
      for (size_t i = 0; i < BUF_SIZE; i++) {
        m.buf[i] = val;
      }
      m.initialized = true;
    } else {
      m.buf[m.i] = val;
      m.i = (++m.i)%BUF_SIZE;
    }
}

float getAverage (struct MovingAverageBuffer &m) {
   float sum = 0.0f;
   for (size_t i = 0; i < BUF_SIZE; i++) {
        sum += m.buf[i];
   }
   return sum/((float)BUF_SIZE);
}

// ================================================================
// ===                  MPU INTERRUPT ROUTINE                   ===
// ================================================================

// indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt = false;     
void dmpDataReady() {
     // top-half of the interrupt routine,
     // the rest is done in main loop for fast interrupt handling
    mpuInterrupt = true;
}

// ================================================================
// ===                  ILI9341_t3 TFT DISPLAY                  ===
// ================================================================

static const int TFT_DC = 9;
// The SPI chip-select of the ILI9341_t3 display is port 10.
static const int TFT_CS = 10;
// The ILI9341_t3 display.
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

// ================================================================
// ===                         SD CARD                          ===
// ================================================================

// The SPI chip-select of the SD card reader is port 6.
static const int SD_CS = 6;
// Fat32 filesystem on SD card
SdFat sd;
// Log file use for writing out the sensor data.
SdFile log_file;

long log_counter = 1;

// ================================================================
// ===                         AUXILIARY                        ===
// ================================================================

// Serial report output stream
ArduinoOutStream cout(Serial);

void setup (void) {
    // Initialize serial port with baud rate 38400bips
    Serial.begin(38400);
    // Wait until serial monitor is openend.
    //while (!Serial) {}
    // Small delay to ensure the serial port is initialized.
    delay(200);

    // Initialize the I2C ports.
    cout << pstr("Initializing SPI ports...");
    Wire.begin();
    cout << pstr("ok") << endl;

    // Initialize the TFT display.
    cout << pstr("Initializing TFT...");
    pinMode(TFT_CS, OUTPUT);
    tft.begin();
    // Setup a vertical orientation of the display.
    tft.setRotation(0);
    cout << pstr("ok") << endl;

    // Initialize sd card or print a detailed error message and halt
    // We run the SPI communication with the SD card by half speed, 
    // that is, 15Mhz since the Teensy 3.1 can only run the SPI port
    // with 30Mhz when the CPU is overclocked.
    // We do so since the communcation was unreliable with fullspeed.
    // Probably the rudimentary wiring over the breadboard is the problem.
    cout << pstr("Initializing SD card...");
    if (!sd.begin(SD_CS, SPI_HALF_SPEED))
      sd.initErrorHalt();

    cout << pstr("ok") << endl;

    // Create a new log file with write permission.
    cout << pstr("Create log file...");
    String str_log_file = "log1.csv";
    char buf[16];
    int cnt = 1;
    str_log_file.toCharArray(buf, 16);
    while (sd.exists(buf)) {
      cnt += 1;
      str_log_file = "log" + String(cnt) + ".csv";
      str_log_file.toCharArray(buf,16);
    }
    if (!log_file.open(str_log_file.c_str(), O_WRITE | O_CREAT))
      sd.errorHalt_P(PSTR("Cannot create log file"));

    cout << pstr("ok...") << str_log_file.c_str() << pstr(" created") << endl;

    cout << pstr("Initializing MPU6050 IMU...");
    mpu.initialize(); // the sensor's internal sampling rate will be set to 100Hz.
    cout << pstr("ok") << endl;

    // verify connection
    cout << pstr("Testing device connections...");
    cout << (mpu.testConnection() ? pstr("ok") : pstr("failed")) << endl;

    // load and configure the DMP
    cout << pstr("Initializing Digital Motion Processor (DMP)...");
    devStatus = mpu.dmpInitialize();

    // set gyro and accelerometer offsets, manually fiddeled out
    mpu.setXGyroOffset(36);
    mpu.setYGyroOffset(16);
    mpu.setZGyroOffset(-11);
    mpu.setXAccelOffset(-3047);
    mpu.setYAccelOffset(-61);
    mpu.setZAccelOffset(3381);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        cout << pstr("ok") << endl;

        // turn on the DMP, now that it's ready
        cout << pstr("Enabling Digital Motion Processor (DMP)...");
        mpu.setDMPEnabled(true);
        cout << pstr("ok") << endl;

        // enable Arduino interrupt detection
        cout << pstr("Enabling interrupt detection (Arduino external interrupt 0)...");
        pinMode(0, INPUT);
        attachInterrupt(0, dmpDataReady, RISING);
        cout << pstr("ok") << endl;

        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        cout << pstr("DMP ready! Waiting for first interrupt...");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        cout << pstr("ok") << endl;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        cout << pstr("failed (code ") << devStatus << pstr(")") << endl;
    }
}

void loop (void) {
    // if programming failed, don't try to do anything
    if (!dmpReady) return; // FIXME: print error message and halt

    // wait for MPU interrupt or extra packet(s) available
    // 
    // since the MPU internally samples with a rate of 100Hz,
    // there are 10ms left between each package arrives
    // 
    // note, the variable mpuInterrupt is shared with the
    // interrupt routine dmpDataReady. However, we don't have to
    // dissable the interrupts temporarily since the variable
    // mpuInterrupt is a bool which is written and read atomic
    // 
    // while there is no MPU package arrived, process the program logic
    while (!mpuInterrupt && fifoCount < packetSize) {
      // ...
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        cout << pstr("FIFO overflow!") << endl;

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        // compute the magnitude (i.e. the length by applying the Pythagoras) of
        // the  acceleration vector. doing so, does not force us to select a specific
        // acceleration axis later on for the breath detection.
        float magnitude = sqrt(sq(aaReal.x) + sq(aaReal.y) + sq(aaReal.z));

        Serial.println(magnitude);

        // finally let us log the data 
        log_file.printf("%d, %.4f\n", log_counter, magnitude);
        log_counter++;

        // flushes the data to the file
        // FIXME: flush only after e.g. 100 recordings; saves SD card lifetime.
        log_file.sync(); 
    }
}
