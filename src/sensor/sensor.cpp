#include <Arduino.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <arduinoFFT.h>
#include <Wire.h>

#include "MPU6050_6Axis_MotionApps20.h"
#include "mode.h"
#include "config.h"

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
arduinoFFT FFT = arduinoFFT();

/*
These values can be changed in order to evaluate the functions
*/
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 100;
/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion quat;     // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 gg;      // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector

VectorInt16 aaBuffer[100];

uint16_t packetCount = 0;
char packetBuffer[1024];

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

void performFFT()
{
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);                 /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples);                   /* Compute magnitudes */
  uint16_t bufferSize = samples >> 2;

  for (uint16_t i = 5; i < bufferSize; i++)
  {
    double abscissa = (i * samplingFrequency) / samples;
    Serial.print(abscissa, 6);
    Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vReal[i], 4);
  }

  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  // frequency = x;
  Serial.print("MajorPeak (Hz):");
  Serial.println(x, 6);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loopSensor(void *pvParameters)
{
  // This task is not allowed to return. This is a small helper, maybe the loop can be
  // designed better.
  while (true)
  {
    // if programming failed, don't try to do anything
    if (!dmpReady)
    {
      continue;
    }

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
      if (mpuInterrupt && fifoCount < packetSize)
      {
        // try to get out of the infinite loop
        fifoCount = mpu.getFIFOCount();
      }
      // other program behavior stuff here

      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
      if (packetCount == samples)
      {
        performFFT();
        packetCount = 0;
      }
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    if (fifoCount < packetSize)
    {
      //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
      continue;
    }

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
    {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

      continue;
    }

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
    {
      // Safty check in case we missed the data calculation and need to reset counter to
      // avoid buffer overflow.
      if (packetCount >= samples)
      {
        packetCount = 0;
      }

      // read a packet from FIFO
      while (fifoCount >= packetSize)
      { // Lets catch up to NOW, someone is using the dreaded delay()!
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
      }

      // display real acceleration, adjusted to remove gravity
      mpu.dmpGetQuaternion(&quat, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &quat);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

      double magnitude = sqrt(aaReal.x * aaReal.x + aaReal.y * aaReal.y + aaReal.z * aaReal.z);
      vReal[packetCount] = magnitude;
      vImag[packetCount] = 0;

      packetCount++;
    }
  }
}

void setupSensor()
{
  Serial.println(F("Setup: Sensor"));

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  delay(1000);

  mpu.initialize();
  pinMode(SENSOR_INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-1707);
  mpu.setYAccelOffset(-545);
  mpu.setZAccelOffset(1262);
  mpu.setXGyroOffset(54);
  mpu.setYGyroOffset(47);
  mpu.setZGyroOffset(-3);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);

    mpu.setDLPFMode(6);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(SENSOR_INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    delay(30000);
    ESP.restart();
  }

  xTaskCreatePinnedToCore(
      loopSensor,      // Function to implement the task
      "sensor",        // Name of the task
      10000,           // Stack size in words
      NULL,            // Task input parameter
      0,               // Priority of the task
      NULL,            // Task handle.
      SENSOR_CORE_ID); // Core where the task should run
}
