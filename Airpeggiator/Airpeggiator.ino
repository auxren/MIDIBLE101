/* Written by Oren Levy (auxren.com) based off some lackluster
   documentation released by Intel and Jeff Rowberg's IMU code.
   MIDI over BLE info from: https://developer.apple.com/bluetooth/Apple-Bluetooth-Low-Energy-MIDI-Specification.pdf


  The MIT License (MIT)

  Copyright (c) 2016 auxren

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
  ===============================================
*/

//Set your tempo, key, and scale
#define BPM        120
#define MainKey    C
#define MainScale  HarmonicMinor

#include <CurieBLE.h>
#include "CurieIMU.h"

//Setting up buffers for MIDI stuff
#define TXRX_BUF_LEN              20 //max number of bytes
#define RX_BUF_LEN                20 //max number of bytes
uint8_t rx_buf[RX_BUF_LEN];
int rx_buf_num, rx_state = 0;
uint8_t rx_temp_buf[20];
uint8_t outBufMidi[128];

//Defining scales
#define Major 0
#define Minor 1
#define Mixolydian 2
#define MajorPentatonic 3
#define MinorPentatonic 4
#define HarmonicMinor 5
//Defining notes
#define C 0
#define Db 1
#define D 2
#define Eb 3
#define E 4
#define F 5
#define Gb 6
#define G 7
#define Ab 8
#define A 9
#define Bb 10
#define B 11
//Defining the actual scales
uint8_t MajorScale[] = {0, 2, 4, 5, 7, 9, 11};
uint8_t MinorScale[] = {0, 2, 3, 5, 7, 8, 10};
uint8_t MixolydianScale[] = {0, 2, 4, 4, 7, 9, 10};
uint8_t MajorPentatonicScale[] = {0, 2, 4, 7, 9};
uint8_t MinorPentatonicScale[] = {0, 3, 5, 7, 10};
uint8_t HarmonicMinorScale[] = {0, 2, 3, 5, 7, 8, 10};
const float alpha = 0.5;

//Variables for IMU stuff
int rollNote = 0;
int pitchNote = 0;
int newNote = 0;
int accelFlag = 0;
int calibrateOffsets = 1; // int to determine whether calibration takes place or not
double fXg = 0;
double fYg = 0;
double fZg = 0;
double roll, pitch, yaw; // Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer
int ax, ay, az;         // accelerometer values
int gx, gy, gz;         // gyrometer values

//Buffer to hold 5 bytes of MIDI data. Note the timestamp is forced
uint8_t midiData[] = {0x80, 0x80, 0x00, 0x00, 0x00};

//Loads up buffer with values for note On
void noteOn(char chan, char note, char vel) //channel 1
{
  midiData[2] = 0x90 + chan;
  midiData[3] = note;
  midiData[4] = vel;
}

//Loads up buffer with values for note Off
void noteOff(char chan, char note) //channel 1
{
  midiData[2] = 0x80 + chan;
  midiData[3] = note;
  midiData[4] = 0;
}

BLEPeripheral midiDevice; // create peripheral instance

BLEService midiSvc("03B80E5A-EDE8-4B33-A751-6CE34EC4C700"); // create service

// create switch characteristic and allow remote device to read and write
BLECharacteristic midiChar("7772E5DB-3868-4112-A1A9-F2669D106BF3", BLEWrite | BLEWriteWithoutResponse | BLENotify | BLERead, 5);

void setup() {
  Serial.begin(9600);
  IMUSetup();
  BLESetup();
  Serial.println(("Bluetooth device active, waiting for connections..."));
}

static void eventCallback(void)
{
  accelFlag = 1;
}

void IMUSetup()
{
  // initialize device
  Serial.println("Initializing IMU device...");
  CurieIMU.begin();

  // verify connection
  Serial.println("Testing device connections...");
  if (CurieIMU.begin()) {
    Serial.println("CurieIMU connection successful");
  } else {
    Serial.println("CurieIMU connection failed");
  }

  // use the code below to calibrate accel/gyro offset values
  if (calibrateOffsets == 1) {
    Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -235
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 168
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));

    Serial.println("About to calibrate. Make sure your board is stable and upright");
    delay(5000);

    // The board must be resting in a horizontal position for
    // the following calibration procedure to work correctly!
    Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateGyroOffset();
    Serial.println(" Done");

    Serial.print("Starting Acceleration calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -2359
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 1688
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
  }
}


void BLESetup()
{
  // set the local name peripheral advertises
  midiDevice.setLocalName("Auxren");
  midiDevice.setDeviceName("Auxren");

  // set the UUID for the service this peripheral advertises
  midiDevice.setAdvertisedServiceUuid(midiSvc.uuid());

  // add service and characteristic
  midiDevice.addAttribute(midiSvc);
  midiDevice.addAttribute(midiChar);

  // assign event handlers for connected, disconnected to peripheral
  midiDevice.setEventHandler(BLEConnected, midiDeviceConnectHandler);
  midiDevice.setEventHandler(BLEDisconnected, midiDeviceDisconnectHandler);

  // assign event handlers for characteristic
  midiChar.setEventHandler(BLEWritten, midiCharacteristicWritten);
  // set an initial value for the characteristic
  midiChar.setValue(midiData, 5);

  // advertise the service
  midiDevice.begin();
}


/* This quantizer takes in the unquantized not number,
    the scale, and the key. It then figures out what note
    to 'snap' to based off the scale definitions. This is probably
    not the most efficient way to do this, but I needed something
    quick and it works.
*/
uint8_t quantize(int note, char scale, char key)
{
  int modNote = 0;
  uint8_t tempScale[8];
  modNote = note % 12;
  if (scale == Major)
  {
    for (int i = 0; i < sizeof(MajorScale); i++)
    {
      if (modNote == MajorScale[i] + key)
      {
        return note;
      }
      else if (modNote + 1 == MajorScale[i] + key)
      {
        return note + 1;
      }
    }
  }
  else if (scale == Minor)
  {
    for (int i = 0; i < sizeof(MinorScale); i++)
    {
      if (modNote == MinorScale[i] + key)
      {
        return note;
      }
      else if (modNote + 1 == MinorScale[i] + key)
      {
        return note + 1;
      }
    }
  }
  else if (scale == MajorPentatonic)
  {
    for (int i = 0; i < sizeof(MajorPentatonicScale); i++)
    {
      if (modNote == MajorPentatonicScale[i] + key)
      {
        return note;
      }
      else if (modNote + 1 == MajorPentatonicScale[i] + key)
      {
        return note + 1;
      }
    }
  }
  else if (scale == MinorPentatonic)
  {
    for (int i = 0; i < sizeof(MinorPentatonicScale); i++)
    {
      if (modNote == MinorPentatonicScale[i] + key)
      {
        return note;
      }
      else if (modNote + 1 == MinorPentatonicScale[i] + key)
      {
        return note + 1;
      }
    }
  }
  else if (scale == Mixolydian)
  {
    for (int i = 0; i < sizeof(MixolydianScale); i++)
    {
      if (modNote == MixolydianScale[i] + key)
      {
        return note;
      }
      else if (modNote + 1 == MixolydianScale[i] + key)
      {
        return note + 1;
      }
    }
  }
  else if (scale == HarmonicMinor)
  {
    for (int i = 0; i < sizeof(HarmonicMinorScale); i++)
    {
      if (modNote == HarmonicMinorScale[i] + key)
      {
        return note;
      }
      else if (modNote + 1 == HarmonicMinorScale[i] + key)
      {
        return note + 1;
      }
    }
  }
}

void loop() {
  // read raw accel/gyro measurements from device
  CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);
  roll = atan2(ay, az) * RAD_TO_DEG; //calculate the roll
  pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG; //calculate the pitch

  rollNote = roll / 4 + 80; //convert the raw roll to a MIDI note value
  pitchNote = pitch / 4 + 80; //convert the raw pitch to a MIDI note value

  if (pitch > 10) //filter out notes that are too low
  {
    newNote = quantize(rollNote, MainScale, MainKey); //Quantize the note
    Serial.println(newNote); //print it out so we can see what it plays on the term
    noteOn(0, newNote, 127); //turn the note on
    midiChar.setValue(midiData, 5); //send that shit out over BLE

    delay(8);
    noteOff(0, newNote); //turn the note off
    midiChar.setValue(midiData, 5); //send that shit out over BLE
  }
  delay(BPM);
}

void midiDeviceConnectHandler(BLECentral & central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void midiDeviceDisconnectHandler(BLECentral & central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void midiCharacteristicWritten(BLECentral & central, BLECharacteristic & characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: ");
}
