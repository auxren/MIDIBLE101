
#include <Wire.h>
/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include <BleCharacteristic.h>
#include <BleDevice.h>
#include <BleService.h>
#include <BleCommon.h>
#include <BleDescriptor.h>
#include "CurieImu.h"
#include <Time.h>

#define Major 0
#define Minor 1
#define Mixolydian 2
#define MajorPentatonic 3
#define MinorPentatonic 4
#define HarmonicMinor 5
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
uint8_t MajorScale[] = {0, 2, 4, 5, 7, 9, 11};
uint8_t MinorScale[] = {0, 2, 3, 5, 7, 8, 10};
uint8_t MixolydianScale[] = {0, 2, 4, 4, 7, 9, 10};
uint8_t MajorPentatonicScale[] = {0, 2, 4, 7, 9};
uint8_t MinorPentatonicScale[] = {0, 3, 5, 7, 10};
uint8_t HarmonicMinorScale[] = {0, 2, 3, 5, 7, 8, 10};

const float alpha = 0.5;


int noteTempx = 0;
int noteTempy = 0;
int noteTempz = 0;
int rollNote = 0;
int pitchNote = 0;
int newNote1 = 0;
int newNote2 = 0;

double fXg = 0;
double fYg = 0;
double fZg = 0;

double roll, pitch, yaw; // Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer

#define BPM        120
#define MainKey    C
#define MainScale  MinorPentatonic

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define TXRX_BUF_LEN                    20
#define RX_BUF_LEN                      20 
uint8_t rx_buf[RX_BUF_LEN];
int rx_buf_num, rx_state = 0;
uint8_t rx_temp_buf[20];
uint8_t outBufMidi[128];

/* Bluetooth MAC address for this device */
BleDeviceAddress localAddress;
/* Bluetooth MAC address for remote peer device */
BleDeviceAddress peerAddress;

uint16_t milliseconds_elapsed = 0;
uint8_t CHAR_UUID_MIDI[] = { 0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9,

                             0xA1, 0x12, 0x41, 0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77
                           };

uint8_t midiIOServiceUUID[] = { 0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51,

                                0xA7, 0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03
                              };

uint8_t midiData[] = {0x80, 0x80, 0x00, 0x00, 0x00};

void noteOn(char chan, char note, char vel) //channel 1
{
  midiData[2] = 0x90 + chan;
  midiData[3] = note;
  midiData[4] = vel;
}

void noteOff(char chan, char note) //channel 1
{
  midiData[2] = 0x80 + chan;
  midiData[3] = note;
  midiData[4] = 0;
}

int BPMToMsDelay()
{
  int msOut = 600 / BPM;
  return msOut;
}



BleService midiSvc(midiIOServiceUUID);

BleCharacteristic midiChar(CHAR_UUID_MIDI, 5, BLE_CLIENT_ACCESS_READ_WRITE, BLE_CLIENT_NOTIFY_ENABLED);
BlePeripheral customDevice;

/* Serial port to use for printing informational messages to the user */
#define LOG_SERIAL Serial

/* For convenience, this macro will invoke a specified function call and will
 * check the status value returned to ensure it is successful.  If not, it will
 * print an error message to the serial port and will return from the current function
 */
#define CHECK_STATUS(op)                               \
  do {                                                 \
    BleStatus status = op;                             \
    if (BLE_STATUS_SUCCESS != status) {                \
      LOG_SERIAL.print(#op" returned error status: "); \
      LOG_SERIAL.println(status);                      \
      return;                                          \
    }                                                  \
  } while(0)

void printBleDeviceAddress(BleDeviceAddress &address, const char *label)
{
  LOG_SERIAL.print(label);
  LOG_SERIAL.print(" device address: ");

  /* The address data is stored in little-endian format in memory so the
   * bytes are printed in reverse-order to display a readable address */
  for (int i = BLE_DEVICE_ADDR_LEN - 1; i >= 0 ; i--)
    LOG_SERIAL.print(address.addr[i], HEX);

  LOG_SERIAL.println();
}
/* This function will be called when a BLE GAP event is detected by the
 * Intel Curie BLE device */
void blePeripheralEventCb(BlePeripheral &bleDevice, BlePeripheralEvent event, void *arg)
{
  LOG_SERIAL.println("Here1");
}

int accelFlag = 0;

static void eventCallback(void)
{
  //Serial.println("herehrehrer!");
  accelFlag = 1;
}



void setup() {
  // LOG_SERIAL.begin(115200);
  Serial.begin(9600);
  CurieImu.initialize();
  CurieImu.attachInterrupt(eventCallback);
  // Increase Accelerometer range to allow detection of stronger taps (< 4g)
  //CurieImu.setFullScaleAccelRange(BMI160_ACCEL_RANGE_4G);
  CurieImu.setIntTapEnabled(true);
  CurieImu.setIntEnabled(true);

  // Reduce threshold to allow detection of weaker taps (>= 750mg)
  CurieImu.setTapDetectionThreshold(7); // (6 x 125mg)

  CurieImu.autoCalibrateGyroOffset();

  CurieImu.autoCalibrateXAccelOffset(0);
  CurieImu.autoCalibrateYAccelOffset(0);
  CurieImu.autoCalibrateZAccelOffset(1);

  CurieImu.setGyroOffsetEnabled(true);
  CurieImu.setAccelOffsetEnabled(true);
  CurieImu.setAccelDLPFMode(2);

  CHECK_STATUS(customDevice.setName("TABOR"));
  CHECK_STATUS(customDevice.init());
  CHECK_STATUS(customDevice.getLocalAddress(localAddress));
  //printBleDeviceAddress(localAddress, "local");
  customDevice.setEventCallback(blePeripheralEventCb);
  CHECK_STATUS(customDevice.addPrimaryService(midiSvc, true));
  CHECK_STATUS(midiSvc.addCharacteristic(midiChar));
  // CHECK_STATUS(midiChar.setValue(bleMIDIPacket1, 5));
  CHECK_STATUS(customDevice.start());

  LOG_SERIAL.println("Bluetooth device active, waiting for connections...");
}
int i = 0;
int errorCode = 0;



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


int oldNote = 0;

void loop() {
  CurieImu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  roll = atan2(ay, az) * RAD_TO_DEG;
  pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  double Bfy = gz * sin(roll) - gy * cos(roll);
  double Bfx = gx * cos(pitch) + gy * sin(pitch) * sin(roll) + gz * sin(pitch) * cos(roll);
  yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;

//Serial.println(pitch);
  rollNote = roll/4 + 80;
  pitchNote = pitch/4 + 80;


  if(pitch > 10)
  {

  newNote1 = quantize(rollNote, MainScale, MainKey);
   Serial.println(newNote1);
   noteOn(0, newNote1, 127);
  CHECK_STATUS(midiChar.setValue(midiData, 5));


  delay(8);
  noteOff(0, newNote1);
  CHECK_STATUS(midiChar.setValue(midiData, 5));
  }
  delay(BPM);
  

 /* if((newNote1 != oldNote) && (newNote1 >=50))
  {
    Serial.println(newNote1);
   noteOn(0, newNote1, 127);
  CHECK_STATUS(midiChar.setValue(midiData, 5));


  delay(1);
  noteOff(0, newNote1);
  CHECK_STATUS(midiChar.setValue(midiData, 5));

 // accelFlag = 0;
oldNote = newNote1;
  }*/

/*if(accelFlag == 1)
{

 CurieImu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  roll = atan2(ay, az) * RAD_TO_DEG;
  pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  double Bfy = gz * sin(roll) - gy * cos(roll);
  double Bfx = gx * cos(pitch) + gy * sin(pitch) * sin(roll) + gz * sin(pitch) * cos(roll);
  yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;


  rollNote = roll + 80;
  pitchNote = pitch + 80;

  newNote1 = quantize(rollNote, MainScale, MainKey);
   noteOn(0, newNote1, 127);
  CHECK_STATUS(midiChar.setValue(midiData, 5));


  delay(1);
  noteOff(0, newNote1);
  CHECK_STATUS(midiChar.setValue(midiData, 5));

  accelFlag = 0;
}*/


/*

  roll = atan2(ay, az) * RAD_TO_DEG;
  pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  double Bfy = gz * sin(roll) - gy * cos(roll);
  double Bfx = gx * cos(pitch) + gy * sin(pitch) * sin(roll) + gz * sin(pitch) * cos(roll);
  yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;


  Serial.print(roll); Serial.print("\t");
  Serial.print(pitch); Serial.print("\t");
  Serial.print(yaw); Serial.println("\t");



  // int accelMIDIx =  (ax + 32754) * (127 - 20) / 65508;
  // int gyroMIDIx =  (gx + 32754) * (127 - 20) / 65508;
  //int accelMIDIy =  (ay + 32754) * (127 - 20) / 65508;
  // int accelMIDIz =  (az + 32754) * (127 - 20) / 65508;

  rollNote = roll + 80;
  pitchNote = pitch + 80;

  newNote1 = quantize(rollNote, MainScale, MainKey);
  newNote2 = quantize(pitchNote, MainScale, MainKey);
  noteOn(0, newNote1, 127);
  CHECK_STATUS(midiChar.setValue(midiData, 5));
  noteOn(0, newNote2, 127);
  CHECK_STATUS(midiChar.setValue(midiData, 5));

  delay(BPMToMsDelay() / 2);
  noteOff(0, newNote1);
  CHECK_STATUS(midiChar.setValue(midiData, 5));
  noteOff(0, newNote2);
  CHECK_STATUS(midiChar.setValue(midiData, 5));
  delay(BPMToMsDelay() / 2);*/

  /*  if (accelMIDIx != noteTempx)
    {*/
  /* int newNote = quantize(gyroMIDIx, MainScale, MainKey);
   noteOn(0, newNote, 127);
   CHECK_STATUS(midiChar.setValue(midiData, 5));

   delay(BPMToMsDelay()/2);
   noteOff(0,newNote);
   CHECK_STATUS(midiChar.setValue(midiData, 5));
   delay(BPMToMsDelay()/2);
   */
  //noteTempx = accelMIDIx;
  //}
  /* if (accelMIDIy != noteTempy)
   {
     int newNote = quantize(accelMIDIy, HarmonicMinor, Bb);
     noteOn(1, newNote, 127);
     CHECK_STATUS(midiChar.setValue(midiData, 5));

     delay(100);
     noteOff(1, newNote);
     CHECK_STATUS(midiChar.setValue(midiData, 5));

     noteTempy = accelMIDIy;
   }
   if (accelMIDIz != noteTempz)
   {
     int newNote = quantize(accelMIDIz, HarmonicMinor, Bb);
     noteOn(2, newNote, 127);
     CHECK_STATUS(midiChar.setValue(midiData, 5));

     delay(100);
     noteOff(2, newNote);
     CHECK_STATUS(midiChar.setValue(midiData, 5));

     noteTempz = accelMIDIz;
   }
   if (az < -2000)
   {
     midiData[2] = 0xB0;
     midiData[3] = 123;
     midiData[4] = 127;
     CHECK_STATUS(midiChar.setValue(midiData, 5));
     midiData[2] = 0xB1;
     midiData[3] = 123;
     midiData[4] = 127;
     CHECK_STATUS(midiChar.setValue(midiData, 5));
     midiData[2] = 0xB2;
     midiData[3] = 123;
     midiData[4] = 127;
     CHECK_STATUS(midiChar.setValue(midiData, 5));
   }
  */
}

void playRollNote()
{
 


}
