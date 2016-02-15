
/* Written by Oren Levy (auxren.com) based off some lackluster
 * documentation released by Intel.
 * MIDI over BLE info from: https://developer.apple.com/bluetooth/Apple-Bluetooth-Low-Energy-MIDI-Specification.pdf
 */
#include <BleCharacteristic.h>
#include <BleDevice.h>
#include <BleService.h>
#include <BleCommon.h>
#include <BleDescriptor.h>


#define TXRX_BUF_LEN              20 //max number of bytes
#define RX_BUF_LEN                20 //max number of bytes
uint8_t rx_buf[RX_BUF_LEN];
int rx_buf_num, rx_state = 0;
uint8_t rx_temp_buf[20];
uint8_t outBufMidi[128];

/* Bluetooth MAC address for this device */
BleDeviceAddress localAddress;
/* Bluetooth MAC address for remote peer device */
BleDeviceAddress peerAddress;

//Character ID for MIDI compliance
uint8_t CHAR_UUID_MIDI[] = { 0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9,

                             0xA1, 0x12, 0x41, 0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77
                           };
//Service ID for MIDI compliance
uint8_t midiIOServiceUUID[] = { 0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51,

                                0xA7, 0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03
                              };

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
 
}

void setup() {
  LOG_SERIAL.begin(9600);

  CHECK_STATUS(customDevice.setName("Auxren's MIDI"));
  CHECK_STATUS(customDevice.init());
  CHECK_STATUS(customDevice.getLocalAddress(localAddress));
  customDevice.setEventCallback(blePeripheralEventCb);
  CHECK_STATUS(customDevice.addPrimaryService(midiSvc, true));
  CHECK_STATUS(midiSvc.addCharacteristic(midiChar));
  CHECK_STATUS(customDevice.start());

  LOG_SERIAL.println("Bluetooth device active, waiting for connections...");
}



void loop() {

  int note = random(0, 127);

  noteOn(0, note, 127); //loads up midiData buffer
  CHECK_STATUS(midiChar.setValue(midiData, 5)); //posts 5 bytes
  delay(500);
  noteOff(0, note);
  delay(500);
}

