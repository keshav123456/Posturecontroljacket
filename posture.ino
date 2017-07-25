#include <MadgwickAHRS.h>


#include <BMI160.h>
#include <CurieIMU.h>


#include <CurieBLE.h>


Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
int test = 0;
int heartRateMeasurement = 467;


/*
   Copyright (c) 2015 Intel Corporation.  All rights reserved.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
   This sketch example partially implements the standard Bluetooth Low-Energy Heart Rate service.
   For more information: https://developer.bluetooth.org/gatt/services/Pages/ServicesHome.aspx
*/


BLEPeripheral blePeripheral;       // BLE Peripheral Device (the board you're programming)
BLEService heartRateService("180D"); // BLE Heart Rate Service

// BLE Heart Rate Measurement Characteristic"
BLECharacteristic heartRateChar("2A37",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, 2);  // remote clients will be able to get notifications if this characteristic changes
                              // the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications
                              // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.heart_rate_measurement.xml

int oldHeartRate = 0;  // last heart rate reading from analog input
long previousMillis = 0;  // last time the heart rate was checked, in ms
long previousMilli = 0;

void setup() {
  Serial.begin(9600);    // initialize serial communication
  pinMode(13, OUTPUT);   // initialize the LED on pin 13 to indicate when a central is connected

  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
  pinMode(6,OUTPUT);
  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet */
  blePeripheral.setLocalName("Posture control");
  blePeripheral.setAdvertisedServiceUuid(heartRateService.uuid());  // add the service UUID
  blePeripheral.addAttribute(heartRateService);   // Add the BLE Heart Rate service
  blePeripheral.addAttribute(heartRateChar); // add the Heart Rate Measurement characteristic

  /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
  blePeripheral.begin();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  // listen for BLE peripherals to connect:

      readacc();

  
  BLECentral central = blePeripheral.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:

    

    
    // check the heart rate measurement every 200ms
    // as long as the central is still connected:
    while (central.connected()) {
      
      readacc();
      
      long currentMillis = millis();
      // if 200ms have passed, check the heart rate measurement:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        updateHeartRate();
      }
    }


      
    // when the central disconnects, turn off the LED:
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }


  /* start of the accele code */

  
}

void readacc() {
  
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    
   /* Serial.print("Orientation: ");
    
    Serial.print("       ");
    Serial.print(pitch);
    Serial.print("        ");


    */


if (pitch < -70){

                  digitalWrite(13,LOW);
                  digitalWrite(6,LOW);
                  heartRateMeasurement = 320;
                  test = 0;
          
  }

else if (pitch > -70) {

        long currentMilli = millis();

      while (currentMilli - previousMilli >= 10000) {
        previousMilli = currentMilli;
         ++test; 
        heartRateMeasurement = 550;
        Serial.println(test);
        digitalWrite(13,HIGH);

   
      }

      if (test == 2) {
        
        heartRateMeasurement = 750;
        digitalWrite(6,HIGH);
        delay(500);
        digitalWrite(6, LOW);
        delay(3000);
        }
  
  }     

    
    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
  
  }

void updateHeartRate() {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the heart rate's measurement.

     
  */
  int heartRate = map(heartRateMeasurement, 0, 1023, 0, 100);
    Serial.print("Heart Rate is now: "); // print it
    Serial.println(heartRate);
    const unsigned char heartRateCharArray[2] = { 0, (char)heartRate };
    heartRateChar.setValue(heartRateCharArray, 2);  // and update the heart rate measurement characteristic
    oldHeartRate = heartRate;           // save the level for next comparison

}


float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}



float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
