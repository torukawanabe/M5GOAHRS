#include <M5Stack.h>
#include <Ticker.h>
#include "utility/MPU9250.h"
#include <MadgwickAHRS.h>

/*
 * Because M5 Go has magnets, we should not use electronic compass for AHRS
 * This program uses 6-axis sensor valuses for AHRS by using MadgwickAHRS library
 * - sensor refresh  == 100hz
 * - display refresh == 10hz
*/

Madgwick MadgwickFilter;

#define IMU_UPDATE_INTERVAL_SECONDS 0.01
#define DISP_UPDATE_INTERVAL_SECONDS 0.1
Ticker imuUpdater, dispUpdater;
MPU9250 IMU;

volatile float yaw, pitch, roll, refreshRate;

void updateIMU() {
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    IMU.readAccelData(IMU.accelCount);
    IMU.getAres();

    IMU.ax = (float)IMU.accelCount[0] * IMU.aRes; // - accelBias[0];
    IMU.ay = (float)IMU.accelCount[1] * IMU.aRes; // - accelBias[1];
    IMU.az = (float)IMU.accelCount[2] * IMU.aRes; // - accelBias[2];

    IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
    IMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    IMU.gx = (float)IMU.gyroCount[0] * IMU.gRes;
    IMU.gy = (float)IMU.gyroCount[1] * IMU.gRes;
    IMU.gz = (float)IMU.gyroCount[2] * IMU.gRes;

    /*
    IMU.readMagData(IMU.magCount);
    IMU.getMres();

    IMU.mx = (float)IMU.magCount[0] * IMU.mRes * IMU.magCalibration[0] -
             IMU.magbias[0];
    IMU.my = (float)IMU.magCount[1] * IMU.mRes * IMU.magCalibration[1] -
             IMU.magbias[1];
    IMU.mz = (float)IMU.magCount[2] * IMU.mRes * IMU.magCalibration[2] -
             IMU.magbias[2];
    */
    
    IMU.updateTime();
    
    MadgwickFilter.updateIMU(IMU.gx, IMU.gy, IMU.gz, IMU.ax, IMU.ay, IMU.az);
    //MadgwickFilter.update(IMU.gx, IMU.gy, IMU.gz, IMU.ax, IMU.ay, IMU.az, IMU.my, IMU.mx, IMU.mz);
    
    IMU.delt_t = millis() - IMU.count;

    roll  = MadgwickFilter.getRoll();
    pitch = MadgwickFilter.getPitch();
    yaw   = MadgwickFilter.getYaw();

    refreshRate = (float) IMU.sumCount / IMU.sum;

    IMU.count = millis();
    IMU.sumCount = 0;
    IMU.sum = 0;
  }
}

void updateDisp() {
  M5.Lcd.setCursor(0, 0); M5.Lcd.print("     x       y       z ");
  M5.Lcd.setCursor(0,  24);
  M5.Lcd.printf("% 6d  % 6d  % 6d     mg   \r\n",  (int)(1000 * IMU.ax), (int)(1000 * IMU.ay), (int)(1000 * IMU.az));
  M5.Lcd.setCursor(0,  44);
  M5.Lcd.printf("% 6d  % 6d  % 6d      o/s  \r\n", (int)(IMU.gx), (int)(IMU.gy), (int)(IMU.gz));

  M5.Lcd.setCursor(0,  100);
  M5.Lcd.printf("  yaw: % 5.2f    pitch: % 5.2f    roll: % 5.2f   \r\n", yaw, pitch, roll);
}

void setup() {
  M5.begin();
  Wire.begin();

  IMU.MPU9250SelfTest(IMU.SelfTest);
  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
  IMU.initMPU9250();
  //IMU.initAK8963(IMU.magCalibration);

  imuUpdater.attach(IMU_UPDATE_INTERVAL_SECONDS, updateIMU);
  dispUpdater.attach(DISP_UPDATE_INTERVAL_SECONDS, updateDisp);

  MadgwickFilter.begin(100); //100Hz
}

void loop() {

  M5.update();
}
