/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MPU6050_PLUGIN_H
#define MPU6050_PLUGIN_H

// #include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// #include "MPU6050.h" // not necessary if using MotionApps include file
#include "Wire.h"
#include <quadruped_base/quadruped_components.h>

namespace MPU60506DOF
{   
    class Plugin
    {
        MPU6050 imu_;
        champ::Quaternion orientation_;
        champ::Gyroscope gyro_;
        champ::Accelerometer accel_;
        champ::Magnetometer mag_;

        bool blinkState = false;
        
        // MPU control/status vars
        bool dmpReady = false;  // set true if DMP init was successful
        uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer
        
        // orientation/motion vars
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorInt16 aa;         // [x, y, z]            accel sensor measurements
        VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
        VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
        VectorFloat gravity;    // [x, y, z]            gravity vector
        VectorInt16 gy;     // [x, y, z]            gravity-free accel sensor measurements
        float euler[3];         // [psi, theta, phi]    Euler angle container
        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
        
        volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
        public:
            Plugin()
            {
                initialize();
            }

            void initialize()
            {
                delay(100);

                Wire.begin();

                Wire.setClock(400000);
                imu_.initialize();
                // Serial.println(imu_.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
                devStatus = imu_.dmpInitialize();
                imu_.setXGyroOffset(220);
                imu_.setYGyroOffset(76);
                imu_.setZGyroOffset(-85);
                imu_.setZAccelOffset(1788); // 1688 factory default for my test chip
                // make sure it worked (returns 0 if so)
                if (devStatus == 0) {
                    // Calibration Time: generate offsets and calibrate our MPU6050
                    imu_.CalibrateAccel(6);
                    imu_.CalibrateGyro(6);
                    imu_.PrintActiveOffsets();
                    // turn on the DMP, now that it's ready
                    // Serial.println(F("Enabling DMP..."));
                    imu_.setDMPEnabled(true);

                    // enable Arduino interrupt detection
                    // Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
                    // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
                    // Serial.println(F(")..."));
                    // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
                    mpuIntStatus = imu_.getIntStatus();

                    // set our DMP Ready flag so the main loop() function knows it's okay to use it
                    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
                    dmpReady = true;

                    // get expected DMP packet size for later comparison
                    packetSize = imu_.dmpGetFIFOPacketSize();
                } else {
                    // ERROR!
                    // 1 = initial memory load failed
                    // 2 = DMP configuration updates failed
		    dmpReady = false;
                    // (if it's going to break, usually the code will be 1)
                    // Serial.print(F("DMP Initialization failed (code "));
                    // Serial.print(devStatus);
                    // Serial.println(F(")"));
                }

            }

            void readchamp(champ::Quaternion &orientation)
            {
                if (!dmpReady) return;
                // read a packet from FIFO
                if (imu_.dmpGetCurrentFIFOPacket(fifoBuffer)) // Get the Latest packet 
                {
                    imu_.dmpGetQuaternion(&q, fifoBuffer);
                    orientation.w = q.w;
                    orientation.x = q.x;
                    orientation.y = q.y;
                    orientation.z = q.z;

                    // to euler
                    //https://stackoverflow.com/questions/30279065/how-to-get-the-euler-angles-from-the-orientation-vector-sensor-type-orientation-vecto
                    // orientation.x  = atan2f(-2.* (q[2] * q[3] - q[0] * q[1]) , q[0] * q[0] - q[1] * q[1]- q[2] * q[2] + q[3] * q[3]); 
                    // orientation.y  = asinf(2. * (q[1] * q[3] + q[0] * q[2]));
                    // orientation.z  = atan2f( 2. * (-q[1] * q[2] + q[0] * q[3]) , q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]); 
                }   
            }

            void readGyroscope(champ::Gyroscope &gyro)
            {
                if (!dmpReady) return;
                // read a packet from FIFO
                if (imu_.dmpGetCurrentFIFOPacket(fifoBuffer)) // Get the Latest packet 
                {
                    imu_.dmpGetGyro(&gy, fifoBuffer);
                    gyro.x = gy.x;
                    gyro.y = gy.y;
                    gyro.z = gy.z;
                }
            }

            void readAccelerometer(champ::Accelerometer &accel)
            {
                if (!dmpReady) return;
                // read a packet from FIFO
                if (imu_.dmpGetCurrentFIFOPacket(fifoBuffer)) // Get the Latest packet 
                {
                    imu_.dmpGetQuaternion(&q, fifoBuffer);
                    imu_.dmpGetAccel(&aa, fifoBuffer);
                    accel.x = aa.x;
                    accel.y = aa.y;
                    accel.z = aa.z;
                    // imu_.dmpGetGravity(&gravity, &q);
                    // imu_.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                    // accel.x = aaReal.x;
                    // accel.y = aaReal.y;
                    // accel.z = aaReal.z;
                }
            }

            void readMagnetometer(champ::Magnetometer &mag)
            {
                if (!dmpReady) return;
                // read a packet from FIFO
                if (imu_.dmpGetCurrentFIFOPacket(fifoBuffer)) // Get the Latest packet 
                {
                    mag.x = 0.0;
                    mag.y = 0.0;
                    mag.z = 0.0;
                }
            }

            void read(champ::Quaternion &orientation, champ::Accelerometer &accel, champ::Gyroscope &gyro, champ::Magnetometer &mag)
            {
                orientation.w = orientation_.w;
                orientation.x = orientation_.x;
                orientation.y = orientation_.y;
                orientation.z = orientation_.z;

                gyro.x = gyro_.x;
                gyro.y = gyro_.y;
                gyro.z = gyro_.z;

                accel.x = accel_.x;
                accel.y = accel_.y;
                accel.z = accel_.z;

                mag.x = mag_.x;
                mag.y = mag_.y;
                mag.z = mag_.z;
            }

            void run()
            {
                if (!dmpReady) return;
                // read a packet from FIFO
                if (imu_.dmpGetCurrentFIFOPacket(fifoBuffer)) // Get the Latest packet 
                {
                    imu_.dmpGetQuaternion(&q, fifoBuffer);
                    imu_.dmpGetAccel(&aa, fifoBuffer);
                    // imu_.dmpGetGravity(&gravity, &q);
                    // imu_.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                    imu_.dmpGetGyro(&gy, fifoBuffer);
                    orientation_.w = q.w;
                    orientation_.x = q.x;
                    orientation_.y = q.y;
                    orientation_.z = q.z;

                    gyro_.x = gy.x;
                    gyro_.y = gy.y;
                    gyro_.z = gy.z;

                    accel_.x = aa.x;
                    accel_.y = aa.y;
                    accel_.z = aa.z;
                    // accel_.x = aaReal.x;
                    // accel_.y = aaReal.y;
                    // accel_.z = aaReal.z;

                    mag_.x = 0.0;
                    mag_.y = 0.0;
                    mag_.z = 0.0;
                }
            }
    };
}

#endif

