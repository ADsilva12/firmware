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

#ifndef BNO080_PLUGIN_H
#define BNO080_PLUGIN_H

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <quadruped_base/quadruped_components.h>

namespace BNO0809DOF
{   
    class Plugin
    {
        BNO080 imu_;
        champ::Quaternion orientation_;
        champ::Gyroscope gyro_;
        champ::Accelerometer accel_;
        champ::Magnetometer mag_;

        public:
            Plugin()
            {
                initialize();
            }

            void initialize()
            {
                delay(100);

                Wire.begin();
                imu_.begin(0x4B, Wire, 15);

                Wire.setClock(400000);
                imu_.enableRotationVector(50);
                imu_.enableAccelerometer(50);
                imu_.enableGyro(50); 
                imu_.enableMagnetometer(50);
            }

            void readchamp(champ::Quaternion &orientation)
            {
                if(imu_.dataAvailable())
                {
                    orientation.w = imu_.getQuatReal();
                    orientation.x = imu_.getQuatI();
                    orientation.y = imu_.getQuatJ();
                    orientation.z = imu_.getQuatK();

                    // to euler
                    //https://stackoverflow.com/questions/30279065/how-to-get-the-euler-angles-from-the-orientation-vector-sensor-type-orientation-vecto
                    // orientation.x  = atan2f(-2.* (q[2] * q[3] - q[0] * q[1]) , q[0] * q[0] - q[1] * q[1]- q[2] * q[2] + q[3] * q[3]); 
                    // orientation.y  = asinf(2. * (q[1] * q[3] + q[0] * q[2]));
                    // orientation.z  = atan2f( 2. * (-q[1] * q[2] + q[0] * q[3]) , q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]); 
                }   
            }

            void readGyroscope(champ::Gyroscope &gyro)
            {
                if(imu_.dataAvailable())
                {
                    gyro.x = imu_.getGyroX();
                    gyro.y = imu_.getGyroY();
                    gyro.z = imu_.getGyroZ();
                } 
            }

            void readAccelerometer(champ::Accelerometer &accel)
            {
                if(imu_.dataAvailable())
                {
                    accel.x = imu_.getAccelX();
                    accel.y = imu_.getAccelY();
                    accel.z = imu_.getAccelZ();
                }
            }

            void readMagnetometer(champ::Magnetometer &mag)
            {
                if(imu_.dataAvailable())
                {
                    mag.x = imu_.getMagX();
                    mag.y = imu_.getMagY();
                    mag.z = imu_.getMagZ();
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
                if(imu_.dataAvailable())
                {
                    orientation_.w = imu_.getQuatReal();
                    orientation_.x = imu_.getQuatI();
                    orientation_.y = imu_.getQuatJ();
                    orientation_.z = imu_.getQuatK();

                    gyro_.x = imu_.getGyroX();
                    gyro_.y = imu_.getGyroY();
                    gyro_.z = imu_.getGyroZ();

                    accel_.x = imu_.getAccelX();
                    accel_.y = imu_.getAccelY();
                    accel_.z = imu_.getAccelZ();

                    mag_.x = imu_.getMagX();
                    mag_.y = imu_.getMagY();
                    mag_.z = imu_.getMagZ();
                }
            }
    };
}

#endif

