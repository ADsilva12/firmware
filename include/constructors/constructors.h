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

#ifndef CONSTRUCTORS_H
#define CONSTRUCTORS_H

#include <actuators/actuator.h>
#include <imu/imu.h>
#include <gait_config.h>
#include <quadruped_base/quadruped_components.h>
#include <comms/input_interfaces/single_input_interface.h>
#include <comms/input_interfaces/dual_input_interface.h>
#include <comms/status_interfaces/status_interface.h>
#include <comms/input_interfaces/rosserial_interface.h>

#ifdef USE_SIMULATION_ACTUATOR
    #include <actuators/simulation_actuator_plugin.h>

    SimulationActuator::Plugin lfh_actuator;
    SimulationActuator::Plugin lfu_actuator;
    SimulationActuator::Plugin lfl_actuator;

    SimulationActuator::Plugin rfh_actuator;
    SimulationActuator::Plugin rfu_actuator;
    SimulationActuator::Plugin rfl_actuator;

    SimulationActuator::Plugin lhh_actuator;
    SimulationActuator::Plugin lhu_actuator;
    SimulationActuator::Plugin lhl_actuator;

    SimulationActuator::Plugin rhh_actuator;
    SimulationActuator::Plugin rhu_actuator;
    SimulationActuator::Plugin rhl_actuator;

    Actuator<SimulationActuator::Plugin> actuators
    (
        PANTOGRAPH_LEG,
        lfh_actuator, lfu_actuator, lfl_actuator,
        rfh_actuator,rfu_actuator,rfl_actuator,
        lhh_actuator,lhu_actuator,lhl_actuator,
        rhh_actuator,rhu_actuator, rhl_actuator
    );
#endif 

#ifdef USE_SERVO_ACTUATOR
    #include <actuators/digital_servo_plugin.h>

    DigitalServo::Plugin lfh_actuator(LFH_PIN, -2.35619, 2.35619, LFH_OFFSET, LFH_INV);
    DigitalServo::Plugin lfu_actuator(LFU_PIN, -2.35619, 2.35619, LFU_OFFSET, LFU_INV);
    DigitalServo::Plugin lfl_actuator(LFL_PIN, -2.35619, 2.35619, LFL_OFFSET, LFL_INV);

    DigitalServo::Plugin rfh_actuator(RFH_PIN, -2.35619, 2.35619, RFH_OFFSET, RFH_INV);
    DigitalServo::Plugin rfu_actuator(RFU_PIN, -2.35619, 2.35619, RFU_OFFSET, RFU_INV);
    DigitalServo::Plugin rfl_actuator(RFL_PIN, -2.35619, 2.35619, RFL_OFFSET, RFL_INV);

    DigitalServo::Plugin lhh_actuator(LHH_PIN, -2.35619, 2.35619, LHH_OFFSET, LHH_INV);
    DigitalServo::Plugin lhu_actuator(LHU_PIN, -2.35619, 2.35619, LHU_OFFSET, LHU_INV);
    DigitalServo::Plugin lhl_actuator(LHL_PIN, -2.35619, 2.35619, LHL_OFFSET, LHL_INV);

    DigitalServo::Plugin rhh_actuator(RHH_PIN, -2.35619, 2.35619, RHH_OFFSET, RHH_INV);
    DigitalServo::Plugin rhu_actuator(RHU_PIN, -2.35619, 2.35619, RHU_OFFSET, RHU_INV);
    DigitalServo::Plugin rhl_actuator(RHL_PIN, -2.35619, 2.35619, RHL_OFFSET, RHL_INV);

    Actuator<DigitalServo::Plugin> actuators
    (
        PANTOGRAPH_LEG,
        lfh_actuator, lfu_actuator, lfl_actuator,
        rfh_actuator, rfu_actuator, rfl_actuator,
        lhh_actuator, lhu_actuator, lhl_actuator,
        rhh_actuator, rhu_actuator, rhl_actuator
    );
#endif 

#ifdef USE_DYNAMIXEL_ACTUATOR
    #include <actuators/dynamixel12a_plugin.h>

    OneWireMInterface ax12Interface(Serial1);
    // (serial_interface, actuator_leg_id, actuator_driver_id,  min_angle,max_angle, inverted)
    DynamixelAX12A::Plugin lfh_actuator(ax12Interface, 0,  LFH_SERVO_ID, 0, 0, LFH_INV);
    DynamixelAX12A::Plugin lfu_actuator(ax12Interface, 1,  LFU_SERVO_ID, 0, 0, LFU_INV);
    DynamixelAX12A::Plugin lfl_actuator(ax12Interface, 2,  LFL_SERVO_ID, 0, 0, LFL_INV);

    DynamixelAX12A::Plugin rfh_actuator(ax12Interface, 3,  RFH_SERVO_ID, 0, 0, RFH_INV);
    DynamixelAX12A::Plugin rfu_actuator(ax12Interface, 4,  RFU_SERVO_ID, 0, 0, RFU_INV);
    DynamixelAX12A::Plugin rfl_actuator(ax12Interface, 5,  RFL_SERVO_ID, 0, 0, RFL_INV);

    DynamixelAX12A::Plugin lhh_actuator(ax12Interface, 6,  LHH_SERVO_ID, 0, 0, LHH_INV);
    DynamixelAX12A::Plugin lhu_actuator(ax12Interface, 7,  LHU_SERVO_ID, 0, 0, LHU_INV);
    DynamixelAX12A::Plugin lhl_actuator(ax12Interface, 8,  LHL_SERVO_ID, 0, 0, LHL_INV);

    DynamixelAX12A::Plugin rhh_actuator(ax12Interface, 9,  RHH_SERVO_ID, 0, 0, RHH_INV);
    DynamixelAX12A::Plugin rhu_actuator(ax12Interface, 10, RHU_SERVO_ID, 0, 0, RHU_INV);
    DynamixelAX12A::Plugin rhl_actuator(ax12Interface, 11, RHL_SERVO_ID, 0, 0, RHL_INV);

    Actuator<DynamixelAX12A::Plugin> actuators
    (
        PANTOGRAPH_LEG,
        lfh_actuator, lfu_actuator, lfl_actuator,
        rfh_actuator, rfu_actuator ,rfl_actuator,
        lhh_actuator, lhu_actuator, lhl_actuator,
        rhh_actuator, rhu_actuator, rhl_actuator
    );
#endif 

#ifdef USE_SIMULATION_IMU
    #include <imu/simulation_imu_plugin.h>

    IMU<SimulationIMU::Plugin> imu;
#endif

#ifdef USE_BNO0809DOF_IMU
    #include <imu/bno080_plugin.h>

    IMU<BNO0809DOF::Plugin> imu;
#endif

champ::GaitConfig gait_config(
    KNEE_ORIENTATION,
    PANTOGRAPH_LEG,
    MAX_LINEAR_VELOCITY_X,
    MAX_LINEAR_VELOCITY_Y,
    MAX_ANGULAR_VELOCITY_Z,
    COM_X_TRANSLATION,
    SWING_HEIGHT,
    STANCE_DEPTH,
    STANCE_DURATION,
    NOMINAL_HEIGHT
);

#ifdef USE_ROS
    #include <comms/input_interfaces/rosserial_interface.h>

    champ::Interfaces::ROSSerial ros_interface;
    champ::Interfaces::Status<champ::Interfaces::ROSSerial> status_interface(ros_interface);
    champ::Interfaces::SingleInput<champ::Interfaces::ROSSerial> command_interface(ros_interface);
#endif

#ifdef USE_ROS_RF
    #include <comms/input_interfaces/rosserial_interface.h>
    #include <comms/input_interfaces/rf_interface.h>

    champ::Interfaces::ROSSerial ros_interface;
    champ::Interfaces::RF rf_interface(ELE_PIN, RUD_PIN, AIL_PIN, THR_PIN, AUX1_PIN, AUX2_PIN);

    champ::Interfaces::Status<champ::Interfaces::ROSSerial> status_interface(ros_interface);
    champ::Interfaces::DualInput<champ::Interfaces::ROSSerial, champ::Interfaces::RF> command_interface(ros_interface, rf_interface);
#endif

#endif