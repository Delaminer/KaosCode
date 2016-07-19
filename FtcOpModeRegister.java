/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

/**
 * Register Op Modes
 */
public class FtcOpModeRegister implements OpModeRegister {

  /**
   * The Op Mode Manager will call this method when it wants a list of all
   * available op modes. Add your op mode to the list to enable it.
   *
   * @param manager op mode manager
   */
  public void register(OpModeManager manager) {

    /*
     * register your op modes here.
     * The first parameter is the name of the op mode
     * The second parameter is the op mode class property
     *
     * If two or more op modes are registered with the same name, the app will display an error.
     */

    //manager.register("NullOp", NullOp.class);
    //manager.register("RobotBootAutoOldA", RobotBootAutoOldA.class);
    //manager.register("MatrixK9TeleOp", MatrixK9TeleOp.class);
    //manager.register("RobotBootAutoA", RobotBootAutoA.class);
//    manager.register("RobotBootAutoABlue", RobotBootAutoABlue.class);
//    manager.register("RobotBootAutoARed", RobotBootAutoARed.class);
//    manager.register("RobotBootAutoB", RobotBootAutoB.class);
//    manager.register("RobotBootAutoCBlue", RobotBootAutoCBlue.class);
//    manager.register("RobotBootAutoCRed", RobotBootAutoCRed.class);
//    //manager.register("RobotBootTeleOp", RobotBootTeleOp.class);
//    manager.register("RobotBootTeleOpRed", RobotBootTeleOpRed.class);
//    manager.register("RobotBootTeleOpBlue", RobotBootTeleOpBlue.class);
    //manager.register("RobotBoot", RobotBoot.class);
    //manager.register("KaosCalibration", KaosCalibration.class);
    //manager.register("KILTTankDrive", KILTTankDrive.class);
    //manager.register ("PushBotAuto", PushBotAuto.class);
    //manager.register ("PushBotManual", PushBotManual.class);
    //manager.register("Sensor", Sensor.class);
    manager.register("BlueAutoA", RobotBootAutoABlue.class);
    manager.register("BlueAutoB", RobotBootAutoB.class);
    manager.register("BlueAutoC", RobotBootAutoCBlue.class);
    manager.register("BlueTeleOp", RobotBootTeleOpBlue.class);

    manager.register("RedAutoA", RobotBootAutoARed.class);
    manager.register("RedAutoB", RobotBootAutoB.class);
    manager.register("RedAutoC", RobotBootAutoCRed.class);
    manager.register("RedTeleOp", RobotBootTeleOpRed.class);



    /*
     * Uncomment any of the following lines if you want to register an op mode.
     */
    //manager.register("MR Gyro Test", MRGyroTest.class);

    //manager.register("AdafruitRGBExample", AdafruitRGBExample.class);
    //manager.register("ColorSensorDriver", ColorSensorDriver.class);

    //manager.register("IrSeekerOp", IrSeekerOp.class);
    //manager.register("CompassCalibration", CompassCalibration.class);
    //manager.register("I2cAddressChangeExample", LinearI2cAddressChange.class);


    //manager.register("NxtTeleOp", NxtTeleOp.class);

    //manager.register("LinearK9TeleOp", LinearK9TeleOp.class);
    //manager.register("LinearIrExample", LinearIrExample.class);


    //manager.register ("PushBotManual1", PushBotManual1.class);
    //manager.register ("PushBotAutoSensors", PushBotAutoSensors.class);
    //manager.register ("PushBotIrEvent", PushBotIrEvent.class);

    //manager.register ("PushBotManualSensors", PushBotManualSensors.class);
    //manager.register ("PushBotOdsDetectEvent", PushBotOdsDetectEvent.class);
    //manager.register ("PushBotOdsFollowEvent", PushBotOdsFollowEvent.class);
    //manager.register ("PushBotTouchEvent", PushBotTouchEvent.class);

    //manager.register("PushBotDriveTouch", PushBotDriveTouch.java);
    //manager.register("PushBotIrSeek", PushBotIrSeek.java);
    //manager.register("PushBotSquare", PushBotSquare.java);
  }
}
