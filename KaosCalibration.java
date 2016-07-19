/* Copyright (c) 2014 Qualcomm Technologies Inc

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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class KaosCalibration extends OpMode {

    // core competition variables
//    String teamColor;
//    String startPosition;
//    int startDelay;
//    String finalRamp;

    // core device variables
    DcMotor motorLeftFront = null;
    DcMotor motorLeftRear = null;
    DcMotor motorRightFront = null;
    DcMotor motorRightRear = null;
 //   DcMotor winchMotor = null;
 //   DcMotor armMotor = null;
 //   Servo servo = null;
//    Servo servoLeftArm = null;
//    Servo servoRightArm = null;
//    Servo winchBrake= null;
//    TouchSensor touchSensor = null;
//    OpticalDistanceSensor opticalSensor = null;
//    ColorSensor colorSensor = null;

    private static enum KAOS_DIRECTION
    {
        UNKNOWN,
        STRAIGHT,
        RIGHT,
        LEFT;

        private KAOS_DIRECTION() {
        }
    }

    int m_stateStep = 0;
    KAOS_DIRECTION m_stateDirection = KAOS_DIRECTION.UNKNOWN;
    float m_stateRotationCount = 0;

    // contructor
    public KaosCalibration() {

    }

    @Override
    public void init() {

        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        motorLeftRear = hardwareMap.dcMotor.get("motorLeftRear");
        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
        motorRightRear = hardwareMap.dcMotor.get("motorRightRear");

        //touchSensor = hardwareMap.touchSensor.get("touchSensor");
        //opticalSensor = hardwareMap.opticalDistanceSensor.get("opticalSensor");

        // since color sensor might not be on board, handle it
        try
        {
        //    colorSensor = hardwareMap.colorSensor.get("colorSensor");
        //    colorSensor.enableLed(true);
        }
        catch (Exception ex)
        {

        }
        m_stateStep = 0;

        try
        {
        //    winchMotor = hardwareMap.dcMotor.get("winchMotor") ;
        //    armMotor = hardwareMap.dcMotor.get("armMotor") ;
        }
        catch (Exception ex)
        {

        }

        try
        {
        //    hardwareMap.servo.get("servoLeftArm") ;
        //    hardwareMap.servo.get("servoRightArm") ;
        //    hardwareMap.servo.get("servo") ;
        //    hardwareMap.servo.get("winchBrake") ;
        }
        catch (Exception ex)
        {

        }
        }


        //SetLeftTrackRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS,
        //        DcMotor.Direction.REVERSE, true, 0, 0);
        //SetRightTrackRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS,
        //        DcMotor.Direction.FORWARD, true, 0, 0);


    boolean HaveMotorsCurrentPostionsReachedTargetPositions()
    {
        boolean result = false;
        double leftTargetPosition = Math.abs(RobotUtil.GetMotorTargetPosition(motorLeftFront));
        double rightTargetPosition = Math.abs(RobotUtil.GetMotorTargetPosition(motorRightFront));

        if (HaveMotorCurrentPostionReachedTargetPosition(motorLeftFront, leftTargetPosition) &&
            HaveMotorCurrentPostionReachedTargetPosition(motorRightFront, rightTargetPosition))
        {
            result = true;
        }
        return result;
    }

    boolean HaveMotorCurrentPostionReachedTargetPosition(DcMotor motor, double position)
    {
        boolean result = false;

        if (Math.abs(RobotUtil.GetMotorCurrentPosition(motor)) > position)
        {
            result = true;
        }
        return result;
    }

    @Override
    public void loop() {

        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)RobotUtil.ScaleJoystickInput(right);
        left =  (float)RobotUtil.ScaleJoystickInput(left);

        // write the values to the motors
        SetLeftTrackPower(left);
        SetRightTrackPower(right);

        // GAMEPAD 1 ----------------------------------------------------------------------------

        if (gamepad1.a)
        {
            // straight for 2500 counts
            InitStateMode(KAOS_DIRECTION.STRAIGHT, 2500);
        }
        if (gamepad1.b)
        {
            // straight for 5000 counts
            InitStateMode(KAOS_DIRECTION.STRAIGHT, 5000);
        }
        if (gamepad1.x)
        {
            // straight for 7500 counts
            InitStateMode(KAOS_DIRECTION.STRAIGHT, 7500);
        }
        if (gamepad1.y)
        {
            // straight for 10000 counts
            InitStateMode(KAOS_DIRECTION.STRAIGHT, 10000);
        }

        // GAMEPAD 2 ----------------------------------------------------------------------------

        if (gamepad2.a)
        {
            InitStateMode(KAOS_DIRECTION.LEFT, 750);
        }
        if (gamepad2.b)
        {
            InitStateMode(KAOS_DIRECTION.LEFT, 1500);
        }
        if (gamepad2.x)
        {
            InitStateMode(KAOS_DIRECTION.RIGHT, 750);
        }
        if (gamepad2.y)
        {
            InitStateMode(KAOS_DIRECTION.RIGHT, 1500);
        }

        // run the specified state mode now for testing
        // (BE SURE TO ONLY PRESS THE BUTTON ONCE FOR THE TEST TO BE ACCURATE)
        if (m_stateDirection != KAOS_DIRECTION.UNKNOWN)
        {
            RunStateMode();
        }

        // hsvValues is an array that will hold the hue, saturation, and value information.
 //       float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
//        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
//        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        /*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");
        //        telemetry.addData("isPressed", String.valueOf(touchSensor.isPressed()));
//        telemetry.addData("lightDetected", String.valueOf(opticalSensor.getLightDetected()));
//        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
//        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));

        // only do this if the color sensor is onboard
//        if (colorSensor != null) {
            // convert the RGB values to HSV values.
            //Color.RGBToHSV((colorSensor.red() * 255) / 800, (colorSensor.green() * 255) / 800, (colorSensor.blue() * 255) / 800, hsvValues);

//            Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
  //          relativeLayout.post(new Runnable() {
    //            public void run() {
      //              relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
 //           });

/*            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);
        }
    }
*/
    private void SetTrackPower(float power)
    {
        SetRightTrackPower(power);
        SetLeftTrackPower(power);
    }

    private void SetRightTrackPower(float power)
    {
        RobotUtil.SetMotorPower(motorRightFront, power);
        RobotUtil.SetMotorPower(motorRightRear, power);
    }

    private void SetLeftTrackPower(float power)
    {
        RobotUtil.SetMotorPower(motorLeftFront, power);
        RobotUtil.SetMotorPower(motorLeftRear, power);
    }

    private void StopTracks()
    {
        SetTrackPower(0);
        ResetTrackEncoders();
    }

    private void SetRightTrackDirection(DcMotor.Direction direction)
    {
        RobotUtil.SetMotorDirection(motorRightFront, direction);
        RobotUtil.SetMotorDirection(motorRightRear, direction);
    }

    private void SetLeftTrackDirection(DcMotor.Direction direction)
    {
        RobotUtil.SetMotorDirection(motorLeftFront, direction);
        RobotUtil.SetMotorDirection(motorLeftRear, direction);
    }

    private void ResetTrackEncoders()
    {
        RobotUtil.ResetEncoder(motorRightFront);
        RobotUtil.ResetEncoder(motorLeftFront);
    }

    private void SetLeftTrackRunMode(DcMotorController.RunMode runMode, DcMotor.Direction direction,
        boolean resetEncoder, int initialTargetPosition, float initialPower)
    {
        RobotUtil.SetMotorRunMode(motorLeftFront, runMode, direction, true,
                initialTargetPosition, initialPower);
        RobotUtil.SetMotorRunMode(motorLeftRear, runMode, direction, true,
                initialTargetPosition, initialPower);
    }

    private void SetRightTrackRunMode(DcMotorController.RunMode runMode, DcMotor.Direction direction,
        boolean resetEncoder, int initialTargetPosition, float initialPower)
    {
        RobotUtil.SetMotorRunMode(motorRightFront, runMode, direction, true,
                initialTargetPosition, initialPower);
        RobotUtil.SetMotorRunMode(motorRightRear, runMode, direction, true,
                initialTargetPosition, initialPower);
    }

    private void SetRightTrackTargetPosition(int position)
    {
        RobotUtil.SetMotorTargetPosition(motorRightFront, position);
        //RobotUtil.SetMotorTargetPosition(motorRightRear, position);
    }

    private void SetLeftTrackTargetPosition(int position)
    {
        RobotUtil.SetMotorTargetPosition(motorLeftFront, position);
        //RobotUtil.SetMotorTargetPosition(motorLeftRear, position);
    }

    @Override
    public void stop()
    {
        // do we need to do anything when the robot stops?
    }

    private void InitStateMode(KAOS_DIRECTION direction, int rotationCount)
    {
        // initialize the current state machine for the requested task
        m_stateStep = 0;
        m_stateDirection = direction;
        m_stateRotationCount = rotationCount;
    }

    private void ClearStateMode()
    {
        // clear the current state mode
        m_stateStep = 0;
        m_stateDirection = KAOS_DIRECTION.UNKNOWN;
        m_stateRotationCount = 0;
    }

    private float GetEncoderCountForDegreeTurn(float degree)
    {
        //TODO (move to util class)
        return 1f;
    }

    private float GetEncoderCountForStrightDistance(int inches)
    {
        //TODO (move to util class)
        return 1f;
    }

    private void RunStateMode()
    {
        float leftPower = 0.0f;
        float rightPower = 0.0f;

        switch (m_stateStep) {
            //
            // Synchronize the state machine and hardware.
            //
            case 0:
                // reset/clear encoders
                ResetTrackEncoders();
                m_stateStep++;
                break;
            case 1:
                // set encoder mode, set power and watch for position + reset coders
                switch (m_stateDirection) {
                    case STRAIGHT:
                        leftPower = 1.0f;
                        rightPower = 1.0f;
                        break;
                    case LEFT:
                        leftPower = -1.0f;
                        rightPower = 1.0f;
                        break;
                    case RIGHT:
                        leftPower = 1.0f;
                        rightPower = -1.0f;
                        break;
                }

                SetLeftTrackRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS,
                    DcMotor.Direction.REVERSE, false, 0, leftPower);
                SetRightTrackRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS,
                    DcMotor.Direction.FORWARD, false, 0, rightPower);

                // get the values directly here so they can be logged
                int motorLeftFrontCount = Math.abs(RobotUtil.GetMotorCurrentPosition(motorLeftFront));
                int motorRightFrontCount = Math.abs(RobotUtil.GetMotorCurrentPosition(motorRightFront));

                // log the current information to the phone for testing purposes
                telemetry.addData("StateMode",
                        "CurrentStep: " + String.valueOf(m_stateStep) +
                        ", SetDirection: " + String.valueOf(m_stateDirection) +
                        ", CurrentRotationRight: " + String.valueOf(motorRightFrontCount) +
                        ", CurrentRotationLeft: " + String.valueOf(motorLeftFrontCount));

                if ((motorLeftFrontCount > m_stateRotationCount) &&
                    (motorRightFrontCount > m_stateRotationCount))
                {
                    ResetTrackEncoders();
                    SetTrackPower(0.0f);
                    m_stateStep++;
                }
                break;
            case 2:
                // wait for reset encoders
                if  ((RobotUtil.GetMotorCurrentPosition(motorLeftFront) == 0)  &&
                    (RobotUtil.GetMotorCurrentPosition(motorRightFront) == 0))
                {
                    m_stateStep++;
                }
                break;
            default:
                // should never reach this code\
                ClearStateMode();
                break;
        }
    }
}
