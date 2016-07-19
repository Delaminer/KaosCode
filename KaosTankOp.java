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
public class KaosTankOp extends OpMode {

    // core competition variables
    String teamColor;
    String startPosition;
    int startDelay;
    String finalRamp;

    // core device variables
    DcMotor motorLeftFront;
    DcMotor motorLeftRear;
    DcMotor motorRightFront;
    DcMotor motorRightRear;

    DcMotor motorWinch;
    DcMotor motorArm;

    Servo servoColorButton;
    Servo servoLeftArm;
    Servo servoRightArm;

    TouchSensor touchSensor;
    OpticalDistanceSensor opticalSensor;
    ColorSensor colorSensorLine;
    ColorSensor colorSensorButton;

    int m_state = 0;

    // contructor
    public KaosTankOp() {

    }

    @Override
    public void init() {
        try
        {
            // controller A (left joystick)
            motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
            motorLeftRear = hardwareMap.dcMotor.get("motorLeftRear");
            // controller A (right joystick)
            motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
            motorRightRear = hardwareMap.dcMotor.get("motorRightRear");

            // only used in autonomous mode
            touchSensor = hardwareMap.touchSensor.get("touchSensor");
            opticalSensor = hardwareMap.opticalDistanceSensor.get ("opticalSensor");

            // controller B (left joystick)
            motorWinch = hardwareMap.dcMotor.get("motorWinch");
            // controller B (right joystick)
            motorArm = hardwareMap.dcMotor.get("motorArm");

            // only used in autonomous mode
            servoColorButton = hardwareMap.servo.get("servoColorButton");

            // controller B (LT down, LB up)
            servoLeftArm = hardwareMap.servo.get("servoLeftArm");
            // controller B (RT down, RB up)
            servoRightArm = hardwareMap.servo.get("servoRightArm");

            // only used in autonomous mode
            colorSensorLine = hardwareMap.colorSensor.get("colorSensorLine");
            colorSensorLine.enableLed(true);

            // only used in autonomous mode
            colorSensorButton = hardwareMap.colorSensor.get("colorSensorButton");
            colorSensorButton.enableLed(true);
        }
        catch (Exception ex)
        {

        }

        // set the initial motor directions
        //SetLeftTrackDirection(DcMotor.Direction.REVERSE);
        //SetRightTrackDirection(DcMotor.Direction.FORWARD);

        SetLeftTrackRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS,
                DcMotor.Direction.REVERSE, true, 0, 0);
        SetRightTrackRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS,
                DcMotor.Direction.FORWARD, true, 0, 0);

        // start with the power set to zero
        //SetTrackPower(0);
    }

    private void RunTestB()
    {
        telemetry.addData("test mode", "running test B");
        // initialize the tracks to go STRAIGHT
        SetLeftTrackRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS,
                DcMotor.Direction.REVERSE, true, 0, 1);
        SetRightTrackRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS,
                DcMotor.Direction.FORWARD, true, 0, 1);

//
//        while (!HaveMotorsCurrentPostionsReachedTargetPositions())
//        {
//            try
//            {
//                Thread.sleep(10);
//            }
//            catch (Exception ex)
//            {
//
//            }
//        }
//
//        StopTracks();
//
//        // turn left a bit
//        SetLeftTrackRunMode(DcMotorController.RunMode.RUN_TO_POSITION,
//            DcMotor.Direction.FORWARD, true, 10, 1);
//        SetRightTrackRunMode(DcMotorController.RunMode.RUN_TO_POSITION,
//            DcMotor.Direction.FORWARD, true, 10, 1);
//
//        while (!HaveMotorsCurrentPostionsReachedTargetPositions())
//        {
//            try
//            {
//                Thread.sleep(10);
//            }
//            catch (Exception ex)
//            {
//
//            }
//        }
//
//        StopTracks();
//
//        // go straight again
//        SetLeftTrackRunMode(DcMotorController.RunMode.RUN_TO_POSITION,
//            DcMotor.Direction.REVERSE, true, 10, 1);
//        SetRightTrackRunMode(DcMotorController.RunMode.RUN_TO_POSITION,
//            DcMotor.Direction.FORWARD, true, 10, 1);
//
//        while (!HaveMotorsCurrentPostionsReachedTargetPositions())
//        {
//            try
//            {
//                Thread.sleep(10);
//            }
//            catch (Exception ex)
//            {
//
//            }
//        }
//
//        StopTracks();
    }

    private void RunTestA()
    {
        telemetry.addData("test mode", "running test A");
        // initialize the tracks to go STRAIGHT
        SetLeftTrackRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS,
                DcMotor.Direction.FORWARD, true, 0, 1);
        SetRightTrackRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS,
                DcMotor.Direction.FORWARD, true, 0, 1);

//        // initialize the tracks to go STRAIGHT
//        SetLeftTrackRunMode(DcMotorController.RunMode.RUN_TO_POSITION,
//                DcMotor.Direction.REVERSE, true, 10, 1);
//        SetRightTrackRunMode(DcMotorController.RunMode.RUN_TO_POSITION,
//                DcMotor.Direction.FORWARD, true, 10, 1);
//
//        // TEST!!!!
//        // does the robot stop after 10 rotations of the motors now?
//        try
//        {
//            Thread.sleep(1000);
//        }
//        catch (Exception ex)
//        {
//
//        }
//
//        // stop anyway
//        StopTracks();
//
//        // turn the robot to the left
//        SetLeftTrackRunMode(DcMotorController.RunMode.RUN_TO_POSITION,
//                DcMotor.Direction.FORWARD, true, 10, 1);
//        SetRightTrackRunMode(DcMotorController.RunMode.RUN_TO_POSITION,
//                DcMotor.Direction.FORWARD, true, 10, 1);
//
//        // TEST!!!!
//        // does the robot stop after 10 rotations of the motors now?
//        try
//        {
//            Thread.sleep(1000);
//        }
//        catch (Exception ex)
//        {
//
//        }
//
//        // stop anyway
//        StopTracks();
//
//        // go straight again
//        SetLeftTrackRunMode(DcMotorController.RunMode.RUN_TO_POSITION,
//                DcMotor.Direction.REVERSE, true, 10, 1);
//        SetRightTrackRunMode(DcMotorController.RunMode.RUN_TO_POSITION,
//                DcMotor.Direction.FORWARD, true, 10, 1);
//
//        // TEST!!!!
//        // does the robot stop after 10 rotations of the motors now?
//        try
//        {
//            Thread.sleep(1000);
//        }
//        catch (Exception ex)
//        {
//
//        }
//
//        // stop anyway
//        StopTracks();
    }

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

    private void TestAuto1()
    {
        switch (m_state) {
            //
            // Synchronize the state machine and hardware.
            //
            case 0:
                // reset/clear encoders
                ResetTrackEncoders();
                m_state++;
                break;
            case 1:
                // set encoder mode, set power and watch for position + reset coders
                SetLeftTrackRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS,
                        DcMotor.Direction.REVERSE, false, 0, 1.0f);
                SetRightTrackRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS,
                        DcMotor.Direction.FORWARD, false, 0, 1.0f);

                if (HaveMotorCurrentPostionReachedTargetPosition(motorLeftFront, 1500f) &&
                    HaveMotorCurrentPostionReachedTargetPosition(motorRightFront, 1500f))
                {
                    ResetTrackEncoders();
                    SetTrackPower(0.0f);
                    m_state++;
                }
                break;
            case 2:
                // wait for reset encoders
                if  ((RobotUtil.GetMotorCurrentPosition(motorLeftFront) == 0)  &&
                    (RobotUtil.GetMotorCurrentPosition(motorRightFront) == 0))
                {
                    m_state++;
                }
                break;
            case 3:
                // turn left
                SetLeftTrackRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS,
                        DcMotor.Direction.REVERSE, false, 0, -1.0f);
                SetRightTrackRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS,
                        DcMotor.Direction.FORWARD, false, 0, 1.0f);

                if (HaveMotorCurrentPostionReachedTargetPosition(motorLeftFront, 1500f) &&
                        HaveMotorCurrentPostionReachedTargetPosition(motorRightFront, 1500f))
                {
                    ResetTrackEncoders();
                    SetTrackPower(0.0f);
                    m_state++;
                }
                break;
            case 4:
                // wait for reset encoders
                if  ((RobotUtil.GetMotorCurrentPosition(motorLeftFront) == 0)  &&
                        (RobotUtil.GetMotorCurrentPosition(motorRightFront) == 0))
                {
                    m_state++;
                }
                break;
            case 5:
                //go straight
                SetLeftTrackRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS,
                        DcMotor.Direction.REVERSE, false, 0, 1.0f);
                SetRightTrackRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS,
                        DcMotor.Direction.FORWARD, false, 0, 1.0f);

                if (HaveMotorCurrentPostionReachedTargetPosition(motorLeftFront, 3500f) &&
                        HaveMotorCurrentPostionReachedTargetPosition(motorRightFront, 3500f))
                {
                    ResetTrackEncoders();
                    SetTrackPower(0.0f);
                    m_state++;
                }
                break;
            case 6:
                // wait for reset encoders
                if  ((RobotUtil.GetMotorCurrentPosition(motorLeftFront) == 0)  &&
                        (RobotUtil.GetMotorCurrentPosition(motorRightFront) == 0))
                {
                    m_state++;
                }
                break;
            case 7:
                // turn right
                SetLeftTrackRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS,
                        DcMotor.Direction.REVERSE, false, 0, 1.0f);
                SetRightTrackRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS,
                        DcMotor.Direction.FORWARD, false, 0, -1.0f);

                if (HaveMotorCurrentPostionReachedTargetPosition(motorLeftFront, 1500f) &&
                        HaveMotorCurrentPostionReachedTargetPosition(motorRightFront, 1500f))
                {
                    ResetTrackEncoders();
                    SetTrackPower(0.0f);
                    m_state++;
                }
                break;
            case 8:
                // wait for reset encoders
                if  ((RobotUtil.GetMotorCurrentPosition(motorLeftFront) == 0)  &&
                        (RobotUtil.GetMotorCurrentPosition(motorRightFront) == 0))
                {
                    m_state++;
                }
                break;
            case 9:
                // turn right
                SetLeftTrackRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS,
                        DcMotor.Direction.REVERSE, false, 0, 1.0f);
                SetRightTrackRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS,
                        DcMotor.Direction.FORWARD, false, 0, 1.0f);

                if (HaveMotorCurrentPostionReachedTargetPosition(motorLeftFront, 3500f) &&
                        HaveMotorCurrentPostionReachedTargetPosition(motorRightFront, 3500f))
                {
                    ResetTrackEncoders();
                    SetTrackPower(0.0f);
                    m_state++;
                }
                break;
            case 10:
                // wait for reset encoders
                if  ((RobotUtil.GetMotorCurrentPosition(motorLeftFront) == 0)  &&
                        (RobotUtil.GetMotorCurrentPosition(motorRightFront) == 0))
                {
                    m_state++;
                }
                break;
            default:
        }
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

        //TestAuto1();

        if (gamepad1.a)
        {
            //RunTestA();
        }

        if (gamepad1.b)
        {
            //RunTestB();
        }

        if(touchSensor.isPressed())
        {
            // what should you do here?
            StopTracks();
        }

        if (opticalSensor.getLightDetected () > 0.8)
        {
            // what should you do here (and what vlaue should WE use?)

        }

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        /*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("isPressed", String.valueOf(touchSensor.isPressed()));
        //telemetry.addData("lightDetected", String.valueOf(opticalSensor.getLightDetected()));
        //telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        //telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));

        // only do this if the color sensor is onboard
        if (colorSensorButton != null) {
            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensorButton.red(), colorSensorButton.green(), colorSensorButton.blue(), hsvValues);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.addData("Clear", colorSensorButton.alpha());
            telemetry.addData("Red  ", colorSensorButton.red());
            telemetry.addData("Green", colorSensorButton.green());
            telemetry.addData("Blue ", colorSensorButton.blue());
            telemetry.addData("Hue", hsvValues[0]);
        }
    }

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
        RobotUtil.SetMotorRunMode(motorRightFront,runMode, direction, true,
                initialTargetPosition, initialPower);
        RobotUtil.SetMotorRunMode(motorRightRear,runMode, direction, true,
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
}
