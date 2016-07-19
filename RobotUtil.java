package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by HowellFTCUser1 on 10/17/2015.
 */
public class RobotUtil
{
    public static double GetServoPosition (Servo servo)
    {
        double result = 0.0;

        try
        {
            if (servo != null)
            {
                servo.getPosition();
            }
        }
        catch (Exception ex)
        {
            // log this
            DbgLog.msg("RobotUtil:GetServoPosition() - Exception caught:" + ex.toString());
        }
        return result;
    }

    public static void SetServoPosition (Servo servo, double position)
    {
        // make sure the position passed in within the legal range
        double clippedPosition = Range.clip(position, Servo.MIN_POSITION, Servo.MAX_POSITION);

        try
        {
            if (servo != null)
            {
                servo.setPosition(clippedPosition);
            }
        }
        catch (Exception ex)
        {
            // log this
            DbgLog.msg("RobotUtil:SetServoPosition() - Exception caught:" + ex.toString());
        }
    }

    public static double GetMotorPower (DcMotor motor)
    {
        double result = 0.0;

        try
        {
            if (motor != null)
            {
                result = motor.getPower ();
            }
        }
        catch (Exception ex)
        {
            // log this
            DbgLog.msg("RobotUtil:GetMotorPower() - Exception caught:" + ex.toString());
        }
        return result;
    }

    public static void SetMotorPower(DcMotor motor, float power)
    {
        try
        {
            if (motor != null)
            {
                motor.setPower(power);
            }
        }
        catch (Exception ex)
        {
            // log this
            DbgLog.msg("RobotUtil:SetMotorPower() - Exception caught:" + ex.toString());
        }
    }

    public static void ResetEncoder(DcMotor motor)
    {
        try
        {
            if (motor != null)
            {
                motor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            }
        }
        catch (Exception ex)
        {
            // log this
            DbgLog.msg("RobotUtil:ResetEncoder() - Exception caught:" + ex.toString());
        }
    }

    public static void SetMotorDirection(DcMotor motor, DcMotor.Direction direction)
    {
        try
        {
            if (motor != null)
            {
                motor.setDirection(direction);
            }
        } catch (Exception ex)
        {
            // log this
            DbgLog.msg("RobotUtil:SetMotorDirection() - Exception caught:" + ex.toString());
        }
    }

    public static void SetServoDirection(Servo servo, Servo.Direction direction)
    {
        try
        {
            if (servo != null)
            {
                servo.setDirection(direction);
            }
        } catch (Exception ex)
        {
            // log this
            DbgLog.msg("RobotUtil:SetServoDirection() - Exception caught:" + ex.toString());
        }
    }

    public static int GetMotorCurrentPosition (DcMotor motor)
    {
        int result = 0;

        try
        {
            if (motor != null)
            {
                result = Math.abs(motor.getCurrentPosition());
            }
        }
        catch (Exception ex)
        {
            // log this
            DbgLog.msg("RobotUtil:GetMotorCurrentPosition() - Exception caught:" + ex.toString());
        }
        return result;
    }

    public static int GetMotorTargetPosition (DcMotor motor)
    {
        int result = 0;

        try
        {
            if (motor != null)
            {
                result = Math.abs(motor.getTargetPosition());
            }
        }
        catch (Exception ex)
        {
            // log this
            DbgLog.msg("RobotUtil:GetMotorTargetPosition() - Exception caught:" + ex.toString());
        }
        return result;
    }

    public static void SetMotorTargetPosition (DcMotor motor, int position)
    {
        try
        {
            if (motor != null)
            {
                motor.setTargetPosition(position);
            }
        }
        catch (Exception ex)
        {
            // log this
            DbgLog.msg("RobotUtil:SetMotorTargetPosition() - Exception caught:" + ex.toString());
        }
    }

    public static void SetMotorRunMode(DcMotor motor, DcMotorController.RunMode runMode,
        DcMotor.Direction direction, boolean resetEncoder, int initialTargetPosition, float InitialPower)
    {
        try
        {
            if (motor != null)
            {
                // check to see if the caller wants the encoders reset
                if (resetEncoder)
                {
                    ResetEncoder(motor);
                }

                // set the specified motor run mode
                if (runMode != null)
                {
                    motor.setMode(runMode);

                    if (runMode == DcMotorController.RunMode.RUN_TO_POSITION)
                    {
                       SetMotorTargetPosition(motor, initialTargetPosition);
                    }
                }

                // set the specified motor direction
                if (direction != null)
                {
                    SetMotorDirection(motor, direction);
                }

                // set the initial power
                SetMotorPower(motor, InitialPower);
            }
        }
        catch (Exception ex)
        {
            // log this
            DbgLog.msg("RobotUtil:SetMotorRunMode() - Exception caught:" + ex.toString());
        }
    }

    public static void SetMotorRunMode(DcMotor motor, DcMotorController.RunMode runMode,
        boolean resetEncoder, int initialTargetPosition, float InitialPower)
    {
        try
        {
            if (motor != null)
            {
                // check to see if the caller wants the encoders reset
                if (resetEncoder)
                {
                    ResetEncoder(motor);
                }

                // set the specified motor run mode
                if (runMode != null)
                {
                    motor.setMode(runMode);

                    if (runMode == DcMotorController.RunMode.RUN_TO_POSITION)
                    {
                        SetMotorTargetPosition(motor, initialTargetPosition);
                    }
                }

                // set the initial power
                SetMotorPower(motor, InitialPower);
            }
        }
        catch (Exception ex)
        {
            // log this
            DbgLog.msg("RobotUtil:SetMotorRunMode() - Exception caught:" + ex.toString());
        }
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    public static double ScaleJoystickInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // clip the right/left values so that the values never exceed +/- 1
        dVal = Range.clip(dVal, -1, 1);

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    public static float GetEncoderCountForDegreeTurn(float degree)
    {
        return degree * 24f;
    }

    public static  float GetEncoderCountForStraightDistance(int inches)
    {
        return inches * 140.5f;
    }
}
