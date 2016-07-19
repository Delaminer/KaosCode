package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class RobotTrack
{
    private DcMotor m_frontMotor = null;
    private DcMotor m_rearMotor = null;
    private Servo m_trackTouchSensorServo = null;
    private TouchSensor m_frontTouchSensor = null;
    private boolean m_enableIntelligentBreaking = false;
    private Robot.ROBOT_CONTROL_MODE m_robotControlMode = Robot.ROBOT_CONTROL_MODE.Unknown;
    private float m_lastDrivePowerSet = 0.0f;
    private boolean m_isForcedStopped = false;
    private boolean m_enabled = false;
    private ROBOT_TRACK_SIDE m_trackSide = ROBOT_TRACK_SIDE.Unknown;
    private final double TOUCH_SENSOR_LEFT_ARM_OUT_POSITION = .20; //TODO need to determine the correct values for these positions
    private final double TOUCH_SENSOR_LEFT_ARM_IN_POSITION = .90; //TODO need to determine the correct values for these positions
    private final double TOUCH_SENSOR_RIGHT_ARM_OUT_POSITION = .20; //TODO need to determine the correct values for these positions
    private final double TOUCH_SENSOR_RIGHT_ARM_IN_POSITION = .90; //TODO need to determine the correct values for these positions

    public enum ROBOT_TRACK_SIDE
    {
        Unknown,
        Right,
        Left;
    }

    public RobotTrack()
    {
        // specify a default constructor for testing
    }

    public RobotTrack(DcMotor frontMotor, DcMotor rearMotor, TouchSensor frontTouchSensor,
        Servo trackTouchSensorServo, ROBOT_TRACK_SIDE trackSide)
    {
        try
        {
            m_frontMotor = frontMotor;
            m_rearMotor = rearMotor;
            m_frontTouchSensor = frontTouchSensor;
            m_trackSide = trackSide;
            m_trackTouchSensorServo = trackTouchSensorServo;
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotTrack:RobotTrack() - Exception: " + ex.toString());
        }
    }

    public boolean IsForcedStopped()
    {
        // return if the button was pressed to stop the track
        if ((m_enableIntelligentBreaking) && (m_frontTouchSensor != null))
        {
            // the track can only be forced stopped when this feature is enabled
            return m_isForcedStopped;
        }
        else
        {
            // the track can only be forced stopped when this feature is enabled
            return false;
        }
    }

    public void Enabled(boolean enabled)
    {
        m_enabled = enabled;
        if (!m_enabled)
        {
            // if the robot is not enabled, stop it now
            SetPower(0, false);
        }
    }

    public void Start()
    {
        // perform any start methods
        try
        {

        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotTrack:Start() - Exception: " + ex.toString());
        }
    }

    public void SetRobotControlMode(Robot.ROBOT_CONTROL_MODE robotControlMode)
    {
        m_robotControlMode = robotControlMode;
    }

    public void Loop(float gamepadStickScaledValue)
    {
        // check sensors to see if we need to apply breaks or to stop
        try
        {
            SetPower(gamepadStickScaledValue, true);

            if ((m_enableIntelligentBreaking) && (m_frontTouchSensor != null) &&
                (m_frontTouchSensor.isPressed()))
            {
                // temporarily stop the track
                m_isForcedStopped = true;
                SetPower(0.0f, false);
            }
            else if ((m_enableIntelligentBreaking) && (m_frontTouchSensor != null) &&
                (!m_frontTouchSensor.isPressed()))
            {
                // set the track back to the last power
                m_isForcedStopped = false;
                SetPower(m_lastDrivePowerSet, false);
            }
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotTrack:Loop() - Exception: " + ex.toString());
        }
    }

    public void Init()
    {
        // perform any init methods
        try
        {
            SetRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS, true, 0, 0.0f);
            Enabled(true);
            EnableIntelligentBreaking(false);
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotTrack:Init() - Exception: " + ex.toString());
        }
    }

    public void Stop()
    {
        // perform any stop methods
        try
        {
            EnableIntelligentBreaking(false);
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotTrack:Stop() - Exception: " + ex.toString());
        }
    }

    public void EnableIntelligentBreaking(boolean enabled)
    {
        // enable the intelligent breaking module and make sure the sensors are
        // extended when this feature is in use
        m_enableIntelligentBreaking = enabled;
        if ((enabled) && (m_trackTouchSensorServo != null))
        {
            if (m_trackSide == ROBOT_TRACK_SIDE.Left)
            {
                RobotUtil.SetServoDirection(m_trackTouchSensorServo, Servo.Direction.FORWARD);
                RobotUtil.SetServoPosition(m_trackTouchSensorServo, TOUCH_SENSOR_LEFT_ARM_OUT_POSITION);
            }
            else
            {
                RobotUtil.SetServoDirection(m_trackTouchSensorServo, Servo.Direction.REVERSE);
                RobotUtil.SetServoPosition(m_trackTouchSensorServo, TOUCH_SENSOR_RIGHT_ARM_OUT_POSITION);
            }
        }
        else if ((!enabled) && (m_trackTouchSensorServo != null))
        {
            if (m_trackSide == ROBOT_TRACK_SIDE.Left)
            {
                RobotUtil.SetServoDirection(m_trackTouchSensorServo, Servo.Direction.FORWARD);
                RobotUtil.SetServoPosition(m_trackTouchSensorServo, TOUCH_SENSOR_LEFT_ARM_IN_POSITION);
            }
            else
            {
                RobotUtil.SetServoDirection(m_trackTouchSensorServo, Servo.Direction.REVERSE);
                RobotUtil.SetServoPosition(m_trackTouchSensorServo, TOUCH_SENSOR_RIGHT_ARM_IN_POSITION);
            }
        }

        // clear forced stop
        if (!enabled)
        {
            m_isForcedStopped = false;
        }
    }

    private void SetPower(float power, boolean resetLastDrivePowerSet)
    {
        // set the same power to both motors
        if ((m_enabled) && (m_frontMotor != null) && (m_rearMotor != null))
        {
            RobotUtil.SetMotorPower(m_frontMotor, power);
            RobotUtil.SetMotorPower(m_rearMotor, power);

            // remember the last power set for resuming later
            if (resetLastDrivePowerSet)
            {
                m_lastDrivePowerSet = power;
            }
        }
    }

    public void SetPower(float power)
    {
        SetPower(power, true);
    }

    public void StopTrack()
    {
        // just set the power to zero
        SetPower(0);
    }

    public float GetCurrentPosition()
    {
        // just return the value of the front motor
        return RobotUtil.GetMotorCurrentPosition(m_frontMotor);
    }

    public float GetTargetPosition()
    {
        // just return the value of the front motor
        return RobotUtil.GetMotorTargetPosition(m_frontMotor);
    }

    public void SetTagetPosition(int position)
    {
        // set the same value to both motors
        RobotUtil.SetMotorTargetPosition(m_frontMotor, position);
    }

    public void SetDirectionForward()
    {
        // set the direction of the tracks to opposite directions to go forward
        if (m_trackSide == ROBOT_TRACK_SIDE.Left)
        {
            RobotUtil.SetMotorDirection(m_frontMotor, DcMotor.Direction.FORWARD);
            RobotUtil.SetMotorDirection(m_rearMotor, DcMotor.Direction.FORWARD);
        }
        else
        {
            RobotUtil.SetMotorDirection(m_frontMotor, DcMotor.Direction.REVERSE);
            RobotUtil.SetMotorDirection(m_rearMotor, DcMotor.Direction.REVERSE);
        }
    }

    public void SetDirectionReverse()
    {
        // set the direction of the tracks to opposite directions to go Reverse
        if (m_trackSide == ROBOT_TRACK_SIDE.Left)
        {
            RobotUtil.SetMotorDirection(m_frontMotor, DcMotor.Direction.REVERSE);
            RobotUtil.SetMotorDirection(m_rearMotor, DcMotor.Direction.REVERSE);
        }
        else
        {
            RobotUtil.SetMotorDirection(m_frontMotor, DcMotor.Direction.FORWARD);
            RobotUtil.SetMotorDirection(m_rearMotor, DcMotor.Direction.FORWARD);
        }
    }

    public void ResetEncoder()
    {
        // reset the front motor encoder
        RobotUtil.ResetEncoder(m_frontMotor);
    }

    public void SetRunMode(DcMotorController.RunMode runMode,
        boolean resetEncoder, int initialTargetPosition, float initialPower)
    {
        // set the motors run mode
        if ((m_enabled) && (m_frontMotor != null) && (m_rearMotor != null))
        {
            RobotUtil.SetMotorRunMode(m_frontMotor, runMode, resetEncoder,
                initialTargetPosition, initialPower);
            RobotUtil.SetMotorRunMode(m_rearMotor, runMode, resetEncoder,
                initialTargetPosition, initialPower);
        }
    }

    public boolean HasRunToPosition()
    {
        // determine if the track has run to its target position
        if (GetCurrentPosition() >= GetTargetPosition())
        {
            // the current position is equal to or greater to the target position
            return true;
        }
        else
        {
            // the current position does not exceed the target position
            return false;
        }
    }

    public boolean HasRunToPosition(float targetPosition)
    {
        // determine if the track has run to its target position
        if (GetCurrentPosition() >= Math.abs(targetPosition))
        {
            // the current position is equal to or greater to the target position
            return true;
        }
        else
        {
            // the current position does not exceed the target position
            return false;
        }
    }

    public static boolean HasRunToPosition(float targetPosition, float currentPosition)
    {
        // determine if the track has run to its target position
        if (Math.abs(currentPosition) >= Math.abs(targetPosition))
        {
            // the current position is equal to or greater to the target position
            return true;
        }
        else
        {
            // the current position does not exceed the target position
            return false;
        }
    }
}
