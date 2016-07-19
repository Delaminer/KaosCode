package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class RobotTracks
{
    public RobotTrack LeftTrack = null;
    public RobotTrack RightTrack = null;

    private OpticalDistanceSensor m_opticalDistanceSensor = null;
    private Robot.ROBOT_CONTROL_MODE m_robotControlMode = Robot.ROBOT_CONTROL_MODE.Unknown;
    private boolean m_enableIntelligentBraking = false;
    private float m_lastDrivePowerSetLeft = 0.0f;
    private float m_lastDrivePowerSetRight = 0.0f;
    private final float MIN_LIGHT_DETECTED = 0.8f; // minimum light required to "see" something coming
    private boolean m_enabled = false;
    private boolean m_teleOpPowerReduced = false;

    public RobotTracks(DcMotor leftFrontMotor, DcMotor leftRearMotor,
        DcMotor rightFrontMotor, DcMotor rightRearMotor,
        TouchSensor leftFrontTouchSensor, TouchSensor rightFrontTouchSensor,
        OpticalDistanceSensor opticalDistanceSensor,
        Servo leftTrackTouchSensorServo, Servo rightTrackTouchSensorServo)
    {
        // get the left and right tracks configured
        try
        {
            LeftTrack = new RobotTrack(leftFrontMotor, leftRearMotor, leftFrontTouchSensor,
                leftTrackTouchSensorServo, RobotTrack.ROBOT_TRACK_SIDE.Left);
        }
        catch (Exception ex)
        {
            // unable to set up the left and right tracks
            LeftTrack = new RobotTrack();
            DbgLog.msg("RobotTracks:RobotTracks1() - Left Track, Exception: " + ex.toString());
        }
        try
        {
            RightTrack = new RobotTrack(rightFrontMotor, rightRearMotor, rightFrontTouchSensor,
                rightTrackTouchSensorServo, RobotTrack.ROBOT_TRACK_SIDE.Right);
        }
        catch (Exception ex)
        {
            // unable to set up the left and right tracks
            RightTrack = new RobotTrack();
            DbgLog.msg("RobotTracks:RobotTracks1() - Right Track, Exception: " + ex.toString());
        }

        // save the references to the other required objects
        try
        {
            m_opticalDistanceSensor = opticalDistanceSensor;
            m_opticalDistanceSensor.enableLed(true);
        }
        catch (Exception ex)
        {
            // unable to save & configure other references
            DbgLog.msg("RobotTracks:RobotTracks2() - Exception: " + ex.toString());
        }
    }

    public void EnableIntelligentBreaking(boolean enabled)
    {
        // enable to disable intelligent breaking
        m_enableIntelligentBraking = enabled;
        if (LeftTrack != null)
        {
            LeftTrack.EnableIntelligentBreaking(enabled);
        }
        if (RightTrack != null)
        {
            RightTrack.EnableIntelligentBreaking(enabled);
        }
    }

    public void Enabled(boolean enabled)
    {
        m_enabled = enabled;
        if (LeftTrack != null)
        {
            LeftTrack.Enabled(enabled);
        }
        if (RightTrack != null)
        {
            RightTrack.Enabled(enabled);
        }
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
            DbgLog.msg("RobotTracks:Start() - Exception: " + ex.toString());
        }
    }

    public void SetRobotControlMode(Robot.ROBOT_CONTROL_MODE robotControlMode)
    {
        m_robotControlMode = robotControlMode;
        if (LeftTrack != null)
        {
            LeftTrack.SetRobotControlMode(robotControlMode);
        }
        if (RightTrack != null)
        {
            RightTrack.SetRobotControlMode(robotControlMode);
        }
    }

    public void Loop(float gamepad1LeftStickScaledValue, float gamepad1RightStickScaledValue,
        boolean gamepad2Y)
    {
        // toggle intelligent breaking?
        /*if ((gamepad1LeftBumper) && (!gamepad1RightBumper))
        {
            m_enableIntelligentBraking = true;
            EnableIntelligentBreaking(m_enableIntelligentBraking);
        }
        else if ((!gamepad1LeftBumper) && (gamepad1RightBumper))
        {
            m_enableIntelligentBraking = false;
            EnableIntelligentBreaking(m_enableIntelligentBraking);
        }
*/      m_enableIntelligentBraking = false;
        try
        {
            if ((m_enabled) && (LeftTrack != null) && (RightTrack != null))
            {
                if (m_robotControlMode == Robot.ROBOT_CONTROL_MODE.Autonomous)
                {
                    if ((m_enableIntelligentBraking) && (m_opticalDistanceSensor != null) &&
                        (m_opticalDistanceSensor.getLightDetected() > MIN_LIGHT_DETECTED))
                    {
                        // need to de-grade the breaking is we are close to something
                        // (set to half power for now) TODO - really need an intelligent degrading algorithum here
                        LeftTrack.SetPower(m_lastDrivePowerSetLeft / 2);
                        RightTrack.SetPower(m_lastDrivePowerSetRight / 2);
                    }
                    else if ((m_enableIntelligentBraking) && (m_opticalDistanceSensor != null) &&
                        (m_opticalDistanceSensor.getLightDetected() < MIN_LIGHT_DETECTED))
                    {
                        // set back to originally specified 'full' power that the user set
                        LeftTrack.SetPower(m_lastDrivePowerSetLeft);
                        RightTrack.SetPower(m_lastDrivePowerSetRight);
                    }
                }
                else if (m_robotControlMode == Robot.ROBOT_CONTROL_MODE.TeleOp)
                {
                    // has the power been reduced?  toggle on/off
                    if (gamepad2Y)
                    {
                        m_teleOpPowerReduced = !m_teleOpPowerReduced;
                    }

                    // see if we are in reduced power mode for climbing
                    if (m_teleOpPowerReduced)
                    {
                        LeftTrack.Loop(gamepad1LeftStickScaledValue / 3);
                        RightTrack.Loop(gamepad1RightStickScaledValue / 3);
                    }
                    else
                    {
                        LeftTrack.Loop(gamepad1LeftStickScaledValue);
                        RightTrack.Loop(gamepad1RightStickScaledValue);
                    }
                }
            }
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotTracks:Loop() - Exception: " + ex.toString());
        }
    }

    public void Init()
    {
        // call init on each track
        try
        {
            // enable the tracks by default
            Enabled(true);

            SetDirectionForward();
            if (LeftTrack != null)
            {
                LeftTrack.Init();
            }
            if (RightTrack != null)
            {
                RightTrack.Init();
            }

            // set the power to its default level
            SetPower(0, true);
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotTracks:Init() - Exception: " + ex.toString());
        }
    }

    public void Stop()
    {
        // perform any stop methods
        try
        {
            // stop both tracks
            if (LeftTrack != null)
            {
                LeftTrack.StopTrack();
            }
            if (RightTrack != null)
            {
                RightTrack.StopTrack();
            }
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotTracks:Stop() - Exception: " + ex.toString());
        }
    }

    public void SetRunMode(DcMotorController.RunMode runMode,
        boolean resetEncoder, int initialTargetPosition, float initialPower)
    {
        if (m_enabled)
        {
            if (LeftTrack != null)
            {
                LeftTrack.SetRunMode(runMode, resetEncoder, initialTargetPosition, initialPower);
            }
            if (RightTrack != null)
            {
                RightTrack.SetRunMode(runMode, resetEncoder, initialTargetPosition, initialPower);
            }

            // remember the last power set for resuming later
            m_lastDrivePowerSetLeft = initialPower;
            m_lastDrivePowerSetRight = initialPower;
        }
    }

    private void SetPower(float power, boolean resetLastDrivePowerSet)
    {
        // set the same power to both motors
        if (m_enabled)
        {
            if (LeftTrack != null)
            {
                LeftTrack.SetPower(power);
            }
            if (RightTrack != null)
            {
                RightTrack.SetPower(power);
            }

            // remember the last power set for resuming later
            if (resetLastDrivePowerSet)
            {
                m_lastDrivePowerSetLeft = power;
                m_lastDrivePowerSetRight = power;
            }
        }
    }

    public void SetPower(float power)
    {
        SetPower(power, true);
    }

    public void ResetEcoders()
    {
        if (LeftTrack != null)
        {
            LeftTrack.ResetEncoder();
        }
        if (RightTrack != null)
        {
            RightTrack.ResetEncoder();
        }
    }

    public void SetPower (float leftPower, float rightPower)
    {
        SetPower(leftPower, rightPower, true);
    }

    private void SetPower (float leftPower, float rightPower, boolean resetLastDrivePowerSet)
    {
        if (m_enabled)
        {
            if (LeftTrack != null)
            {
                LeftTrack.SetPower(leftPower);
            }
            if (RightTrack != null)
            {
                RightTrack.SetPower(rightPower);
            }

            // remember the last power set for resuming later
            if (resetLastDrivePowerSet)
            {
                m_lastDrivePowerSetLeft = leftPower;
                m_lastDrivePowerSetRight = rightPower;
            }
        }
    }

    public void SetDirectionForward()
    {
        if (LeftTrack != null)
        {
            LeftTrack.SetDirectionForward();
        }
        if (RightTrack != null)
        {
            RightTrack.SetDirectionForward();
        }
    }

    public void SetDirectionReverse()
    {
        if (LeftTrack != null)
        {
            LeftTrack.SetDirectionReverse();
        }
        if (RightTrack != null)
        {
            RightTrack.SetDirectionReverse();
        }
    }
}
