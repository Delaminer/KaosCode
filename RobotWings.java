package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotWings
{
    public RobotWing LeftWing = null;
    public RobotWing RightWing = null;
    public RobotLowWing LeftLowWing = null;
    public RobotLowWing RightLowWing = null;
    private boolean m_enabled = false;
    private Robot.ROBOT_TEAM_COLOR m_teamColor = Robot.ROBOT_TEAM_COLOR.Unknown;
    private Robot.ROBOT_CONTROL_MODE m_robotControlMode = Robot.ROBOT_CONTROL_MODE.Unknown;

    public RobotWings()
    {
        // create a default constructor for testing when errors occur
    }

    public RobotWings(Servo leftArmServo, Servo rightArmServo, Servo leftLowArmServo, Servo rightLowArmServo)
    {
        // set the arm servo
        try
        {
            LeftWing = new RobotWing(leftArmServo, Robot.ROBOT_WING_SIDE.Left);
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotWings:RobotWings() - LeftWing, Exception: " + ex.toString());
        }
        try
        {
            RightWing = new RobotWing(rightArmServo, Robot.ROBOT_WING_SIDE.Right);
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotWings:RobotWings() - RightWing, Exception: " + ex.toString());
        }
        try
        {
            LeftLowWing = new RobotLowWing(leftLowArmServo, Robot.ROBOT_WING_SIDE.Left);
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotWings:RobotWings() - LeftLowWing, Exception: " + ex.toString());
        }
        try
        {
            RightLowWing = new RobotLowWing(rightLowArmServo, Robot.ROBOT_WING_SIDE.Right);
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotWings:RobotWings() - RightWing, Exception: " + ex.toString());
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
            DbgLog.msg("RobotWings:Start() - Exception: " + ex.toString());
        }
    }


    public void SetTeamColor(Robot.ROBOT_TEAM_COLOR teamColor)
    {
        m_teamColor = teamColor;
    }

    public void SetRobotControlMode(Robot.ROBOT_CONTROL_MODE robotControlMode)
    {
        m_robotControlMode = robotControlMode;
        if (LeftWing != null)
        {
            LeftWing.SetRobotControlMode(robotControlMode);
        }
        if (RightWing != null)
        {
            RightWing.SetRobotControlMode(robotControlMode);
        }
        if (LeftLowWing != null)
        {
            LeftLowWing.SetRobotControlMode(robotControlMode);
        }
        if (RightLowWing != null)
        {
            RightLowWing.SetRobotControlMode(robotControlMode);
        }
    }

    public void Loop(
        boolean gamepad2LeftBumper, float gamepad2LeftTrigger,
        boolean gamepad2RightBumper, float gamepad2RightTrigger,
        boolean gamepad1LeftBumper, float gamepad1LeftTrigger,
        boolean gamepad1RightBumper, float gamepad1RightTrigger)
    {
        // handle the left arm
        try
        {
            if (m_robotControlMode == Robot.ROBOT_CONTROL_MODE.TeleOp)
            {
                if ((m_enabled) && (LeftWing != null) && (gamepad2LeftBumper))
                {
                    LeftWing.WingUp();
                }
                else if ((m_enabled) && (LeftWing != null) && (gamepad2LeftTrigger > 0))
                {
                    LeftWing.WingDown();
                }

                // handle the right arm
                if ((m_enabled) && (RightWing != null) && (gamepad2RightBumper))
                {
                    RightWing.WingUp();
                }
                else if ((m_enabled) && (RightWing != null) && (gamepad2RightTrigger > 0))
                {
                    RightWing.WingDown();
                }

                // handle left lower arm
                if ((m_enabled) && (LeftLowWing != null) && (gamepad1LeftBumper))
                {
                    LeftLowWing.WingIn();
                }
                else if ((m_enabled) && (LeftLowWing != null) && (gamepad1LeftTrigger > 0))
                {
                    LeftLowWing.WingOut();
                }

                // handle the right lower arm
                if ((m_enabled) && (RightLowWing != null) && (gamepad1RightBumper))
                {
                    RightLowWing.WingIn();
                }
                else if ((m_enabled) && (RightLowWing != null) && (gamepad1RightTrigger > 0))
                {
                    RightLowWing.WingOut();
                }

                // call loop on each wing
                if (LeftWing != null)
                {
                    LeftWing.Loop();
                }
                if (RightWing != null)
                {
                    RightWing.Loop();
                }
                if (LeftLowWing != null)
                {
                    LeftLowWing.Loop();
                }
                if (RightLowWing != null)
                {
                    RightLowWing.Loop();
                }
            }
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotWings:Loop() - Exception: " + ex.toString());
        }
    }

    public void Init()
    {
        // call init on each wing
        try
        {
            // try reversing the left low servo
            RobotUtil.SetServoDirection(LeftLowWing.m_servo, Servo.Direction.REVERSE);

            if (LeftWing != null)
            {
                LeftWing.Init();
            }
            if (RightWing != null)
            {
                RightWing.Init();
            }
            if (LeftLowWing != null)
            {
                LeftLowWing.Init();
            }
            if (RightLowWing != null)
            {
                RightLowWing.Init();
            }

            // set the defaults
            Enabled(true);
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotWings:Init() - Exception: " + ex.toString());
        }
    }

    public void Stop()
    {
        // perform any stop methods
        try
        {
            // raise the wings
            WingsUp();
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotWings:Stop() - Exception: " + ex.toString());
        }
    }

    public void Enabled(boolean enabled)
    {
        m_enabled = enabled;

        // if the wings are disabled, this will also pull them up
        if (LeftWing != null)
        {
            if ((m_robotControlMode == Robot.ROBOT_CONTROL_MODE.TeleOp) && (m_teamColor == Robot.ROBOT_TEAM_COLOR.Blue))
            {
                LeftWing.Enabled(enabled);
            }
            else if ((m_robotControlMode == Robot.ROBOT_CONTROL_MODE.TeleOp) && (m_teamColor == Robot.ROBOT_TEAM_COLOR.Red))
            {
                LeftWing.Enabled(false);
            } else
            {
                LeftWing.Enabled(enabled);
            }
        }
        if (RightWing != null)
        {
            if ((m_robotControlMode == Robot.ROBOT_CONTROL_MODE.TeleOp) && (m_teamColor == Robot.ROBOT_TEAM_COLOR.Blue))
            {
                RightWing.Enabled(false);
            }
            else if ((m_robotControlMode == Robot.ROBOT_CONTROL_MODE.TeleOp) && (m_teamColor == Robot.ROBOT_TEAM_COLOR.Red))
            {
                RightWing.Enabled(enabled);
            } else
            {
                RightWing.Enabled(enabled);
            }
        }
        if (LeftLowWing != null)
        {
            if ((m_robotControlMode == Robot.ROBOT_CONTROL_MODE.TeleOp) && (m_teamColor == Robot.ROBOT_TEAM_COLOR.Blue))
            {
                LeftLowWing.Enabled(enabled);
            }
            else if ((m_robotControlMode == Robot.ROBOT_CONTROL_MODE.TeleOp) && (m_teamColor == Robot.ROBOT_TEAM_COLOR.Red))
            {
                LeftLowWing.Enabled(false);
            } else
            {
                LeftLowWing.Enabled(enabled);
            }
        }
        if (RightLowWing != null)
        {
            if ((m_robotControlMode == Robot.ROBOT_CONTROL_MODE.TeleOp) && (m_teamColor == Robot.ROBOT_TEAM_COLOR.Blue))
            {
                RightLowWing.Enabled(false);
            }
            else if ((m_robotControlMode == Robot.ROBOT_CONTROL_MODE.TeleOp) && (m_teamColor == Robot.ROBOT_TEAM_COLOR.Red))
            {
                RightLowWing.Enabled(enabled);
            }
            else
            {
                RightLowWing.Enabled(enabled);
            }
        }
    }

    public void WingsUp()
    {
        if ((m_enabled) && (LeftWing != null))
        {
            LeftWing.WingUp();
        }
        if ((m_enabled) && (RightWing != null))
        {
            RightWing.WingUp();
        }
        if ((m_enabled) && (LeftLowWing != null))
        {
            LeftLowWing.WingIn();
        }
        if ((m_enabled) && (RightLowWing != null))
        {
            RightLowWing.WingIn();
        }
    }

    public void WingsDown()
    {
        if ((m_enabled) && (LeftWing != null))
        {
            LeftWing.WingDown();
        }
        if ((m_enabled) && (RightWing != null))
        {
            RightWing.WingDown();
        }
        if ((m_enabled) && (LeftLowWing != null))
        {
            LeftLowWing.WingOut();
        }
        if ((m_enabled) && (RightLowWing != null))
        {
            RightLowWing.WingOut();
        }
    }
}
