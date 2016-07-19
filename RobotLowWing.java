package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotLowWing
{
    public Servo m_servo = null;
    private boolean m_enabled = false;
    private final double LEFT_WING_IN_POSITION = .0; //TODO need to determine the correct values for these positions
    private final double LEFT_WING_OUT_POSITION = .65; //TODO need to determine the correct values for these positions
    private final double RIGHT_WING_IN_POSITION = .0; //TODO need to determine the correct values for these positions
    private final double RIGHT_WING_OUT_POSITION = .65; //TODO need to determine the correct values for these positions
    private Robot.ROBOT_CONTROL_MODE m_robotControlMode = Robot.ROBOT_CONTROL_MODE.Unknown;
    private Robot.ROBOT_WING_SIDE m_wingSide = Robot.ROBOT_WING_SIDE.Unknown;


    public RobotLowWing()
    {
        // create a default instructor for testing and when errors occur
    }

    public RobotLowWing(Servo servo, Robot.ROBOT_WING_SIDE wingSide)
    {
        try
        {
            m_servo = servo;
            m_wingSide = wingSide;
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotWing:RobotWing() - Exception: " + ex.toString());
        }
    }

    public void Start()
    {
        // perform any stop methods
        try
        {

        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotWing:Start() - Exception: " + ex.toString());
        }
    }

    public void SetRobotControlMode(Robot.ROBOT_CONTROL_MODE robotControlMode)
    {
        m_robotControlMode = robotControlMode;
    }

    public void Loop()
    {
        // perform any stop methods
        try
        {

        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotWing:Loop() - Exception: " + ex.toString());
        }
    }

    public void Init()
    {
        // perform any init methods
        try
        {
            Enabled(true);
            WingIn();
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotWing:Init() - Exception: " + ex.toString());
        }
    }

    public void Stop()
    {
        // perform any stop methods
        try
        {
            Enabled(true);
            WingIn();
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotWing:Stop() - Exception: " + ex.toString());
        }
    }

    public void Enabled(boolean enabled)
    {

        // if the wing is not enabled, pull it up
        if (m_servo != null)
        {
            m_enabled = true;
            WingIn();
        }
        m_enabled = enabled;
    }

    public void WingIn()
    {
        if ((m_enabled) && (m_servo != null))
        {
            if (m_wingSide == Robot.ROBOT_WING_SIDE.Left)
            {
                RobotUtil.SetServoPosition(m_servo, LEFT_WING_IN_POSITION);
            }
            else
            {
                RobotUtil.SetServoPosition(m_servo, RIGHT_WING_IN_POSITION);
            }
        }
    }

    public void WingOut()
    {
        if ((m_enabled) && (m_servo != null))
        {
            if (m_wingSide == Robot.ROBOT_WING_SIDE.Left)
            {
                RobotUtil.SetServoPosition(m_servo, LEFT_WING_OUT_POSITION);
            }
            else
            {
                RobotUtil.SetServoPosition(m_servo, RIGHT_WING_OUT_POSITION);
            }
        }
    }
}
