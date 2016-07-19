package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotWing
{
    private Servo m_servo = null;
    private boolean m_enabled = false;
    private final double LEFT_WING_UP_POSITION = .16; //TODO need to determine the correct values for these positions
    private final double LEFT_WING_DOWN_POSITION = .81; //TODO need to determine the correct values for these positions
    private final double RIGHT_WING_UP_POSITION = .79; //TODO need to determine the correct values for these positions
    private final double RIGHT_WING_DOWN_POSITION = .15; //TODO need to determine the correct values for these positions
    private Robot.ROBOT_CONTROL_MODE m_robotControlMode = Robot.ROBOT_CONTROL_MODE.Unknown;
    private Robot.ROBOT_WING_SIDE m_wingSide = Robot.ROBOT_WING_SIDE.Unknown;


    public RobotWing()
    {
        // create a default instructor for testing and when errors occur
    }

    public RobotWing(Servo servo, Robot.ROBOT_WING_SIDE wingSide)
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
            WingUp();
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
            WingUp();
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
            WingUp();
        }
        m_enabled = enabled;
    }

    public void WingUp()
    {
        if ((m_enabled) && (m_servo != null))
        {
            if (m_wingSide == Robot.ROBOT_WING_SIDE.Left)
            {
                RobotUtil.SetServoPosition(m_servo, LEFT_WING_UP_POSITION);
            }
            else
            {
                RobotUtil.SetServoPosition(m_servo, RIGHT_WING_UP_POSITION);
            }
        }
    }

    public void WingDown()
    {
        if ((m_enabled) && (m_servo != null))
        {
            if (m_wingSide == Robot.ROBOT_WING_SIDE.Left)
            {
                RobotUtil.SetServoPosition(m_servo, LEFT_WING_DOWN_POSITION);
            }
            else
            {
                RobotUtil.SetServoPosition(m_servo, RIGHT_WING_DOWN_POSITION);
            }
        }
    }
}
