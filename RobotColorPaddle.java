package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Color;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotColorPaddle
{
    private Servo m_servo = null;
    public ColorSensor m_colorSensor = null;
    private DeviceInterfaceModule m_deviceInterfaceModule = null;
    private Robot.ROBOT_TEAM_COLOR m_teamColor = Robot.ROBOT_TEAM_COLOR.Unknown;
    private Robot.ROBOT_CONTROL_MODE m_robotControlMode = Robot.ROBOT_CONTROL_MODE.Unknown;
    private boolean m_enabled = false;
    private final double PADDLE_POSITION_LEFT = .65; //TODO will need to be adjusted
    private final double PADDLE_POSITION_CENTER = .45;
    private final double PADDLE_POSITION_RIGHT = .25; //TODO will need to be adjusted
    private final int MIN_RGB_VALUE_BLUE = 500;
    private final int MIN_RGB_VALUE_RED = 500;
    public final int CIM_LED_CHANNEL = 7;

    public RobotColorPaddle()
    {
        // have a default constructor for testing when errors occur
    }

    public RobotColorPaddle(Servo servo, ColorSensor colorSensor, DeviceInterfaceModule deviceInterfaceModule)
    {
        try
        {
            m_servo = servo;
            m_colorSensor = colorSensor;
            m_deviceInterfaceModule = deviceInterfaceModule;
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotColorPaddle:RobotColorPaddle() - Exception: " + ex.toString());
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
            DbgLog.msg("RobotColorPaddle:Start() - Exception: " + ex.toString());
        }
    }

    public void SetRobotControlMode(Robot.ROBOT_CONTROL_MODE robotControlMode)
    {
        m_robotControlMode = robotControlMode;
    }

    public void LEDEnabled(boolean enabled)
    {
        try
        {
            if (m_deviceInterfaceModule != null)
            {
                m_deviceInterfaceModule.setDigitalChannelState(CIM_LED_CHANNEL, enabled);
            }
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotColorPaddle:LEDEnabled() - Exception: " + ex.toString());
        }
    }

    public void Loop(boolean gamepad2DpadLeft, boolean gamepad2DpadRight,
        boolean gamepad2DpadUp, boolean gamepad2DpadDown)
    {
        // should only be called from TeleOp mode for testing purposes only!
        try
        {
            if ((m_enabled) && (m_servo != null) && (m_robotControlMode == Robot.ROBOT_CONTROL_MODE.TeleOp))
            {
                if (gamepad2DpadLeft)
                {
                    RobotUtil.SetServoPosition(m_servo, PADDLE_POSITION_LEFT);
                }
                else if (gamepad2DpadRight)
                {
                    RobotUtil.SetServoPosition(m_servo, PADDLE_POSITION_RIGHT);
                }
                else if (gamepad2DpadUp)
                {
                    RobotUtil.SetServoPosition(m_servo, PADDLE_POSITION_CENTER);
                }
                else if (gamepad2DpadDown)
                {
                    //TODO - execute test - TEMP
                    PushTeamColorButton();
                }
                else
                {
                    // don't do anything yet - will be left in last position
                }
            }
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotColorPaddle:Loop() - Exception: " + ex.toString());
        }
    }

    public void Init()
    {
        // perform any init methods
        try
        {
            Enabled(true);
            SetPaddleCenter();
            LEDEnabled(true);
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotColorPaddle:Init() - Exception: " + ex.toString());
        }
    }

    public void Stop()
    {
        // perform any stop methods
        try
        {
            // center the paddle
            SetPaddleCenter();
            LEDEnabled(false);
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotColorPaddle:Stop() - Exception: " + ex.toString());
        }
    }

    public void Enabled(boolean enabled)
    {
        m_enabled = enabled;

        // if the paddle is disabled, center it
        if ((!m_enabled) && (m_servo != null))
        {
            RobotUtil.SetServoPosition(m_servo, PADDLE_POSITION_CENTER);
        }
    }

    public void SetTeamColor(Robot.ROBOT_TEAM_COLOR teamColor)
    {
        m_teamColor = teamColor;
    }

    public void SetPaddleCenter()
    {
        if ((m_enabled) && (m_servo != null))
        {
            // center the bumper paddle
            RobotUtil.SetServoPosition(m_servo, PADDLE_POSITION_CENTER);
        }
    }

    public void PushTeamColorButton()
    {
        // should only be called from Autonomous mode!
        if ((m_enabled) && (m_servo != null) && (m_colorSensor != null))
        {
            double paddlePosition = GetPaddleBumpTargetPosition();
            RobotUtil.SetServoPosition(m_servo, paddlePosition);
        }
    }

    private double GetPaddleBumpTargetPosition()
    {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        int red = 0;
        int blue = 0;
        int green = 0;
        int clear = 0;
        float hue = 0.0f;

        if (m_colorSensor != null)
        {
            // convert the RGB values to HSV values.
            Color.RGBToHSV((m_colorSensor.red() * 255) / 800, (m_colorSensor.green() * 255) / 800, (m_colorSensor.blue() * 255) / 800, hsvValues);

            // convert the hue to a color
            int color = Color.HSVToColor(0xff, values);

            red = m_colorSensor.red();
            green = m_colorSensor.green();
            blue = m_colorSensor.blue();
            clear = m_colorSensor.alpha();
            hue = hsvValues[0];

            // Assuming the color sensor is only on one side (the RIGHT)
            if ((m_teamColor == Robot.ROBOT_TEAM_COLOR.Blue)  &&
                (blue > MIN_RGB_VALUE_BLUE) && (blue > red))
            {
                return PADDLE_POSITION_RIGHT;
            }
            else if ((m_teamColor == Robot.ROBOT_TEAM_COLOR.Blue)  &&
                (red > MIN_RGB_VALUE_RED) && (red > blue))
            {
                return PADDLE_POSITION_LEFT;
            }
            else if ((m_teamColor == Robot.ROBOT_TEAM_COLOR.Red)  &&
                (red > MIN_RGB_VALUE_RED) && (red > blue))
            {
                return PADDLE_POSITION_RIGHT;
            }
            else if ((m_teamColor == Robot.ROBOT_TEAM_COLOR.Red)  &&
                (blue > MIN_RGB_VALUE_BLUE) && (blue > red))
            {
                return PADDLE_POSITION_LEFT;
            }
            else
            {
                // could not determine the color, keep the paddle in the center
                return PADDLE_POSITION_CENTER;
            }
        }
        else
        {
            // could not determine the color, keep the paddle in the center
            return PADDLE_POSITION_CENTER;
        }
    }
}
