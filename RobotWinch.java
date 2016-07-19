package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotWinch
{
    private DcMotor m_winchMotor = null;
    private DcMotor m_tapeMotor = null;
    private Servo m_lockServo = null;
    private Servo m_dumpServo = null;
    private Robot.ROBOT_CONTROL_MODE m_robotControlMode = Robot.ROBOT_CONTROL_MODE.Unknown;
    private final float LOCK_SERVO_LOCKED_POSITION = .45f;
    private final float LOCK_SERVO_UNLOCKED_POSITION = .65f;
    private final float DUMP_SERVO_UP_POSITION = .0f;
    private final float DUMP_SERVO_DOWN_POSITION = .99f;
    private boolean m_enabled = false;
    private boolean m_armLocked = true;
    private boolean m_dumpUp = false;

    public RobotWinch()
    {

    }

    public RobotWinch(DcMotor winchMotor, DcMotor tapeMotor, Servo lockServo, Servo deckServo)
    {
        try
        {
            m_winchMotor = winchMotor;
            m_tapeMotor = tapeMotor;
            m_lockServo = lockServo;
            m_dumpServo = deckServo;
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotWinch:RobotWinch() - Exception: " + ex.toString());
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
            DbgLog.msg("RobotWinch:Start() - Exception: " + ex.toString());
        }
    }

    public void SetRobotControlMode(Robot.ROBOT_CONTROL_MODE robotControlMode)
    {
        m_robotControlMode = robotControlMode;
    }

    public void Loop(float gamepad2LeftStickScaledValue, float gamepad2RightStickScaledValue,
                     boolean gamepad2DpadUp, boolean gamepad2DpadDown)
    {
        // perform any loop methods
        try
        {
            if (m_robotControlMode == Robot.ROBOT_CONTROL_MODE.TeleOp)
            {
                if ((m_enabled) && (m_winchMotor != null))
                {
                    // if there is no power being set, we should "auto-lock" the arm
                    if (gamepad2RightStickScaledValue == 0.0f)
                    {
                        WinchLocked(true);
                    }
                    else
                    {
                        WinchLocked(false);
                    }

                    RobotUtil.SetMotorPower(m_winchMotor, gamepad2RightStickScaledValue);
                }

                if ((m_enabled) && (m_tapeMotor != null))
                {
                    RobotUtil.SetMotorPower(m_tapeMotor, gamepad2LeftStickScaledValue);

                }

                // raise or lower the dumper
                if ((m_enabled) && (m_dumpServo != null))
                {
                    if ((gamepad2DpadUp) && (!gamepad2DpadDown) && (!m_dumpUp))
                    {
                        DumpUp(true);
                    }
                    else if ((!gamepad2DpadUp) && (gamepad2DpadDown) && (m_dumpUp))
                    {
                        DumpUp(false);
                    }
                }
            }
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotWinch:Loop() - Exception: " + ex.toString());
        }
    }

    public void Init()
    {
        // perform any init methods
        try
        {
            // set the default run modes for these motors
            RobotUtil.SetMotorRunMode(m_winchMotor, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS,
                DcMotor.Direction.REVERSE, true, 0, 0.0f);
            RobotUtil.SetMotorRunMode(m_tapeMotor, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS,
                    DcMotor.Direction.REVERSE, true, 0, 0.0f);
            //RobotUtil.SetServoDirection(m_dumpServo, Servo.Direction.REVERSE);

            // enable the arm by default
            Enabled(true);

            // set the arm in the locked position to start with
            WinchLocked(true);

            // set the deck down by default
            DumpUp(false);
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotWinch:Init() - Exception: " + ex.toString());
        }
    }

    public void Stop()
    {
        // perform any stop methods
        try
        {
            StopTape();
            StopWinch();
        }
        catch (Exception ex)
        {
            DbgLog.msg("RobotWinch:Stop() - Exception: " + ex.toString());
        }
    }

    public void Enabled(boolean enabled)
    {
        m_enabled = enabled;
    }

    public void WinchLocked(boolean locked)
    {
        m_armLocked = locked;
        if ((m_enabled) && (m_lockServo != null))
        {
            if (m_armLocked)
            {
                // set the servo to the locked position
                RobotUtil.SetServoPosition(m_lockServo, LOCK_SERVO_UNLOCKED_POSITION);
            }
            else
            {
                // set the servo to the unlocked position
                RobotUtil.SetServoPosition(m_lockServo, LOCK_SERVO_LOCKED_POSITION);
            }
        }
    }

    public void DumpUp(boolean up)
    {
        m_dumpUp = up;
        if ((m_enabled) && (m_dumpServo != null))
        {
            if (m_dumpUp)
            {
                // set the servo to the up position
                RobotUtil.SetServoPosition(m_dumpServo, DUMP_SERVO_UP_POSITION);
            }
            else
            {
                // set the servo to the down position
                RobotUtil.SetServoPosition(m_dumpServo, DUMP_SERVO_DOWN_POSITION);
            }
        }
    }

    public void TapeOut()
    {
        if ((m_enabled) && (!m_armLocked) && (m_tapeMotor != null))
        {
            //TODO - really need a sensor for this if we need this mode!

        }
    }

    private void SetTapePower(float power)
    {
        // set arm power
        if ((m_enabled) && (m_tapeMotor != null))
        {
            RobotUtil.SetMotorPower(m_tapeMotor, power);
        }
    }

    private void SetWinchPower(float power)
    {
        // set arm power
        if ((m_enabled) && (m_winchMotor != null))
        {
            if (power == 0.0f)
            {
                WinchLocked(true);
            }
            else
            {
                WinchLocked(false);
            }

            RobotUtil.SetMotorPower(m_winchMotor, power);
        }
    }

    public void StopTape()
    {
        // just set the power to zero
        SetTapePower(0);
    }

    public void StopWinch()
    {
        // just set the power to zero
        SetWinchPower(0);
    }

    public float GetTapeCurrentPosition()
    {
        // return the current position of the arm
        return RobotUtil.GetMotorCurrentPosition(m_tapeMotor);
    }

    public float GetWinchCurrentPosition()
    {
        // return the current position of the arm
        return RobotUtil.GetMotorCurrentPosition(m_winchMotor);
    }

    public float GetTapeTargetPosition()
    {
        // return the target position of the arm
        return RobotUtil.GetMotorTargetPosition(m_tapeMotor);
    }

    public float GetWinchTargetPosition()
    {
        // return the target position of the arm
        return RobotUtil.GetMotorTargetPosition(m_winchMotor);
    }

    public void SetTapeTargetPosition(int position)
    {
        // set the target position of the arm
        RobotUtil.SetMotorTargetPosition(m_tapeMotor, position);
    }

    public void SetWinchTargetPosition(int position)
    {
        // set the target position of the arm
        RobotUtil.SetMotorTargetPosition(m_winchMotor, position);
    }

    public void SetTapeDirectionUp()
    {
        // set the direction of arm motor to go up
        RobotUtil.SetMotorDirection(m_tapeMotor, DcMotor.Direction.FORWARD);
    }

    public void SetWinchDirectionOut()
    {
        // set the direction of arm motor to go up
        RobotUtil.SetMotorDirection(m_winchMotor, DcMotor.Direction.FORWARD);
    }

    public void SetTapeDirectionDown()
    {
        // set the direction of arm motor to go down
        RobotUtil.SetMotorDirection(m_tapeMotor, DcMotor.Direction.REVERSE);
    }

    public void SetWinchDirectionIn()
    {
        // set the direction of arm motor to go down
        RobotUtil.SetMotorDirection(m_winchMotor, DcMotor.Direction.REVERSE);
    }

    public void ResetTapeEncoder()
    {
        // reset the front motor encoder
        RobotUtil.ResetEncoder(m_tapeMotor);
    }

    public void ResetWinchEncoder()
    {
        // reset the front motor encoder
        RobotUtil.ResetEncoder(m_winchMotor);
    }

    public void SetTapeRunMode(DcMotorController.RunMode runMode,
                               boolean resetEncoder, int initialTargetPosition, float initialPower)
    {
        // set the motors run mode
        if ((m_enabled) && (m_tapeMotor != null))
        {
            RobotUtil.SetMotorRunMode(m_tapeMotor, runMode, resetEncoder,
                initialTargetPosition, initialPower);
        }
    }

    public void SetWinchRunMode(DcMotorController.RunMode runMode,
        boolean resetEncoder, int initialTargetPosition, float initialPower)
    {
        // set the motors run mode
        if ((m_enabled) && (m_winchMotor != null))
        {
            RobotUtil.SetMotorRunMode(m_winchMotor, runMode, resetEncoder,
                initialTargetPosition, 0);

            // need to set power this way so lock is enabled/disabled
            SetWinchPower(initialPower);
        }
    }

    public boolean HasTapeRunToPosition()
    {
        // determine if the track has run to its target position
        if (GetTapeCurrentPosition() >= GetTapeTargetPosition())
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

    public boolean HasTapeRunToPosition(float targetPosition)
    {
        // determine if the track has run to its target position
        if (GetTapeCurrentPosition() >= Math.abs(targetPosition))
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

    public boolean HasWinchRunToPosition()
    {
        // determine if the track has run to its target position
        if (GetWinchCurrentPosition() >= GetWinchTargetPosition())
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

    public boolean HasWinchRunToPosition(float targetPosition)
    {
        // determine if the track has run to its target position
        if (GetWinchCurrentPosition() >= Math.abs(targetPosition))
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
