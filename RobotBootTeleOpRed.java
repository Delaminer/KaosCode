package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RobotBootTeleOpRed extends OpMode {

    Robot m_robot = null;

    public RobotBootTeleOpRed()
    {

    }

    @Override
    public void init()
    {
        // initialize the robot object
        m_robot = new Robot(this.hardwareMap, this.telemetry, Robot.ROBOT_CONTROL_MODE.TeleOp,
                Robot.ROBOT_AUTONOMOUS_MODE.Unknown, Robot.ROBOT_TEAM_COLOR.Red);

        // let the robot initialize any other objects
        m_robot.Init();
    }

    @Override
    public void start()
    {
        // call the start method on the robot to get the bot going
        if (m_robot != null)
        {
            m_robot.Start();
        }
    }

    @Override
    public void loop()
    {
        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        float gamepad1RightStick = (float) RobotUtil.ScaleJoystickInput(-gamepad1.right_stick_y);
        float gamepad1LeftStick =  (float) RobotUtil.ScaleJoystickInput(-gamepad1.left_stick_y);
        float gamepad2RightStick = (float) RobotUtil.ScaleJoystickInput(-gamepad2.right_stick_y);
        float gamepad2LeftStick =  (float) RobotUtil.ScaleJoystickInput(-gamepad2.left_stick_y);

        // CONTROLLER MAP (these are the only controls in use right now)
        // gamepad1.right_stick_y: right track forward and backward
        // gamepad1.left_stick_y: left track forward and backward
        // gamepad1.y: sets program to Autonomous mode (TEMP)
        // gamepad1.x: sets program to TeleOp mode (TEMP)
        // gamepad1.start: will also re-read switches
        // gamepad1.left_bumper: left low wing in
        // gamepad1.left_trigger:  left low wing out
        // gamepad1.right_bumper: right low wing in
        // gamepad1.right_trigger: right low wing out
        // gamepad2.right_stick_y: winch motor
        // gamepad2.left_stick_y: tape motor
        // gamepad2.dpad_left: left color paddle bump
        // gamepad2.dpad_right: right color paddle bump
        // gamepad2.dpad_up: center color paddle | dumper up
        // gamepad2.dpad_down: execute color detection auto bump | dumper down
        // gamepad2.y: cuts track power 1/3 in TeleOp (toggle)
        // gamepad2.x:
        // gamepad2.left_bumper: left wing up
        // gamepad2.left_trigger: left wing down
        // gamepad2.right_bumper: right wing up
        // gamepad2.right_trigger: right wing down

        // let the robot controller know it some time to do work
        m_robot.Loop(gamepad1LeftStick, gamepad1RightStick, gamepad2.left_bumper, gamepad2.left_trigger,
                gamepad2.right_bumper, gamepad2.right_trigger, gamepad1.x, gamepad1.y,
                gamepad2RightStick, gamepad2LeftStick,
                gamepad2.dpad_left, gamepad2.dpad_right, gamepad2.dpad_up, gamepad2.dpad_down,
                gamepad1.start, gamepad2.y, gamepad2.x, gamepad1.left_bumper, gamepad1.left_trigger,
                gamepad1.right_bumper, gamepad1.right_trigger);
    }

    @Override
    public void stop()
    {
        // do we need to do anything when the robot stops?
        if (m_robot != null)
        {
            m_robot.Stop();
        }
    }
}
