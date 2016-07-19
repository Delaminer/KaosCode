package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.robocol.Telemetry;

//TODO - investigate LinarOpMode and the waits and cycles

public class Robot //extends OpMode
{
    // core competition variables (need to set with switches)
    private ROBOT_TEAM_COLOR m_teamColor = ROBOT_TEAM_COLOR.Unknown;
    private ROBOT_TEAM_COLOR m_manualSwitchTeamColor = ROBOT_TEAM_COLOR.Unknown;
    private ROBOT_STARTING_POSITION m_startingPosition = ROBOT_STARTING_POSITION.Unknown;
    private ROBOT_FINAL_DESTINATION_RAMP m_finalDestinationRamp = ROBOT_FINAL_DESTINATION_RAMP.Unknown;
    private int m_startDelay;
    public ColorSensor m_colorSensor = null;
    public RobotTracks RobotTracks = null;
    public RobotWings RobotWings = null;
    public RobotColorPaddle RobotColorPaddle = null;
    public RobotWinch RobotWinch = null;
    private ROBOT_CONTROL_MODE m_robotControlMode = ROBOT_CONTROL_MODE.Unknown;
    private boolean m_autonomousStateMachineEnabled = false;
    private int m_autonomousStateMachineStep = 0;
    private boolean m_autonomousStateMachineComplete = false;
    private HardwareMap m_hardwareMap = null;
    private Telemetry m_telemetry = null;
    private TouchSensor m_teamColorSwitchSensor = null;
    private TouchSensor m_finalDestinationRampSwitchSensor = null;
    private TouchSensor m_startDelaySwitchSensor = null;
    private TouchSensor m_startingPositionSwitchSensor = null;
    private ROBOT_AUTONOMOUS_MODE m_robotAutonomousMode = ROBOT_AUTONOMOUS_MODE.Unknown;
    private OpticalDistanceSensor mDistance = null;

    public enum ROBOT_FINAL_DESTINATION_RAMP
    {
        Unknown,
        HomeRamp,
        AwayRamp;
    }

    public enum ROBOT_STARTING_POSITION
    {
        Unknown,
        PositionA,
        PositionB;
    }

    public enum ROBOT_CONTROL_MODE
    {
        Unknown,
        Autonomous,
        TeleOp;
    }

    public enum ROBOT_TEAM_COLOR
    {
        Unknown,
        Red,
        Blue;
    }

    public enum ROBOT_AUTONOMOUS_MODE
    {
        Unknown,
        ModeOldA,
        ModeA,
        ModeABlue,
        ModeARed,
        ModeB,
        ModeCBlue,
        ModeCRed;
    }

    public enum ROBOT_WING_SIDE
    {
        Unknown,
        Right,
        Left;
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, ROBOT_CONTROL_MODE robotControlMode,
        ROBOT_AUTONOMOUS_MODE robotAutonomousMode, ROBOT_TEAM_COLOR robotTeamColor)
    {
        // save the references
        m_hardwareMap = hardwareMap;                    // maintains a reference to the hardware map class
        m_telemetry = telemetry;                        // maintains a reference to the telemetry class so we can write messages to the phone
        m_robotAutonomousMode = robotAutonomousMode;    // maintain which mode we were told to start in when in autonomous
        m_manualSwitchTeamColor = robotTeamColor;       // specifies an override for the team color and ignores the actual switch value

        // get the switch sensors so they can be read during the Init()
        try
        {
            m_teamColorSwitchSensor = m_hardwareMap.touchSensor.get("switchTeamColor");
            m_finalDestinationRampSwitchSensor = m_hardwareMap.touchSensor.get("switchFinalDestinationRamp");
            m_startDelaySwitchSensor = m_hardwareMap.touchSensor.get("switchStartDelay");
            m_startingPositionSwitchSensor = m_hardwareMap.touchSensor.get("switchStartingPosition");
        }
        catch (Exception ex)
        {
            // failed to get the sensors that may or may not be on the robot
            //m_telemetry.addData("Robot:Robot()", "Failed to get the switch sensors." + ex.toString());
            //DbgLog.msg("Robot:Robot() - Failed to get the switch sensors." + ex.toString());
        }

        // get references to the various motors, sensors and servos in stages
        TouchSensor touchSensorLeftTrackFront = null;
        TouchSensor touchSensorRightTrackFront = null;
        Servo servoLeftTrackTouch = null;
        Servo servoRightTrackTouch = null;
        OpticalDistanceSensor opticalDistanceSensorFront = null;
        DcMotor motorLeftFront = null;
        DcMotor motorLeftRear = null;
        DcMotor motorRightFront = null;
        DcMotor motorRightRear = null;

        // get the motors
        try
        {
            motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        }
        catch (Exception ex)
        {
            // failed to get the motors that may or may not be on the robot
            m_telemetry.addData("Robot:Robot()", "Failed to get the motorLeftFront that may or may not be on the robot." + ex.toString());
            DbgLog.msg("Robot:Robot() - Failed to get the motorLeftFront that may or may not be on the robot." + ex.toString());
        }
        try
        {
            motorLeftRear = hardwareMap.dcMotor.get("motorLeftRear");
        }
        catch (Exception ex)
        {
            // failed to get the motors that may or may not be on the robot
            m_telemetry.addData("Robot:Robot()", "Failed to get the motorLeftRear that may or may not be on the robot." + ex.toString());
            DbgLog.msg("Robot:Robot() - Failed to get the motorLeftRear that may or may not be on the robot." + ex.toString());
        }
        try
        {
            motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
        }
        catch (Exception ex)
        {
            // failed to get the motors that may or may not be on the robot
            m_telemetry.addData("Robot:Robot()", "Failed to get the motorRightFront that may or may not be on the robot." + ex.toString());
            DbgLog.msg("Robot:Robot() - Failed to get the motorRightFront that may or may not be on the robot." + ex.toString());
        }
        try
        {
            motorRightRear = hardwareMap.dcMotor.get("motorRightRear");
        }
        catch (Exception ex)
        {
            // failed to get the motors that may or may not be on the robot
            m_telemetry.addData("Robot:Robot()", "Failed to get the motorRightRear that may or may not be on the robot." + ex.toString());
            DbgLog.msg("Robot:Robot() - Failed to get the motorRightRear that may or may not be on the robot." + ex.toString());
        }

        // get the touch sensors
        try
        {
            touchSensorLeftTrackFront = hardwareMap.touchSensor.get("touchSensorLeftTrackFront");
            touchSensorRightTrackFront = hardwareMap.touchSensor.get("touchSensorRightTrackFront");
        }
        catch (Exception ex)
        {
            // failed to get the sensors that may or may not be on the robot
            //m_telemetry.addData("Robot:Robot()", "Failed to get the track touch sensors that may or may not be on the robot." + ex.toString());
            //DbgLog.msg("Robot:Robot() - Failed to get the track touch sensors that may or may not be on the robot." + ex.toString());
        }

        // get the touch servos
        try
        {
            servoLeftTrackTouch = hardwareMap.servo.get("servoLeftTrackTouch");
            servoRightTrackTouch = hardwareMap.servo.get("servoRightTrackTouch");
        }
        catch (Exception ex)
        {
            // failed to get the sensors that may or may not be on the robot
            //m_telemetry.addData("Robot:Robot()", "Failed to get the track touch sensor servos that may or may not be on the robot." + ex.toString());
            //DbgLog.msg("Robot:Robot() - Failed to get the track touch sensor servos that may or may not be on the robot." + ex.toString());
        }

        // get the optical distance sensors
//        try
//        {
//            opticalDistanceSensorFront = hardwareMap.opticalDistanceSensor.get("opticalDistanceSensorFront");
//        }
//        catch (Exception ex)
//        {
//            // failed to get the sensors that may or may not be on the robot
//            m_telemetry.addData("Robot:Robot()", "Failed to get the optical distance sensor that may or may not be on the robot." + ex.toString());
//            DbgLog.msg("Robot:Robot() - Failed to get the optical distance sensor that may or may not be on the robot." + ex.toString());
//        }

        // build up the tracks, the motors are always assume to be present and are created inline with the call to the constructor
        try
        {
            RobotTracks = new RobotTracks(
                motorLeftFront,
                motorLeftRear,
                motorRightFront,
                motorRightRear,
                touchSensorLeftTrackFront,
                touchSensorRightTrackFront,
                opticalDistanceSensorFront,
                servoLeftTrackTouch,
                servoRightTrackTouch);
        }
        catch (Exception ex)
        {
            // failed to load up the tracks
            //m_telemetry.addData("Robot:Robot()", "Failed on RobotTracks() " + ex.toString());
            //DbgLog.msg("Robot:Robot() - Failed on RobotTracks() " + ex.toString());
        }

        // build up the robot wings
        try
        {
            RobotWings = new RobotWings(
                hardwareMap.servo.get("servoLeftWing"),
                hardwareMap.servo.get("servoRightWing"),
                hardwareMap.servo.get("servoLeftLowWing"),
                hardwareMap.servo.get("servoRightLowWing"));
        }
        catch (Exception ex)
        {
            // failed to load up the wings, create the class using the default constructor
            RobotWings = new RobotWings();
            m_telemetry.addData("Robot:Robot()", "Failed on RobotWings() " + ex.toString());
            DbgLog.msg("Robot:Robot() - Failed on RobotWings() " + ex.toString());
        }

        // get a reference to the sensors or servos that might or might not be present
        ColorSensor colorSensorPaddleButton = null;
        DeviceInterfaceModule deviceInterfaceModule = null;
        Servo servoColorPaddleButton = null;
        try
        {
            deviceInterfaceModule = hardwareMap.deviceInterfaceModule.get("deviceInterfaceModule");
            deviceInterfaceModule.setDigitalChannelMode(RobotColorPaddle.CIM_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
            colorSensorPaddleButton = hardwareMap.colorSensor.get("colorSensorPaddleButton");
        }
        catch (Exception ex)
        {
            // failed to get the sensors that may or may not be on the robot
            //m_telemetry.addData("Robot:Robot()", "Failed to get the paddle color sensor that may or may not be on the robot." + ex.toString());
            //DbgLog.msg("Robot:Robot() - Failed to get the paddle color sensor that may or may not be on the robot." + ex.toString());
        }

        // get the color paddle button servo
        try
        {
            servoColorPaddleButton = hardwareMap.servo.get("servoColorPaddleButton");
        }
        catch (Exception ex)
        {
            // failed to get the sensors that may or may not be on the robot
            //m_telemetry.addData("Robot:Robot()", "Failed to get the paddle servo that may or may not be on the robot." + ex.toString());
            //DbgLog.msg("Robot:Robot() - Failed to get the paddle servo that may or may not be on the robot." + ex.toString());
        }

        // build up the robot color paddle
        try
        {
            RobotColorPaddle = new RobotColorPaddle(
                servoColorPaddleButton,
                colorSensorPaddleButton,
                deviceInterfaceModule);
        }
        catch (Exception ex)
        {
            // failed to load up the wings
            //RobotColorPaddle = new RobotColorPaddle();
            //m_telemetry.addData("Robot:Robot()", "Failed on RobotColorPaddle() " + ex.toString());
            //DbgLog.msg("Robot:Robot() - Failed on RobotColorPaddle() " + ex.toString());
        }

        // get a reference to the motors or servos that may or may not be on the robot
        DcMotor motorWinch = null;
        DcMotor motorTape = null;
        Servo servoWinchLock = null;
        Servo servoDump = null;

        // get the winch motor
        try
        {
            motorWinch = hardwareMap.dcMotor.get("motorWinch");
        }
        catch (Exception ex)
        {
            // failed to get the sensors that may or may not be on the robot
            m_telemetry.addData("Robot:Robot()", "Failed to get the winch motor that may or may not be on the robot." + ex.toString());
            DbgLog.msg("Robot:Robot() - Failed to get the winch motor that may or may not be on the robot." + ex.toString());
        }

        // get the tape motor
        try
        {
            motorTape = hardwareMap.dcMotor.get("motorTape");
        }
        catch (Exception ex)
        {
            // failed to get the sensors that may or may not be on the robot
            m_telemetry.addData("Robot:Robot()", "Failed to get the tape motor that may or may not be on the robot." + ex.toString());
            DbgLog.msg("Robot:Robot() - Failed to get the tape motor that may or may not be on the robot." + ex.toString());
        }

        // winch lock servo
        try
        {
            servoWinchLock = hardwareMap.servo.get("servoWinchLock");
        }
        catch (Exception ex)
        {
            // failed to get the sensors that may or may not be on the robot
            //m_telemetry.addData("Robot:Robot()", "Failed to get the winch lock servo that may or may not be on the robot." + ex.toString());
            //DbgLog.msg("Robot:Robot() - Failed to get the winch lock servo that may or may not be on the robot." + ex.toString());
        }

        // deck servo
        try
        {
            servoDump = hardwareMap.servo.get("servoDump");
        }
        catch (Exception ex)
        {
            // failed to get the sensors that may or may not be on the robot
            m_telemetry.addData("Robot:Robot()", "Failed to get the deck servo that may or may not be on the robot." + ex.toString());
            DbgLog.msg("Robot:Robot() - Failed to get the deck servo that may or may not be on the robot." + ex.toString());
        }

        // build up the robot main arm
        try
        {
            RobotWinch = new RobotWinch(
                motorWinch,
                motorTape,
                servoWinchLock,
                servoDump);
        }
        catch (Exception ex)
        {
            // failed to load up the wings
            RobotWinch = new RobotWinch();
            m_telemetry.addData("Robot:Robot()", "Failed on RobotWinch() " + ex.toString());
            DbgLog.msg("Robot:Robot() - Failed on RobotWinch() " + ex.toString());
        }

        try
        {
            mDistance = hardwareMap.opticalDistanceSensor.get("sensorODS");
        }
        catch (Exception ex)
        {
            //failed to get ODS distance
            m_telemetry.addData("Robot:Robot()", "Failed on ODS() " + ex.toString());
            DbgLog.msg("Robot:Robot() - Failed on ODS() " + ex.toString());
        }

        // set the control mode in the case that it is set on the contructor from one of the boot strap classes
        SetRobotControlMode(robotControlMode);
    }

    public void SetRobotControlMode(ROBOT_CONTROL_MODE robotControlMode)
    {
        // pass along the robot mode to the sub-components and enable all of the components
        m_robotControlMode = robotControlMode;
        if (RobotTracks != null)
        {
            RobotTracks.SetRobotControlMode(robotControlMode);
            RobotTracks.Enabled(true);
        }
        if (RobotWings != null)
        {
            RobotWings.SetRobotControlMode(robotControlMode);
            RobotWings.Enabled(true);
        }
        if (RobotColorPaddle != null)
        {
            RobotColorPaddle.SetRobotControlMode(robotControlMode);
            RobotColorPaddle.Enabled(true);
        }
        if (RobotWinch != null)
        {
            RobotWinch.SetRobotControlMode(robotControlMode);
            RobotWinch.Enabled(true);
        }

        // what do we need to set up specifically for this mode?
        switch (m_robotControlMode)
        {
            case Unknown:
                // nothing to do
                break;
            case Autonomous:
                // disable/enable the correct things (if needed)
                if (RobotTracks != null)
                {

                }
                if (RobotWings != null)
                {

                }
                if (RobotColorPaddle != null)
                {

                }
                if (RobotWinch != null)
                {

                }

                // enable the state machines on the various components to a known state
                m_autonomousStateMachineEnabled = true;
                m_autonomousStateMachineStep = 0;
                m_autonomousStateMachineComplete = false;

                break;
            case TeleOp:
                // disable/enable the correct things (if needed)
                if (RobotTracks != null)
                {

                }
                if (RobotWings != null)
                {

                }
                if (RobotColorPaddle != null)
                {

                }
                if (RobotWinch != null)
                {

                }

                // disable the state machines as it is not needed in this mode
                m_autonomousStateMachineEnabled = false;
                m_autonomousStateMachineStep = 0;
                break;
            default:
        }
    }

    public void Init()
    {
        // read the switch sensors to see what their surrent state is
        ReadSwitchSensors();

        // set some additional properties now that we know these values right away
        if (RobotColorPaddle != null)
        {
            // the paddle needs to know right away which team we are on
            RobotColorPaddle.SetTeamColor(m_teamColor);
        }
        if (RobotWings != null)
        {
            // the paddle needs to know right away which team we are on
            RobotWings.SetTeamColor(m_teamColor);
        }

        // call init on every sub class (should be an Interface class up the stack)
        try
        {
            if (RobotTracks != null)
            {
                RobotTracks.Init();
            }
            if (RobotWings != null)
            {
                RobotWings.Init();
            }
            if (RobotColorPaddle != null)
            {
                RobotColorPaddle.Init();
            }
            if (RobotWinch != null)
            {
                RobotWinch.Init();
            }
        }
        catch (Exception ex)
        {
            // faile to execute an Init()
            m_telemetry.addData("Robot:Init()", "Failed to execute Init() on each sub class" + ex.toString());
            DbgLog.msg("Robot:Init() - Failed to execute Init() on each sub class" + ex.toString());
        }
    }

    private void ReadSwitchSensors()
    {
        // read the configuration switches and get their values
        try
        {
            // read the team color sensor
            if (m_manualSwitchTeamColor != ROBOT_TEAM_COLOR.Unknown)
            {
                if (m_manualSwitchTeamColor == ROBOT_TEAM_COLOR.Red)
                {
                    m_teamColor = ROBOT_TEAM_COLOR.Red;
                    //m_telemetry.addData("Robot:ReadSwitchSensors(color)", "Manually setting m_teamColor: Red");
            }
                else if (m_manualSwitchTeamColor == ROBOT_TEAM_COLOR.Blue)
                {
                    m_teamColor = ROBOT_TEAM_COLOR.Blue;
                   // m_telemetry.addData("Robot:ReadSwitchSensors(color)", "Manually setting m_teamColor: Blue");
                }
            }
            else
            {
                if ((m_teamColorSwitchSensor != null) && (m_teamColorSwitchSensor.isPressed()))
                {
                    m_teamColor = ROBOT_TEAM_COLOR.Blue;
                    //m_telemetry.addData("Robot:ReadSwitchSensors(color)", "m_teamColor: Blue");
                }
                else if ((m_teamColorSwitchSensor != null) && (!m_teamColorSwitchSensor.isPressed()))
                {
                    m_teamColor = ROBOT_TEAM_COLOR.Red;
                    //m_telemetry.addData("Robot:ReadSwitchSensors(color)", "m_teamColor: Red");
                } else
                {
                    m_teamColor = ROBOT_TEAM_COLOR.Unknown;
                    //m_telemetry.addData("Robot:ReadSwitchSensors(color)", "m_teamColor: Unknown");
                }
            }
        }
        catch (Exception ex)
        {
            // failed to read the team color sensor
            m_teamColor = ROBOT_TEAM_COLOR.Unknown;
            //m_telemetry.addData("Robot:ReadSwitchSensors(color)", "Failed to get m_teamColorSwitchSensor switch" + ex.toString());
            //DbgLog.msg("Robot:ReadSwitchSensors() - Failed to get m_teamColorSwitchSensor switch" + ex.toString());
        }

        // read the configuration switches and get their values
        try
        {
            // read the starting position sensor
            if ((m_startingPositionSwitchSensor != null) && (m_startingPositionSwitchSensor.isPressed()))
            {
                m_startingPosition  = ROBOT_STARTING_POSITION.PositionA;
                //m_telemetry.addData("Robot:ReadSwitchSensors(pos)", "m_startingPosition: PositionA");
            }
            else if ((m_startingPositionSwitchSensor != null) && (!m_startingPositionSwitchSensor.isPressed()))
            {
                m_startingPosition  = ROBOT_STARTING_POSITION.PositionB;
               // m_telemetry.addData("Robot:ReadSwitchSensors(pos)", "m_startingPosition: PositionB");
            }
            else
            {
                m_startingPosition  = ROBOT_STARTING_POSITION.Unknown;
                //m_telemetry.addData("Robot:ReadSwitchSensors(pos)", "m_startingPosition: Unknown");
            }
        }
        catch (Exception ex)
        {
            // failed to read the starting position sensor
            m_startingPosition  = ROBOT_STARTING_POSITION.Unknown;
            //m_telemetry.addData("Robot:ReadSwitchSensors(pos)", "Failed to get m_startingPositionSwitchSensor switch" + ex.toString());
            //DbgLog.msg("Robot:ReadSwitchSensors() - Failed to get m_startingPositionSwitchSensor switch" + ex.toString());
        }

        // read the configuration switches and get their values
        try
        {
            // read the start delay sensor
            if ((m_startDelaySwitchSensor != null) && (m_startDelaySwitchSensor.isPressed()))
            {
                // zero second delay
                m_startDelay = 0;
                //m_telemetry.addData("Robot:ReadSwitchSensors(delay)", "m_startDelay: 0");
            }
            else if ((m_startDelaySwitchSensor != null) && (!m_startDelaySwitchSensor.isPressed()))
            {
                // 10 second delay
                m_startDelay  = 10;
                //m_telemetry.addData("Robot:ReadSwitchSensors(delay)", "m_startDelay: 10");
            }
            else
            {
                // 10 second delay
                m_startDelay  = 0;
                //m_telemetry.addData("Robot:ReadSwitchSensors(delay)", "m_startDelay: Default:0");
            }
        }
        catch (Exception ex)
        {
            // failed to read the start delay sensor
            m_startDelay  = 0;
            //m_telemetry.addData("Robot:ReadSwitchSensors(delay)", "Failed to get m_startDelaySwitchSensor switch" + ex.toString());
            //DbgLog.msg("Robot:ReadSwitchSensors() - Failed to get m_startDelaySwitchSensor switch" + ex.toString());
        }

        // read the configuration switches and get their values
        try
        {
            // read the final destination ramp sensor
            if ((m_finalDestinationRampSwitchSensor != null) && (m_finalDestinationRampSwitchSensor.isPressed()))
            {
                m_finalDestinationRamp = ROBOT_FINAL_DESTINATION_RAMP.HomeRamp;
                //m_telemetry.addData("Robot:ReadSwitchSensors(dest)", "m_finalDestinationRamp: HomeRamp");
            }
            else if ((m_finalDestinationRampSwitchSensor != null) && (!m_finalDestinationRampSwitchSensor.isPressed()))
            {
                m_finalDestinationRamp= ROBOT_FINAL_DESTINATION_RAMP.AwayRamp;
               // m_telemetry.addData("Robot:ReadSwitchSensors(dest)", "m_finalDestinationRamp: AwayRamp");
            }
            else
            {
                m_finalDestinationRamp= ROBOT_FINAL_DESTINATION_RAMP.Unknown;
               // m_telemetry.addData("Robot:ReadSwitchSensors(dest)", "m_finalDestinationRamp: Unknown");
            }
        }
        catch (Exception ex)
        {
            // failed to read the final destination ramp sensor
            m_finalDestinationRamp= ROBOT_FINAL_DESTINATION_RAMP.Unknown;
            //m_telemetry.addData("Robot:ReadSwitchSensors(dest)", "Failed to get m_finalDestinationRampSwitchSensor switch" + ex.toString());
            DbgLog.msg("Robot:ReadSwitchSensors() - Failed to get m_finalDestinationRampSwitchSensor switch" + ex.toString());
        }
    }

    public void Start()
    {
        // start the robot - depending on the mode that it is in, pass the Start() on up the chain
        try
        {
            if (RobotTracks != null)
            {
                RobotTracks.Start();
            }
            if (RobotWings != null)
            {
                RobotWings.Start();
            }
            if (RobotColorPaddle != null)
            {
                RobotColorPaddle.Start();
            }
            if (RobotWinch != null)
            {
                RobotWinch.Start();
            }
        }
        catch (Exception ex)
        {
            // failed to stop certain parts of the robot
            //m_telemetry.addData("Robot:Start()", "Failed to execute Start() on each sub class" + ex.toString());
            //DbgLog.msg("Robot:Start() - Failed to execute Start() on each sub class" + ex.toString());
        }
    }

    public void Stop()
    {
        // stop the robot and put everything back to its default positions
        try
        {
            if (RobotTracks != null)
            {
                RobotTracks.Stop();
            }
            if (RobotWings != null)
            {
                RobotWings.Stop();
            }
            if (RobotColorPaddle != null)
            {
                RobotColorPaddle.Stop();
            }
            if (RobotWinch != null)
            {
                RobotWinch.Stop();
            }
        }
        catch (Exception ex)
        {
            // failed to stop certain parts of the robot
            //m_telemetry.addData("Robot:Stop()", "Failed to execute Stop() on each sub class" + ex.toString());
            //DbgLog.msg("Robot:Stop() - Failed to execute Stop() on each sub class" + ex.toString());
        }
    }

    public void Loop(
        float gamepad1LeftStickScaledValue, float gamepad1RightStickScaledValue,
        boolean gamepad2LeftBumper, float gamepad2LeftTrigger,
        boolean gamepad2RightBumper, float gamepad2RightTrigger,
        boolean gamepad1ButtonX, boolean gamepad1ButtonY,
        float gamepad2LeftStickScaledValue, float gamepad2RightStickScaledValue,
        boolean gamepad2DpadLeft, boolean gamepad2DpadRight, boolean gamepad2DpadUp, boolean gamepad2DpadDown,
        boolean gamepad1Start, boolean gamepad2Y, boolean gamepad2X,
        boolean gamepad1LeftBumper, float gamepad1LeftTrigger,
        boolean gamepad1RightBumper, float gamepad1RightTrigger)
    {
        switch (m_robotControlMode)
        {
            case Unknown:
                // nothing to do - mode not set, watch for a mode set?
                if ((gamepad1Start) && (gamepad1ButtonY))
                {
                    // manaully set the robot to Autonomous mode when it is currently Unknown
                    SetRobotControlMode(ROBOT_CONTROL_MODE.Autonomous);
                }
                else if ((gamepad1Start) && (gamepad1ButtonX))
                {
                    // manaully set the robot to TeleOp mode when it is currently Unknown
                    SetRobotControlMode(ROBOT_CONTROL_MODE.TeleOp);
                }
                break;
            case Autonomous:
                // let focus through to the state machine of the various components
				if (m_robotAutonomousMode == ROBOT_AUTONOMOUS_MODE.ModeA)
				{
					if (!RunAutonomousStateMachineModeA())
					{
						// autonomous is over, switch back to TeleOp
						SetRobotControlMode(ROBOT_CONTROL_MODE.TeleOp);
					}
				}
                else if (m_robotAutonomousMode == ROBOT_AUTONOMOUS_MODE.ModeOldA)
                {
                    if (!RunAutonomousStateMachineModeOLDA())
                    {
                        // autonomous is over, switch back to TeleOp
                        SetRobotControlMode(ROBOT_CONTROL_MODE.TeleOp);
                    }
                }
                else if (m_robotAutonomousMode == ROBOT_AUTONOMOUS_MODE.ModeABlue)
                {
                    if (!RunAutonomousStateMachineModeABlue())
                    {
                        // autonomous is over, switch back to TeleOp
                        SetRobotControlMode(ROBOT_CONTROL_MODE.TeleOp);
                    }
                }
                else if (m_robotAutonomousMode == ROBOT_AUTONOMOUS_MODE.ModeARed)
                {
                    if (!RunAutonomousStateMachineModeARed())
                    {
                        // autonomous is over, switch back to TeleOp
                        SetRobotControlMode(ROBOT_CONTROL_MODE.TeleOp);
                    }
                }
				else if (m_robotAutonomousMode == ROBOT_AUTONOMOUS_MODE.ModeB)
				{
					if (!RunAutonomousStateMachineModeB())
					{
						// autonomous is over, switch back to TeleOp
						SetRobotControlMode(ROBOT_CONTROL_MODE.TeleOp);
					}
				}
                else if (m_robotAutonomousMode == ROBOT_AUTONOMOUS_MODE.ModeCBlue)
                {
                    if (!RunAutonomousStateMachineModeCBlue())
                    {
                        // autonomous is over, switch back to TeleOp
                        SetRobotControlMode(ROBOT_CONTROL_MODE.TeleOp);
                    }
                }
                else if (m_robotAutonomousMode == ROBOT_AUTONOMOUS_MODE.ModeCRed)
                {
                    if (!RunAutonomousStateMachineModeCRed())
                    {
                        // autonomous is over, switch back to TeleOp
                        SetRobotControlMode(ROBOT_CONTROL_MODE.TeleOp);
                    }
                }
                break;
            case TeleOp:
                // user/controller feedback is passed in the loop methods below
                break;
            default:
        }

        // need to read switches?
        if (gamepad1Start)
        {
            // teh start button on controller 1 is pressed, re-read the switches
            ReadSwitchSensors();
        }

        // call all the sub-loop methods and pass it up the chain
        try
        {
            if (RobotTracks != null)
            {
                RobotTracks.Loop(gamepad1LeftStickScaledValue, gamepad1RightStickScaledValue,
                        gamepad2Y); // intelligent braking gamepad1LeftBumper, gamepad1RightBumper
            }
            if (RobotWings != null)
            {
                RobotWings.Loop(gamepad2LeftBumper, gamepad2LeftTrigger,
                        gamepad2RightBumper, gamepad2RightTrigger,
                        gamepad1LeftBumper, gamepad1LeftTrigger,
                        gamepad1RightBumper, gamepad1RightTrigger);
            }
            if (RobotColorPaddle != null)
            {
                RobotColorPaddle.Loop(gamepad2DpadLeft, gamepad2DpadRight, gamepad2DpadUp, gamepad2DpadDown);
            }
            if (RobotWinch != null)
            {
                RobotWinch.Loop(gamepad2LeftStickScaledValue, gamepad2RightStickScaledValue,
                        gamepad2DpadUp, gamepad2DpadDown);
            }
        }
        catch (Exception ex)
        {
            // failed to execute loop on each sub module
            m_telemetry.addData("Robot:Loop()", "Failed to execute Loop() on each sub class" + ex.toString());
            DbgLog.msg("Robot:Loop() - Failed to execute Loop() on each sub class" + ex.toString());
        }
    }

    private boolean RunAutonomousStateMachineModeOLDA()
    {
        int travelDistance = 0;
        int turnDegree = 0;

        if (!m_autonomousStateMachineEnabled)
        {
            // state machine is not enabled, exit
            return false;
        }

        // process the current state
        switch (m_autonomousStateMachineStep)
        {
            case 0:
                // make sure we are starting from a known state
                RobotTracks.ResetEcoders();
                m_autonomousStateMachineStep++;
                break;

            case 1: // STEP 1 - pause for a few seconds if needed

                // sleep the current thread for a while (number of seconds x 1000)
                if (m_startDelay > 0)
                {
                    try
                    {
                        // start delay is in seconds, times that by 1000 to get milliseconds
                        Thread.sleep(m_startDelay * 1000);
                    }
                    catch (InterruptedException ex)
                    {

                    }
                    m_autonomousStateMachineStep++;
                }
                else
                {
                    m_autonomousStateMachineStep++;
                }
                break;

            case 2: // STEP 1

                // Check start position and go straight forward for 27"
                RobotTracks.SetDirectionForward();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 1.0f);
                if (m_startingPosition == ROBOT_STARTING_POSITION.PositionA)
                {
                    travelDistance = 27;
                }
                else
                {
                    travelDistance = 3;
                }
                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(travelDistance))) &&
                    (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(travelDistance))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 3: // STEP 2

                // Wait for the encoders to reset
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                    (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 4: // STEP 3

                // raise the arm slightly above level
                /*RobotWinch.SetTapeDirectionUp();
                RobotWinch.SetTapeRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 1.0f);

                // see if we have run to the correct position
                if (RobotWinch.HasTapeRunToPosition(1000f)) // TODO figure out what this number should be
                {
                    RobotWinch.ResetTapeEncoder();
                    RobotWinch.StopTape();
                    m_autonomousStateMachineStep++;
                }*/
                m_autonomousStateMachineStep++;
                break;
            case 5: // STEP 4

                // Wait for the encoders to reset
                //if  (RobotWinch.GetTapeCurrentPosition() == 0)
                //{
                    m_autonomousStateMachineStep++;
                //}
                break;
            case 6: // STEP 5

                // Turn toward the white line, if team red and start a or team blue and start b, turn left. Else, turn right. (90 degrees)
                RobotTracks.SetDirectionForward();
                if (m_teamColor == ROBOT_TEAM_COLOR.Blue)
                {
                    RobotTracks.LeftTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 0.5f);
                    RobotTracks.RightTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, -0.5f);
                }
                else
                {
                    RobotTracks.LeftTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, -0.5f);
                    RobotTracks.RightTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 0.5f);
                }
                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForDegreeTurn(45))) &&
                    (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForDegreeTurn(45))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 7: // STEP 6

                // Wait for the encoders to reset
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                    (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 8: // STEP 7

                // Continue straight forward (for 68")
                RobotTracks.SetDirectionForward();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 1.0f);

                if (m_startingPosition == ROBOT_STARTING_POSITION.PositionA)
                {
                    travelDistance = 68;
                }
                else
                {
                    travelDistance = 102;
                }
                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(travelDistance))) &&
                    (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(travelDistance))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 9: // STEP 8

                // Wait for reset encoders
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                    (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 10: // STEP 9

                // Turn slightly toward the color tower, (45 degrees)
                RobotTracks.SetDirectionForward();
                if (m_teamColor == ROBOT_TEAM_COLOR.Blue)
                {
                    RobotTracks.LeftTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 0.5f);
                    RobotTracks.RightTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, -0.5f);
                }
                else
                {
                    RobotTracks.LeftTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, -0.5f);
                    RobotTracks.RightTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 0.5f);
                }

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForDegreeTurn(45))) &&
                    (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForDegreeTurn(45))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 11: // STEP 10
                // Wait for reset encoders
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                    (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 12: // STEP 11

                // pull the winch down to hang over the color box
                /*RobotWinch.SetWinchDirectionIn();
                RobotWinch.SetWinchRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 1.0f);

                // see if we have run to the correct position
                if (RobotWinch.HasWinchRunToPosition(15000f)) // TODO figure out what this number should be
                {
                    RobotWinch.ResetWinchEncoder();
                    RobotWinch.StopWinch();
                    m_autonomousStateMachineStep++;
                }*/
                m_autonomousStateMachineStep++;
                break;
            case 13: // STEP 12

                // Wait for the encoders to reset
                //if  (RobotWinch.GetWinchCurrentPosition() == 0)
                //{
                    m_autonomousStateMachineStep++;
                //}
                break;
            case 14: // STEP 13

                // Continue straight forward until we hit the wall (for 18") Note: EnableIntelligentBreaking(true) & watch for forced stop
                RobotTracks.SetDirectionForward();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 1.0f);
                RobotTracks.EnableIntelligentBreaking(true);

                // how far do we need to go?
                if (m_startingPosition == ROBOT_STARTING_POSITION.PositionA)
                {
                    travelDistance = 15;
                }
                else
                {
                    travelDistance = 27;
                }

                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(travelDistance)) || (RobotTracks.LeftTrack.IsForcedStopped())) &&
                    (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(travelDistance))|| (RobotTracks.RightTrack.IsForcedStopped())))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    RobotTracks.EnableIntelligentBreaking(false);
                    m_autonomousStateMachineStep++;
                }
                break;
            case 15: // STEP 14

                // Wait for reset encoders
                if ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                    (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 16: // STEP 15

                // raise the arm slightly again to dump the climbers in
               /* RobotWinch.SetTapeDirectionUp();
                RobotWinch.SetTapeRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 1.0f);

                // see if we have run to the correct position
                if (RobotWinch.HasTapeRunToPosition(100f)) // TODO figure out what this number should be
                {
                    RobotWinch.ResetTapeEncoder();
                    RobotWinch.StopTape();
                    m_autonomousStateMachineStep++;
                }*/
                m_autonomousStateMachineStep++;
                break;
            case 17: // STEP 16

                // Wait for the encoders to reset
               // if  (RobotWinch.GetTapeCurrentPosition() == 0)
                //{
                    m_autonomousStateMachineStep++;
                //}
                break;
            case 18: // STEP 17

                // try to figure out which button to press
               // RobotColorPaddle.PushTeamColorButton();
                m_autonomousStateMachineStep++;
                break;
            case 19: // STEP 18

                // wait for what ??
                m_autonomousStateMachineStep++;
                break;
            case 20: // STEP 19

                // Continue straight backward (for 24")
                RobotTracks.SetDirectionReverse();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 1.0f);

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(24))) &&
                    (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(24))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 21: // STEP 20

                // Wait for reset encoders
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                    (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 22: // STEP 21

                // Turn slightly toward the color tower, (180 ish degrees)
                RobotTracks.SetDirectionForward();
                RobotTracks.LeftTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, -1.0f);
                RobotTracks.RightTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 1.0f);

                if ((m_finalDestinationRamp == ROBOT_FINAL_DESTINATION_RAMP.HomeRamp) && (m_teamColor == ROBOT_TEAM_COLOR.Red))
                {
                    turnDegree = 135;
                }
                else if ((m_finalDestinationRamp == ROBOT_FINAL_DESTINATION_RAMP.HomeRamp) && (m_teamColor == ROBOT_TEAM_COLOR.Blue))
                {
                    turnDegree = 225;
                }
                else
                {
                    turnDegree = 180;
                }
                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForDegreeTurn(turnDegree))) &&
                    (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForDegreeTurn(turnDegree))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 23: // STEP 22
                // Wait for reset encoders
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                    (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 24: // STEP 23

                // Continue straight to base of mountain.
                RobotTracks.SetDirectionReverse();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 1.0f);

                //TODO depends on final hill and team??

                if (m_finalDestinationRamp == ROBOT_FINAL_DESTINATION_RAMP.HomeRamp)
                {
                    travelDistance = 16;
                }
                else
                {
                    travelDistance = 57;
                }

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(travelDistance))) &&
                    (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(travelDistance))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 25: // STEP 24

                // Wait for reset encoders
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                    (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 26: // STEP 25

                // turn a bit and line up on the hill
                RobotTracks.SetDirectionForward();
                RobotTracks.LeftTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, -0.5f);
                RobotTracks.RightTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 0.5f);

                if ((m_finalDestinationRamp == ROBOT_FINAL_DESTINATION_RAMP.HomeRamp) && (m_teamColor == ROBOT_TEAM_COLOR.Red))
                {
                    turnDegree = -90;
                }
                else if ((m_finalDestinationRamp == ROBOT_FINAL_DESTINATION_RAMP.HomeRamp) && (m_teamColor == ROBOT_TEAM_COLOR.Blue))
                {
                    turnDegree = 90;
                }
                else if ((m_finalDestinationRamp == ROBOT_FINAL_DESTINATION_RAMP.AwayRamp) && (m_teamColor == ROBOT_TEAM_COLOR.Blue))
                {
                    turnDegree = -45;
                }
                else
                {
                    turnDegree = 45;
                }


                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForDegreeTurn(turnDegree))) &&
                    (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForDegreeTurn(turnDegree))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 27: // STEP 26

                // complete turn and line up on ramp
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                    (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 28: // STEP 27

                // drop wings (only the wing on the correct side of the robot for the one we are heading up)
                if (m_teamColor == ROBOT_TEAM_COLOR.Blue)
                {
                    RobotWings.LeftWing.WingDown();
                }
                else
                {
                    RobotWings.RightWing.WingDown();
                }
                m_autonomousStateMachineStep++;
                break;
            case 29: // STEP 28

                // start heading up the hill at reduced power (for how far?
                RobotTracks.SetDirectionReverse();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, .5f);
                //Away ramp needs to travel 26" to get to bottom or ramp, home ramp 32"
                //Mountain is 54" to top of high zone, longest awaay is 80", home 86"
                if (m_finalDestinationRamp == ROBOT_FINAL_DESTINATION_RAMP.HomeRamp)
                {
                    travelDistance = 86;
                }
                else
                {
                    travelDistance = 54;
                }
                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(travelDistance))) &&
                    (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(travelDistance))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 30: // STEP 29

                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                    (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 31: // STEP 30

                // pull the wing up once we have stopped
                RobotWings.WingsUp();
                m_autonomousStateMachineStep++;
                break;
			case 32: // STEP 31 - Pause example
			
				// sleep the current thread for a while (number of seconds x 1000)
                try 
				{
					Thread.sleep(1000);
				} 
				catch (InterruptedException e) 
				{
					
				} 
				m_autonomousStateMachineStep++;

                break;
            default:
                // should never reach this code until the mode is done
                RobotColorPaddle.SetPaddleCenter();
                RobotColorPaddle.Stop();
                RobotWings.WingsUp();
                RobotWings.Stop();
                RobotWinch.ResetTapeEncoder();
                RobotWinch.StopTape();
                RobotWinch.ResetWinchEncoder();
                RobotWinch.StopWinch();
                RobotTracks.ResetEcoders();
                RobotTracks.Stop();
                m_autonomousStateMachineComplete = true;
                break;
        }
        return  !m_autonomousStateMachineComplete;
    }

    private boolean RunAutonomousStateMachineModeA()
    {
        int travelDistance = 0;
        int turnDegree = 0;

        if (!m_autonomousStateMachineEnabled)
        {
            // state machine is not enabled, exit
            return false;
        }

        // process the current state
        switch (m_autonomousStateMachineStep)
        {
            case 0:
                // make sure we are starting from a known state
                RobotTracks.ResetEcoders();
                m_autonomousStateMachineStep++;
                break;
            case 1: // STEP 1 - pause for a few seconds if needed

                // sleep the current thread for a while (number of seconds x 1000)
                if (m_startDelay > 0)
                {
                    try
                    {
                        // start delay is in seconds, times that by 1000 to get milliseconds
                        Thread.sleep(m_startDelay * 1000);
                    }
                    catch (InterruptedException ex)
                    {

                    }
                    m_autonomousStateMachineStep++;
                }
                else
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 2:

                RobotTracks.SetDirectionForward();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, .2f);

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(50))) &&
                        (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(50))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 3: // STEP 2

                // Wait for the encoders to reset
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                        (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            /*case 4:

                RobotTracks.SetDirectionForward();
                RobotTracks.LeftTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 0.3f);
                RobotTracks.RightTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, -0.3f);

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForDegreeTurn(30))) &&
                        (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForDegreeTurn(30))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 5:

                // Wait for the encoders to reset
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                        (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 6:

                RobotTracks.SetDirectionForward();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, .2f);

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(18))) &&
                        (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(18))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 7:

                // Wait for the encoders to reset
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                        (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;*/
            default:
                // should never reach this code until the mode is done
                RobotColorPaddle.SetPaddleCenter();
                RobotColorPaddle.Stop();
                RobotWings.WingsUp();
                RobotWings.Stop();
                RobotWinch.ResetTapeEncoder();
                RobotWinch.StopTape();
                RobotWinch.ResetWinchEncoder();
                RobotWinch.StopWinch();
                RobotTracks.ResetEcoders();
                RobotTracks.Stop();
                m_autonomousStateMachineComplete = true;
                break;
        }
        return  !m_autonomousStateMachineComplete;
    }

    private boolean RunAutonomousStateMachineModeABlue()
    {
        int travelDistance = 0;
        int turnDegree = 0;

        if (!m_autonomousStateMachineEnabled)
        {
            // state machine is not enabled, exit
            return false;
        }

        // process the current state
        switch (m_autonomousStateMachineStep)
        {
            case 0:
                // make sure we are starting from a known state
                RobotTracks.ResetEcoders();
                m_autonomousStateMachineStep++;
                break;
            case 1: // STEP 1 - pause for a few seconds if needed

                // sleep the current thread for a while (number of seconds x 1000)
                if (m_startDelay > 0)
                {
                    try
                    {
                        // start delay is in seconds, times that by 1000 to get milliseconds
                        Thread.sleep(m_startDelay * 1000);
                    }
                    catch (InterruptedException ex)
                    {

                    }
                    m_autonomousStateMachineStep++;
                }
                else
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 2:

                RobotTracks.SetDirectionForward();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, .2f);

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(50))) &&
                        (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(50))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 3: // STEP 2

                // Wait for the encoders to reset
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                        (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 4:

                RobotTracks.SetDirectionForward();
                RobotTracks.LeftTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 0.3f);
                RobotTracks.RightTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, -0.3f);

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForDegreeTurn(30))) &&
                        (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForDegreeTurn(30))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 5:

                // Wait for the encoders to reset
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                        (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 6:

                RobotTracks.SetDirectionForward();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, .2f);

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(18))) &&
                        (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(18))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 7:

                // Wait for the encoders to reset
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                        (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            default:
                // should never reach this code until the mode is done
                RobotColorPaddle.SetPaddleCenter();
                RobotColorPaddle.Stop();
                RobotWings.WingsUp();
                RobotWings.Stop();
                RobotWinch.ResetTapeEncoder();
                RobotWinch.StopTape();
                RobotWinch.ResetWinchEncoder();
                RobotWinch.StopWinch();
                RobotTracks.ResetEcoders();
                RobotTracks.Stop();
                m_autonomousStateMachineComplete = true;
                break;
        }
        return  !m_autonomousStateMachineComplete;
    }

    private boolean RunAutonomousStateMachineModeARed()
    {
        int travelDistance = 0;
        int turnDegree = 0;

        if (!m_autonomousStateMachineEnabled)
        {
            // state machine is not enabled, exit
            return false;
        }

        // process the current state
        switch (m_autonomousStateMachineStep)
        {
            case 0:
                // make sure we are starting from a known state
                RobotTracks.ResetEcoders();
                m_autonomousStateMachineStep++;
                break;
            case 1: // STEP 1 - pause for a few seconds if needed

                // sleep the current thread for a while (number of seconds x 1000)
                if (m_startDelay > 0)
                {
                    try
                    {
                        // start delay is in seconds, times that by 1000 to get milliseconds
                        Thread.sleep(m_startDelay * 1000);
                    }
                    catch (InterruptedException ex)
                    {

                    }
                    m_autonomousStateMachineStep++;
                }
                else
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 2:

                RobotTracks.SetDirectionForward();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, .2f);

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(50))) &&
                        (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(50))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 3: // STEP 2

                // Wait for the encoders to reset
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                        (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 4:

                RobotTracks.SetDirectionForward();
                RobotTracks.LeftTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, -0.3f);
                RobotTracks.RightTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 0.3f);

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForDegreeTurn(30))) &&
                        (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForDegreeTurn(30))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 5:

                // Wait for the encoders to reset
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                        (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 6:

                RobotTracks.SetDirectionForward();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, .2f);

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(18))) &&
                        (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(18))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 7:

                // Wait for the encoders to reset
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                        (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            default:
                // should never reach this code until the mode is done
                RobotColorPaddle.SetPaddleCenter();
                RobotColorPaddle.Stop();
                RobotWings.WingsUp();
                RobotWings.Stop();
                RobotWinch.ResetTapeEncoder();
                RobotWinch.StopTape();
                RobotWinch.ResetWinchEncoder();
                RobotWinch.StopWinch();
                RobotTracks.ResetEcoders();
                RobotTracks.Stop();
                m_autonomousStateMachineComplete = true;
                break;
        }
        return  !m_autonomousStateMachineComplete;
    }

    private boolean RunAutonomousStateMachineModeB()
    {
        int travelDistance = 0;
        int turnDegree = 0;

        if (!m_autonomousStateMachineEnabled)
        {
            // state machine is not enabled, exit
            return false;
        }

        // process the current state
        switch (m_autonomousStateMachineStep)
        {
            case 0:
                // make sure we are starting from a known state
                RobotTracks.ResetEcoders();
                m_autonomousStateMachineStep++;
                break;
            case 1: // STEP 1 - pause for a few seconds if needed

                // sleep the current thread for a while (number of seconds x 1000)
                if (m_startDelay > 0)
                {
                    try
                    {
                        // start delay is in seconds, times that by 1000 to get milliseconds
                        Thread.sleep(m_startDelay * 1000);
                    }
                    catch (InterruptedException ex)
                    {

                    }
                    m_autonomousStateMachineStep++;
                }
                else
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 2:

                RobotTracks.SetDirectionForward();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, .2f);

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(75))) &&
                    (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(75))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 3: // STEP 2

                // Wait for the encoders to reset
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                    (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 4:

                RobotTracks.SetDirectionReverse();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 1.0f);

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(50))) &&
                        (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(50))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 5:

                // Wait for the encoders to reset
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                        (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 6:

                RobotTracks.SetDirectionForward();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, .2f);

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(55))) &&
                        (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(55))))
                {
                    RobotTracks.ResetEcoders();
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 7:

                // Wait for the encoders to reset
                if  ((RobotTracks.LeftTrack.GetCurrentPosition() == 0)  &&
                        (RobotTracks.RightTrack.GetCurrentPosition() == 0))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
             default:
                // should never reach this code until the mode is done
                RobotColorPaddle.SetPaddleCenter();
                RobotColorPaddle.Stop();
                RobotWings.WingsUp();
                RobotWings.Stop();
                RobotWinch.ResetTapeEncoder();
                RobotWinch.StopTape();
                RobotWinch.ResetWinchEncoder();
                RobotWinch.StopWinch();
                RobotTracks.ResetEcoders();
                RobotTracks.Stop();
                m_autonomousStateMachineComplete = true;
                break;
        }
        return  !m_autonomousStateMachineComplete;
    }
    // Auto C Blue
    private boolean RunAutonomousStateMachineModeCBlue()
    {
        int travelDistance = 0;
        int turnDegree = 0;

//        // hsvValues is an array that will hold the hue, saturation, and value information.
//        float hsvValues[] = {0F,0F,0F};
//
//        // values is a reference to the hsvValues array.
//        final float values[] = hsvValues;
//        int red = 0;
//        int blue = 0;
//        float hue = 0.0f;
//
//        // convert the RGB values to HSV values.
//        Color.RGBToHSV((m_colorSensor.red() * 255) / 800, (m_colorSensor.green() * 255) / 800,
//                (m_colorSensor.blue() * 255) / 800, hsvValues);
//
//        // convert the hue to a color
//        int color = Color.HSVToColor(0xff, values);
//
//        red = m_colorSensor.red();
//        blue = m_colorSensor.blue();
//        hue = hsvValues[0];

        //ODS
        if (mDistance != null)
        {
            double light = mDistance.getLightDetected();
            m_telemetry.addData("Robot:Robot():LightDetected", light);
        }
        if (!m_autonomousStateMachineEnabled)
        {
            // state machine is not enabled, exit
            return false;
        }

        // process the current state
        switch (m_autonomousStateMachineStep)
        {


            case 0:
                // make sure we are starting from a known state
                //m_telemetry.addData("Unknown to Dump", "Red =",+ red, "Blue =",+ blue);
                RobotTracks.ResetEcoders();
                m_autonomousStateMachineStep++;
                break;
            case 1: // STEP 1 - pause for a few seconds if needed

                // sleep the current thread for a while (number of seconds x 1000)
                if (m_startDelay > 0)
                {
                    try
                    {
                        // start delay is in seconds, times that by 1000 to get milliseconds
                        Thread.sleep(m_startDelay * 1000);
                    }
                    catch (InterruptedException ex)
                    {

                    }
                    m_autonomousStateMachineStep++;
                }
                else {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 2:
                // backwards

                RobotTracks.SetDirectionForward();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, -0.2f);

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(-76))) &&
                        (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(-76))))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            // removed reset
            case 3:
                //Arc Turn towards beacon, back left for blue, accumulate encoder counts

                RobotTracks.SetDirectionForward();
                RobotTracks.LeftTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 0.0f);
                RobotTracks.RightTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, -0.3f);

                // see if we have run to the correct position
                if (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(-93)))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            // removed wait for stop, want to carry momentum since moving slow
            case 4:
                // back towards beacon, check encoder, distance, light to stop

                RobotTracks.SetDirectionForward();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, -0.2f);

                // see if we have run to the correct position, blue > 300, red > 300, dist <= 4.5", dist >= 1.5"
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(-103))) &&
                        (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(-120))) ||
                	(((mDistance.getLightDetected() <= 1 && mDistance.getLightDetected() >= 0.02))
                	//in dump position
                        ))
                {


                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 5:
                // Dump if in the correct postion
                if  ((mDistance.getLightDetected() <= 1 && mDistance.getLightDetected() >= 0.02))
                {
                    //climberDump.DumpDown
                    //m_telemetry.addData("Robot:Robot()", "Safe to Dump", "Red =", m_colorSensor.red(), "Blue =",m_colorSensor.blue());
                    RobotWinch.DumpUp(true);
                	m_autonomousStateMachineStep++;
                    }
                else
                {
                    //do nothing, not in a safew spot to drop climbers
                    //m_telemetry.addData("Robot:Robot()", "Not Safe to Dump", "Red =", m_colorSensor.red(), "Blue =",m_colorSensor.blue());

                	m_autonomousStateMachineStep++;
                    }

                break;
            default:
                // should never reach this code until the mode is done
                RobotColorPaddle.SetPaddleCenter();
                RobotColorPaddle.Stop();
                RobotWings.WingsUp();
                RobotWings.Stop();
                RobotWinch.ResetTapeEncoder();
                RobotWinch.StopTape();
                RobotWinch.ResetWinchEncoder();
                RobotWinch.StopWinch();
                RobotTracks.ResetEcoders();
                RobotTracks.Stop();
                m_autonomousStateMachineComplete = true;
                break;
        }
        return  !m_autonomousStateMachineComplete;
    }
    // Auto C Red
    private boolean RunAutonomousStateMachineModeCRed()
    {
        int travelDistance = 0;
        int turnDegree = 0;

//        // hsvValues is an array that will hold the hue, saturation, and value information.
//        float hsvValues[] = {0F,0F,0F};
//
//        // values is a reference to the hsvValues array.
//        final float values[] = hsvValues;
//        int red = 0;
//        int blue = 0;
//        float hue = 0.0f;
//
//        // convert the RGB values to HSV values.
//        Color.RGBToHSV((m_colorSensor.red() * 255) / 800, (.green() * 255) / 800,
//                (m_colorSensor.blue() * 255) / 800, hsvValues);
//
//        // convert the hue to a color
//        int color = Color.HSVToColor(0xff, values);
//
//        red = m_colorSensor.red();
//        blue = m_colorSensor.blue();
//        hue = hsvValues[0];

        //ODS

        if (mDistance != null)
        {
            double light = mDistance.getLightDetected();
            m_telemetry.addData("Robot:Robot():LightDetected", light);
        }
        if (!m_autonomousStateMachineEnabled)
        {
            // state machine is not enabled, exitm_colorSensor
            return false;
        }

        // process the current state
        switch (m_autonomousStateMachineStep)
        {


            case 0:
                // make sure we are starting from a known state
                //m_telemetry.addData("Unknown to Dump", "Red =",+ red, "Blue =",+ blue);
                RobotTracks.ResetEcoders();
                m_autonomousStateMachineStep++;
                break;
            case 1: // STEP 1 - pause for a few seconds if needed

                // sleep the current thread for a while (number of seconds x 1000)
                if (m_startDelay > 0)
                {
                    try
                    {
                        // start delay is in seconds, times that by 1000 to get milliseconds
                        Thread.sleep(m_startDelay * 1000);
                    }
                    catch (InterruptedException ex)
                    {

                    }
                    m_autonomousStateMachineStep++;
                }
                else {
                    m_autonomousStateMachineStep++;
                }
                break;
            case 2:
                // backwards

                RobotTracks.SetDirectionForward();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, -0.2f);

                // see if we have run to the correct position
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(-76))) &&
                        (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(-76))))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            // removed reset
            case 3:
                //Arc Turn towards beacon, back left for blue, accumulate encoder counts

                RobotTracks.SetDirectionForward();
                RobotTracks.LeftTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, -0.3f);
                RobotTracks.RightTrack.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, 0.0f);

                // see if we have run to the correct position
                if (RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(-93)))
                {
                    m_autonomousStateMachineStep++;
                }
                break;
            // removed wait for stop, want to carry momentum since moving slow
            case 4:
                // back towards beacon, check encoder, distance, light to stop

                RobotTracks.SetDirectionForward();
                RobotTracks.SetRunMode(DcMotorController.RunMode.RUN_USING_ENCODERS, true, 0, -0.2f);

                // see if we have run to the correct position, blue > 300, red > 300, dist <= 4.5", dist >= 1.5"
                if ((RobotTracks.LeftTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(-120))) &&
                        (RobotTracks.RightTrack.HasRunToPosition(RobotUtil.GetEncoderCountForStraightDistance(-103))) ||
                        (((mDistance.getLightDetected() <= 1 && mDistance.getLightDetected() >= 0.2))
                                //in dump position
                        ))
                {
                    RobotTracks.Stop();
                    m_autonomousStateMachineStep++;
                }
                break;
            case 5:
                // Dump if in the correct postion
                if  ((mDistance.getLightDetected() <= 1 && mDistance.getLightDetected() >= 0.02))
                {
                    //climberDump.DumpDown
                    //m_telemetry.addData("Robot:Robot()", "Safe to Dump", "Red =", m_colorSensor.red(), "Blue =",m_colorSensor.blue());
                    RobotWinch.DumpUp(true);


                    m_autonomousStateMachineStep++;
                }
                else
                {
                    //do nothing, not in a safew spot to drop climbers
                    //m_telemetry.addData("Robot:Robot()", "Not Safe to Dump", "Red =", m_colorSensor.red(), "Blue =",m_colorSensor.blue());

                    m_autonomousStateMachineStep++;
                }

                break;
            default:
                // should never reach this code until the mode is done
                RobotColorPaddle.SetPaddleCenter();
                RobotColorPaddle.Stop();
                RobotWings.WingsUp();
                RobotWings.Stop();
                RobotWinch.ResetTapeEncoder();
                RobotWinch.StopTape();
                RobotWinch.ResetWinchEncoder();
                RobotWinch.StopWinch();
                RobotTracks.ResetEcoders();
                RobotTracks.Stop();
                m_autonomousStateMachineComplete = true;
                break;
        }
        return  !m_autonomousStateMachineComplete;
    }
}
