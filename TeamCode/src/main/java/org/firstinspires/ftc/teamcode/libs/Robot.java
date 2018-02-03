package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Sets up hardware
 * Mapping:
 * * Servo Controller Relic:
 * * * Port 1: Relic Arm
 * * * Port 2: Relic Claw
 * * * Port 3: Jewel Arm
 * * * Port 4: Jewel Putter
 * * * Port 5: Not in use
 * * * Port 6: Not in use
 * * Servo Controller Block:
 * * * Port 1: Rotator
 * * * Port 2: Expander
 * * * Port 3: Left Paddle
 * * * Port 4: Right Paddle
 * * * Port 5: Left Wheel
 * * * Port 6: Right Wheel
 */
public class Robot {
    // Reference to the hardware map
    public HardwareMap hardwareMap;

    // Reference to the device interface module
    public DeviceInterfaceModule dim;

    // References to the motor controller

    // public LegacyModuleControllerConfiguration ctrlActuatorMotor;

    // References to the different motors and servos
    // DC Motors
    public DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    public DcMotor motorSlide, motorLift, motorDeposit;

    // Servos
    public Servo servoRotator, servoSlide;
    public Servo servoLeftPaddle, servoRightPaddle;
    public CRServo servoLeftWheel, servoRightWheel;

    public Servo servoRelicArm, servoRelicClaw;

    public Servo servoJewelArm, servoJewelPutter;

    public Servo servoJackClaw;

    // References to sensors
    public ColorSensor sensorColor, sensorLine;
    public ModernRoboticsI2cGyro sensorGyro;
    public ModernRoboticsI2cRangeSensor sensorRange;

    /**
     * Robot class constructor
     * Assigns the hardware references to the software variables
     * @param hwMap Reference to the hardware map
     */
    public Robot (HardwareMap hwMap) {
        hardwareMap = hwMap;

        // Device Interface Module
//        dim = hardwareMap.deviceInterfaceModule.get("dim");

        // Initialize Drive Train Motors
        motorFrontLeft = hardwareMap.dcMotor.get("front_left");
        //// motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight = hardwareMap.dcMotor.get("front_right");
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft = hardwareMap.dcMotor.get("back_left");
        //// motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight = hardwareMap.dcMotor.get("back_right");
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        // Initialize Actuator Motors
        motorLift = hardwareMap.dcMotor.get("lift");
        //// motorLift.setDirection(DcMotor.Direction.REVERSE);
        motorSlide = hardwareMap.dcMotor.get("slide");
        motorDeposit = hardwareMap.dcMotor.get("deposit");

        // Initialize Servo Motors
        servoRotator = hardwareMap.servo.get("rotator");
        servoSlide = hardwareMap.servo.get("expander");
        servoLeftPaddle = hardwareMap.servo.get("left_paddle");
        servoRightPaddle = hardwareMap.servo.get("right_paddle");
        servoLeftWheel = hardwareMap.crservo.get("left_wheel");
        servoRightWheel = hardwareMap.crservo.get("right_wheel");

//        servoRelicArm = hardwareMap.servo.get("arm");
//        servoRelicClaw = hardwareMap.servo.get("claw");

        servoJewelArm = hardwareMap.servo.get("jewel");
        servoJewelPutter = hardwareMap.servo.get("putter");

        servoJackClaw = hardwareMap.servo.get("jack");

        // Initialize Sensors
        sensorColor = hardwareMap.colorSensor.get("color");
        sensorLine = hardwareMap.colorSensor.get("color_line");
        sensorGyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        sensorRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
    }

    //any other hardware methods go here
}
