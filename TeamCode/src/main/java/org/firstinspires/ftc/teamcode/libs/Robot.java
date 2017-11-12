package org.firstinspires.ftc.teamcode.libs;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.LegacyModuleControllerConfiguration;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;

/**
 * Sets up hardware
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
    public DcMotor motorSlide, motorLift;

    // Servos
    public Servo servoRotator, servoSlide, servoLeftPaddle, servoRightPaddle;
    public Servo servoRelicArm, servoRelicClaw;
    public Servo servoJewelArm;

    // References to sensors
    public ColorSensor sensorColor;
    public ModernRoboticsI2cGyro sensorGyro;

    /**
     * Robot class constructor
     * Assigns the hardware references to the software variables
     * @param hwMap Reference to the hardware map
     */
    public Robot (HardwareMap hwMap) {
        hardwareMap = hwMap;

//        dim = hardwareMap.deviceInterfaceModule.get("dim");

        // Initialize Drive Train Motors
        motorFrontLeft = hardwareMap.dcMotor.get("front_left");
//        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight = hardwareMap.dcMotor.get("front_right");
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft = hardwareMap.dcMotor.get("back_left");
//        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight = hardwareMap.dcMotor.get("back_right");
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        // Initialize Actuator Motors
        motorLift = hardwareMap.dcMotor.get("lift");
//        motorLift.setDirection(DcMotor.Direction.REVERSE);
        motorSlide = hardwareMap.dcMotor.get("slide");

        // Initialize Servo Motors
        servoRotator = hardwareMap.servo.get("rotator");
        servoSlide = hardwareMap.servo.get("expander");
        servoLeftPaddle = hardwareMap.servo.get("left_paddle");
        servoRightPaddle = hardwareMap.servo.get("right_paddle");

        servoRelicArm = hardwareMap.servo.get("arm");
        servoRelicClaw = hardwareMap.servo.get("claw");

        servoJewelArm = hardwareMap.servo.get("jewel");

        // Initialize Sensors
        sensorColor = hardwareMap.colorSensor.get("color");
        sensorGyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
    }

    //any other hardware methods go here
}
