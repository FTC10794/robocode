package org.firstinspires.ftc.teamcode.libs;

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
    public DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    // References to sensors

    /**
     * Robot class constructor
     * Assigns the hardware references to the software variables
     * @param hwMap Reference to the hardware map
     */
    public Robot (HardwareMap hwMap) {
        hardwareMap = hwMap;

//        dim = hardwareMap.deviceInterfaceModule.get("dim");

        motorFrontLeft = hardwareMap.dcMotor.get("front_left");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight = hardwareMap.dcMotor.get("front_right");
//        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft = hardwareMap.dcMotor.get("back_left");
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight = hardwareMap.dcMotor.get("back_right");
//        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
    }

    //any other hardware methods go here
}
