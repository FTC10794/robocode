package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcontroller.libs.MotorFunctions;

import org.firstinspires.ftc.teamcode.libs.Robot;
import java.text.SimpleDateFormat;
import java.util.Date;


@TeleOp(name="HoloDrive", group="Teleop")  // @Autonomous(...) is the other common choice
//@Disabled
public class HolonomicTeleop extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private Robot robot;
    private MotorFunctions motorFunctions;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        /**
         * Initializes the library functions
         * Robot hardware and motor functions
         */
        robot = new Robot(hardwareMap);
        motorFunctions = new MotorFunctions();

        //servo wheels initial positions

    }

    /*
    * Code to run REPEATEDLY when the driver hits INIT
    * We don't need this method
    */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // servo initial positions

        runtime.reset();
    }

    /*
     * Assigns the gamepad buttons to the motor and servos
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        /**
         * Sets the power level of the drive motors to the joystick values
         */

        float[] motor_power = motorFunctions.holonomic(gamepad1.left_stick_x,
                gamepad1.left_stick_y, gamepad1.right_stick_x);

        robot.motorFrontLeft.setPower(motor_power[0]);
        robot.motorFrontRight.setPower(motor_power[1]);
        robot.motorBackLeft.setPower(motor_power[2]);
        robot.motorBackRight.setPower(motor_power[3]);

        /**
         * Gamepad 1
         */
        if (gamepad1.a) {
            //not in use
        }
        if (gamepad1.b) {
            //not in use
        }
        if (gamepad1.x) {
            //not in use
        }
        if (gamepad1.y) {
            // not in use
        }
        if (gamepad1.dpad_up) {
            // not in use
        }
        if (gamepad1.dpad_down) {
            //not in use
        }
        if (gamepad1.dpad_left) {
            //not in use
        }
        if (gamepad1.dpad_right) {
            //not in use
        }
        if (gamepad1.left_bumper) {
            // not in use
        }
        if (gamepad1.right_bumper) {
            // not in use
        }
        if (gamepad1.left_bumper) {
            // not in use
        }
        if (gamepad1.right_bumper) {
            // not in use
        }
        if (gamepad1.left_trigger > 0.25) {
            // not in use
        }
        if (gamepad1.right_trigger > 0.25) {
            // not in use
        }

        /**
         * Gamepad 2
         */

        if (gamepad2.a) {
            // not in use
        }
        if (gamepad2.y) {
            // not in use
        }
        if (gamepad2.x) {
            // not in use
        }
        if (gamepad2.b) {
            // not in use
        }
        if (gamepad2.dpad_up) {
            // not in use
        }
        if (gamepad2.dpad_down) {
            // not in use
        }
        if (gamepad2.dpad_left) {
            // not in use
        }
        if (gamepad2.dpad_right) {
            // not in use
        }
        if (gamepad2.left_bumper) {
            // not in use
        }
        if (gamepad2.right_bumper) {
            // not in use
        }
        if (gamepad2.left_trigger > 0.1) {
            // not in use
        }
        if (gamepad2.right_trigger > 0.1) {
            // not in use
        }
    }

    /**
     * Code to run ONCE after STOP is pressed
     * DO NOT USE THIS METHOD!!
     * Violation of game rules
     */
    @Override
    public void stop() {

    }


    // General Functions

    /**
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted. This is
     * simple shorthand for the operating-system-provided {@link Thread#sleep(long) sleep()} method.
     *
     * @param milliseconds amount of time to sleep, in milliseconds
     * @see Thread#sleep(long)
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
