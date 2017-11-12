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

    private double posBlockSlide = 0,
            posRelicArm = 0;

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
        motorFunctions = new MotorFunctions(-1, 1, 0, 1, .01);

        //servo motors initial positions
        robot.servoLeftPaddle.setPosition(0);
        robot.servoRightPaddle.setPosition(1);
        robot.servoRotator.setPosition(0.0833);
        robot.servoSlide.setPosition(0);

        robot.servoRelicArm.setPosition(1);
        robot.servoRelicClaw.setPosition(0);

        robot.servoJewelArm.setPosition(0);
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

        robot.motorLift.setPower(Range.clip(gamepad2.right_stick_y, -0.55, 0.55));

        /**
         * Gamepad 1
         */
        if (gamepad1.a) {
            // Relic Arm Down
            robot.servoRelicArm.setPosition(0);
        }
        if (gamepad1.x) {
            // Relic Claw In
            robot.servoRelicClaw.setPosition(0);
        }
        if (gamepad1.b) {
            // Relic Claw Out
            robot.servoRelicClaw.setPosition(.5);
        }
        if (gamepad1.y) {
            // Relic Arm Up
            robot.servoRelicArm.setPosition(0.7);
        }
        if (gamepad1.dpad_up) {
            // Jewel Arm Up
            robot.servoJewelArm.setPosition(0);
        }
        if (gamepad1.dpad_down) {
            // Jewel Arm Down
            robot.servoJewelArm.setPosition(1);
        }
        if (gamepad1.dpad_left) {
            // Relic Arm Precision
            posRelicArm = MotorFunctions.servoIncrement(posRelicArm);
            robot.servoRelicArm.setPosition(posRelicArm);
        }
        if (gamepad1.dpad_right) {
            // Relic Arm Precision
            posRelicArm = MotorFunctions.servoDecrement(posRelicArm);
            robot.servoRelicArm.setPosition(posRelicArm);
        }
        if (gamepad1.left_bumper) {
            // not in use
            robot.motorSlide.setPower(0);
        }
        if (gamepad1.right_bumper) {
            // not in use
            robot.motorSlide.setPower(0);
        }
        if (gamepad1.left_trigger > 0.25) {
            // Relic Slide
            robot.motorSlide.setPower(.5);
        }
        if (gamepad1.right_trigger > 0.25) {
            // Relic Slide
            robot.motorSlide.setPower(-0.25);
        }

        /**
         * Gamepad 2
         */

        if (gamepad2.a) {
            // flapper in
            robot.servoLeftPaddle.setPosition(.5);
            robot.servoRightPaddle.setPosition(.5);
        }
        if (gamepad2.y) {
            //center rotator
            robot.servoRotator.setPosition(0.0833);
        }
        if (gamepad2.x) {
            // not in use
        }
        if (gamepad2.b) {
            // flapper out
            robot.servoLeftPaddle.setPosition(0);
            robot.servoRightPaddle.setPosition(1);
        }
        if (gamepad2.dpad_up) {
            // not in use (TEST)
//            posBlockSlide = MotorFunctions.servo(posBlockSlide, -0.001);
            robot.servoSlide.setPosition(.28);
        }
        if (gamepad2.dpad_down) {
            // not in use (TEST)
            robot.servoSlide.setPosition(0);
        }
        if (gamepad2.dpad_left) {
            // slide out
            posBlockSlide = MotorFunctions.servoIncrement(posBlockSlide);
            robot.servoSlide.setPosition(posBlockSlide);
        }
        if (gamepad2.dpad_right) {
            // slide out
            posBlockSlide = MotorFunctions.servoDecrement(posBlockSlide);
            robot.servoSlide.setPosition(posBlockSlide);
        }
        if (gamepad2.left_bumper) {
            // turn left
//            posBlockTurn = MotorFunctions.servo(posBlockTurn, .005, 0, 0.25);
            robot.servoRotator.setPosition(.16);
        }
        if (gamepad2.right_bumper) {
            // turn right
//            posBlockTurn = MotorFunctions.servo(posBlockTurn, -0.005, 0, 0.25);
            robot.servoRotator.setPosition(.005);
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