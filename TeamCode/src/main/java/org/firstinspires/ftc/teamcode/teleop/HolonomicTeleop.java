package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.libs.MotorFunctions;
import org.firstinspires.ftc.teamcode.libs.Robot;
import org.firstinspires.ftc.teamcode.libs.RobotHardwareValues;


@TeleOp(name="HoloDrive", group="Teleop")  // @Auto(...) is the other common choice
//@Disabled
public class HolonomicTeleop extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private Robot robot;
    private MotorFunctions motorFunctions;
    private RobotHardwareValues hardwareValues;

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
        robot.servoLeftPaddle.setPosition(hardwareValues.servoLeftPaddleInit);
        robot.servoRightPaddle.setPosition(hardwareValues.servoRightPaddleInit);
        robot.servoRotator.setPosition(hardwareValues.servoRotatorCenter);
        robot.servoSlide.setPosition(hardwareValues.servoSlideOneBlock);

//        robot.servoRelicArm.setPosition(hardwareValues.servoRelicArmInit);
//        robot.servoRelicClaw.setPosition(hardwareValues.servoRelicClawOpen);

        robot.servoJewelArm.setPosition(hardwareValues.servoJewelArmUp);
        robot.servoJewelPutter.setPosition(hardwareValues.servoJewelPutterCenter);

        robot.servoLeftWheel.setPower(0);
        robot.servoRightWheel.setPower(0.1); //zero spins in
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

        robot.motorLift.setPower(Range.clip(gamepad2.left_stick_y, -0.75, 0.75));

        /**
         * Gamepad 1
         */
        if (gamepad1.a) {
            // Relic Arm Down
//            robot.servoRelicArm.setPosition(hardwareValues.servoRelicArmDown);
        }
        if (gamepad1.x) {
            // Relic Claw Open
//            robot.servoRelicClaw.setPosition(hardwareValues.servoRelicClawOpen);
        }
        if (gamepad1.b) {
            // Relic Claw Out
//            robot.servoRelicClaw.setPosition(hardwareValues.servoRelicClawClosed);
        }
        if (gamepad1.y) {
            // Relic Arm Up
//            robot.servoRelicArm.setPosition(hardwareValues.servoRelicArmUp);
        }
        if (gamepad1.dpad_up) {
            // Jewel Arm Up
            robot.servoJewelArm.setPosition(hardwareValues.servoJewelArmUp);
        }
        if (gamepad1.dpad_down) {
            // not in use
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
        }
        if (gamepad1.right_bumper) {
            // not in use
        }
        if (gamepad1.left_trigger > 0.25) {
            // Relic Slide .5
            robot.motorSlide.setPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > 0.25) {
            // Relic Slide
            robot.motorSlide.setPower(-gamepad1.right_trigger);
        } else {
            robot.motorSlide.setPower(0);
        }

        /**
         * Gamepad 2
         */

        if (gamepad2.a) {
            // flappers out
            robot.servoLeftPaddle.setPosition(hardwareValues.servoLeftPaddleOut);
            robot.servoRightPaddle.setPosition(hardwareValues.servoRightPaddleOut);

            // set continuous rotation servo to stop
            robot.servoLeftWheel.setPower(hardwareValues.servoLeftWheelOff);
            robot.servoRightWheel.setPower(hardwareValues.servoRightWheelOff);
        }
        if (gamepad2.y) {
            //center rotator
            robot.servoRotator.setPosition(hardwareValues.servoRotatorCenter);
        }
        if (gamepad2.x) {
            // paddles loosen
            robot.servoLeftPaddle.setPosition(hardwareValues.servoLeftPaddleScoring);
            robot.servoRightPaddle.setPosition(hardwareValues.servoRightPaddleScoring);

            // servos kick out
            robot.servoLeftWheel.setPower(hardwareValues.servoLeftWheelOut);
            robot.servoRightWheel.setPower(hardwareValues.servoRightWheelOut);
        }
        if (gamepad2.b) {
            // servos pull in
            robot.servoLeftWheel.setPower(hardwareValues.servoLeftWheelIn);
            robot.servoRightWheel.setPower(hardwareValues.servoRightWheelIn);

            // flapper in
            robot.servoLeftPaddle.setPosition(hardwareValues.servoLeftPaddleIn);
            robot.servoRightPaddle.setPosition(hardwareValues.servoRightPaddleIn);
        }
        if (gamepad2.left_bumper) {
            // turn left
            robot.servoRotator.setPosition(hardwareValues.servoRotatorLeft);
        }
        if (gamepad2.right_bumper) {
            // turn right
            robot.servoRotator.setPosition(hardwareValues.servoRotatorRight);
        }
        if (gamepad2.left_trigger > 0.1) {
            // not in use
        }
        if (gamepad2.right_trigger > 0.1) {
            // not in use
        }
        if (gamepad2.dpad_up) {
            // two block position
            robot.servoSlide.setPosition(hardwareValues.servoSlideTwoBlock);
        }
        if (gamepad2.dpad_down) {
            // one block position
            robot.servoSlide.setPosition(hardwareValues.servoSlideOneBlock);
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