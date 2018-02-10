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
        robot.servoLeftPaddle.setPosition(RobotHardwareValues.servoLeftPaddleInit);
        robot.servoRightPaddle.setPosition(RobotHardwareValues.servoRightPaddleInit);
        robot.servoRotator.setPosition(RobotHardwareValues.servoRotatorCenter);
        robot.servoSlide.setPosition(RobotHardwareValues.servoSlideOneBlock);
        robot.servoLeftWheel.setPower(RobotHardwareValues.servoLeftWheelOff);
        robot.servoRightWheel.setPower(RobotHardwareValues.servoRightWheelOff);

//        robot.servoRelicArm.setPosition(RobotHardwareValues.servoRelicArmInit);
//        robot.servoRelicClaw.setPosition(RobotHardwareValues.servoRelicClawOpen);

        robot.servoJewelArm.setPosition(RobotHardwareValues.servoJewelArmUp);
        robot.servoJewelPutter.setPosition(RobotHardwareValues.servoJewelPutterCenter);

        robot.servoLeftWheel.setPower(0);
        robot.servoRightWheel.setPower(0.1); //zero spins in

        robot.servoJackClaw.setPosition(RobotHardwareValues.servoJackClawInit);
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

        robot.motorLift.setPower(Range.clip(gamepad2.left_stick_y, -1, 1));
        robot.servoLeftWheel.setPower(Range.clip(gamepad2.right_stick_y, -1, 1));
        robot.servoRightWheel.setPower(-1 * Range.clip(gamepad2.right_stick_y + 0.15, -1, 1));

        /**
         * Gamepad 1
         */
        if (gamepad1.a) {
            // Relic Arm Down
//            robot.servoRelicArm.setPosition(RobotHardwareValues.servoRelicArmDown);
        }
        if (gamepad1.x) {
            // Relic Claw Open
//            robot.servoRelicClaw.setPosition(RobotHardwareValues.servoRelicClawOpen);
        }
        if (gamepad1.b) {
            // Relic Claw Out
//            robot.servoRelicClaw.setPosition(RobotHardwareValues.servoRelicClawClosed);
        }
        if (gamepad1.y) {
            // Relic Arm Up
//            robot.servoRelicArm.setPosition(RobotHardwareValues.servoRelicArmUp);
        }
        if (gamepad1.dpad_up) {
            // Jewel Arm Up
            robot.servoJewelArm.setPosition(RobotHardwareValues.servoJewelArmUp);
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
            // Paddles In
            // servos pull in
//            robot.servoLeftWheel.setPower(RobotHardwareValues.servoLeftWheelGentle); //this is wrong
//            robot.servoRightWheel.setPower(RobotHardwareValues.servoRightWheelGentle); //this is wrong

            // paddles in
            robot.servoLeftPaddle.setPosition(RobotHardwareValues.servoLeftPaddleIn);
            robot.servoRightPaddle.setPosition(RobotHardwareValues.servoRightPaddleIn);
        }
        if (gamepad2.y) {
            //Center Rotator
            robot.servoRotator.setPosition(RobotHardwareValues.servoRotatorCenter);
        }
        if (gamepad2.x) {
            // Paddles Loosen
            robot.servoLeftPaddle.setPosition(RobotHardwareValues.servoLeftPaddleScoring);
            robot.servoRightPaddle.setPosition(RobotHardwareValues.servoRightPaddleScoring);

            // servos kick out
//            robot.servoLeftWheel.setPower(RobotHardwareValues.servoLeftWheelOut);
//            robot.servoRightWheel.setPower(RobotHardwareValues.servoRightWheelOut);
        }
        if (gamepad2.b) {
            // Paddles Out
            robot.servoLeftPaddle.setPosition(RobotHardwareValues.servoLeftPaddleOut);
            robot.servoRightPaddle.setPosition(RobotHardwareValues.servoRightPaddleOut);

            // set continuous rotation servo to intake
//            robot.servoLeftWheel.setPower(RobotHardwareValues.servoLeftWheelIn); //this one is right
//            robot.servoRightWheel.setPower(RobotHardwareValues.servoRightWheelIn); //this one is wrong
        }
        if (gamepad2.left_bumper) {
            // turn left
            // robot.servoRotator.setPosition(RobotHardwareValues.servoRotatorLeft);
            robot.servoJackClaw.setPosition(0);
            robot.motorDeposit.setPower(-.5);
            sleep(100);
            robot.motorDeposit.setPower(0);
        }
        if (gamepad2.right_bumper) {
            // turn right
            robot.servoRotator.setPosition(RobotHardwareValues.servoRotatorRight);
        }
        if (gamepad2.left_trigger > 0.1) {
            // not in use
        }
        if (gamepad2.right_trigger > 0.1) {
            // not in use
        }
        if (gamepad2.dpad_up) {
            // two block position
            robot.servoSlide.setPosition(RobotHardwareValues.servoSlideTwoBlock);
        }
        if (gamepad2.dpad_down) {
            // one block position
            robot.servoSlide.setPosition(RobotHardwareValues.servoSlideOneBlock);
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