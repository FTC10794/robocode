package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.ColorSensor;
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


@TeleOp(name="JacksAuto", group="Autonomous")  // @Autonomous(...) is the other common choice
//@Disabled
public class jacksTestAuto extends OpMode {
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
        //causes the servo to over heat
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
        robot.servoLeftPaddle.setPosition(1);
        robot.servoRightPaddle.setPosition(0);
        //pause
        sleep(1000);
        //set motor lift up
        robot.motorLift.setPower(-0.50);
        sleep(1000);
        robot.servoJewelArm.setPosition(.85);
        sleep(1000);
        robot.servoJewelArm.setPosition(0);
        sleep(1000);
        robot.motorFrontLeft.setPower(.60);
        robot.motorBackLeft.setPower(.60);
        robot.motorFrontRight.setPower(-0.60);
        robot.motorBackRight.setPower(-0.60);
        sleep(800);
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);
        sleep(200);
        robot.motorFrontLeft.setPower(-0.40);
        robot.motorBackLeft.setPower(-0.40);
        robot.motorFrontRight.setPower(-0.40);
        robot.motorBackRight.setPower(-0.40);
        sleep(250);
        robot.motorLift.setPower(0);
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);
        /*
        sleep(200);
        robot.motorFrontLeft.setPower(.60);
        robot.motorBackLeft.setPower(.60);
        robot.motorFrontRight.setPower(-0.60);
        robot.motorBackRight.setPower(-0.60);
        sleep(750);
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);
        sleep(50);



        sleep(1000);
        robot.motorLift.setPower(-0.50);
        sleep(1500);
        robot.motorFrontLeft.setPower(.50);
        robot.motorBackLeft.setPower(-.50);
        robot.motorFrontRight.setPower(-.50);
        robot.motorBackRight.setPower(.50);
        sleep(300);
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);
        sleep(100);
        */
        runtime.reset();
    }

    /*
     * Assigns the gamepad buttons to the motor and servos
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());


    }

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