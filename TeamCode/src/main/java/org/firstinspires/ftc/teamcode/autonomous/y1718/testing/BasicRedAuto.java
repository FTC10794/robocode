package org.firstinspires.ftc.teamcode.autonomous.y1718.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.libs.MotorFunctions;
import org.firstinspires.ftc.teamcode.libs.Robot;

@Autonomous(name="Basic Red Auto", group="Red Auto")
@Disabled
public class BasicRedAuto extends LinearOpMode {
    boolean isActive;
    private ElapsedTime     runtime                 = new ElapsedTime();

    private Robot           robot;
    private MotorFunctions  motorFunctions;

    private double          posBlockSlide           = 0,
            posRelicArm             = 0;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Running Op Mode");

        /**
         * Initializes the library functions
         * Robot hardware and motor functions
         */
        robot = new Robot(hardwareMap);
        motorFunctions = new MotorFunctions(-1, 1, 0, 1, .01);

        //Wait For Auto to Start
        waitForStart();

        initialize();
        blockPickup();
        jewelDetection();

        sleep(2500);

    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Modular Functions
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Initialize Motor and Servo Positions
     */
    public void initialize() {
        telemetry.addData(">", "Initialized");
        telemetry.update();

        //servo motors initial positions
        robot.servoLeftPaddle.setPosition(.5);
        robot.servoRightPaddle.setPosition(.5);
        robot.servoRotator.setPosition(0.0833);
        robot.servoSlide.setPosition(0);

        robot.servoRelicArm.setPosition(1);
        robot.servoRelicClaw.setPosition(0);

        robot.servoJewelArm.setPosition(0);
        sleep(1000);
    }

    /**
     * Pick up the block (pre-loaded)
     */
    public void blockPickup() {
        telemetry.addData(">", "Picking Up Block");
        telemetry.update();

        //set paddles closed
        robot.servoLeftPaddle.setPosition(0);
        robot.servoRightPaddle.setPosition(1);

        //pause
        sleep(1000);

        //set motor lift up
        robot.motorLift.setPower(-0.50);
        sleep(1000);
    }

    /**
     * Detect Jewel and remove from platform
     */
    public void jewelDetection() {
        telemetry.addData(">", "Jewel Detection");
        telemetry.update();

        final int numTries = 3;

        //bring down the arm
        robot.servoJewelArm.setPosition(.85);
        sleep(1000);

        // color sensor
        for (int i = 0; i < numTries; i++) {
            if (robot.sensorColor.blue() < robot.sensorColor.red()) {
                telemetry.addData(">> Color: ", "Blue");
                telemetry.update();

                // drive right
                drive(270, .35);
                break;
            } else if (robot.sensorColor.blue() > robot.sensorColor.red()) {
                telemetry.addData(">> Color:", "Red");
                telemetry.update();

                // drive left
                drive(90, .35);
                break;
            } else {
                sleep(500);
            }
        }
        sleep(500);
        drive(180, .2);
        robot.servoJewelArm.setPosition(0);
        sleep(500);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Control Functions
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Drive "Straight"
     * 180 deg: Front
     * 0 deg: Back
     * 90 deg: Left
     * 270 deg: Right
     * @param dir direction of travel
     * @param holdTime time to hold
     */
    public void drive(double dir, double holdTime) {
        double[] motorSpeed = holonomicAuto(1, dir, 0);
        holonomicHold(motorSpeed, holdTime);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Helper Functions
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Method to drive the robot using the holonomic drive
     * @param speed Speed of the robot drive train motors
     * @param angle Angle of travel
     * @param dirSpeed Speed of directional change
     * @return double[] motor powers to assign
     */
    public double[] holonomicAuto(double speed, double angle, double dirSpeed) {
        double[] motorSpeeds = {0, 0, 0, 0};

        double heading = Math.toRadians(angle) + ((Math.PI)/4);
        motorSpeeds[0] = Range.clip(speed * Math.sin(heading) + dirSpeed, -1, 1);
        motorSpeeds[1] = Range.clip(speed * Math.cos(heading) - dirSpeed, -1, 1);
        motorSpeeds[2] = Range.clip(speed * Math.cos(heading) + dirSpeed, -1, 1);
        motorSpeeds[3] = Range.clip(speed * Math.sin(heading) - dirSpeed, -1, 1);

        return motorSpeeds;
    }

    /**
     * Hold speed
     * @param speed motor speeds
     * @param holdTime time to hold
     */
    public void holonomicHold(double[] speed, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            robot.motorFrontLeft.setPower(speed[0]);
            robot.motorFrontRight.setPower(speed[1]);
            robot.motorBackLeft.setPower(speed[2]);
            robot.motorBackRight.setPower(speed[3]);
            telemetry.update();
        }

        stopMotors();
    }

    /**
     * Stop all motion
     */
    public void stopMotors() {
        // Stop all motion;
        robot.motorFrontLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorBackRight.setPower(0);
    }

}
