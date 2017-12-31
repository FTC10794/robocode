package org.firstinspires.ftc.teamcode.autonomous.y1718.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.libs.MotorFunctions;
import org.firstinspires.ftc.teamcode.libs.Robot;

@Autonomous(name="Corner Blue Auto", group="Blue Auto")
@Disabled
public class CornerBlueAuto extends LinearOpMode {
    private ElapsedTime     runtime                 = new ElapsedTime();

    private Robot           robot;
    private MotorFunctions  motorFunctions;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() throws InterruptedException {
        runtime.reset();
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
        driveToLocker();
        blockDeposit();
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

        //servo motors initial positions
        robot.servoLeftPaddle.setPosition(0);
        robot.servoRightPaddle.setPosition(1);
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

        //set paddles closed
        robot.servoLeftPaddle.setPosition(1);
        robot.servoRightPaddle.setPosition(0);

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
        final int numTries = 3;

        //bring down the arm
        robot.servoJewelArm.setPosition(.85);
        sleep(1000);

        // color sensor
        for (int i = 0; i < numTries; i++) {
            if (robot.sensorColor.blue() > robot.sensorColor.red()) {
                telemetry.addData(">> Color: ", "Blue");

                // drive forwards
                drive(270, .35);
                break;
            } else if (robot.sensorColor.blue() < robot.sensorColor.red()) {
                telemetry.addData(">> Color:", "Red");

                // drive backwards
                drive(90, .35);
                break;
            } else {
                sleep(500);
            }
        }
        sleep(500);
        drive(180, .5);
        robot.servoJewelArm.setPosition(0);
        sleep(1000);
    }

    public void driveToLocker() {
        telemetry.addData(">", "Driving to Locker");

        // drive to cryptolocker
        telemetry.addData(">>", "Drive Left");
        drive(90, 2);

        // turn 180 degrees to face cryptolocker
        telemetry.addData(">>", "Turn 180 deg");
        turn(180);

        // drive forward
        telemetry.addData(">>", "Drive Forward");
        drive(180, .5);
        sleep(500);
    }

    public void blockDeposit() {
        telemetry.addData(">", "Deposit Block");

        //set motor lift down
        robot.motorLift.setPower(-0.50);
        sleep(1000);

        //open paddles
        robot.servoLeftPaddle.setPosition(0);
        robot.servoRightPaddle.setPosition(1);
        sleep(500);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Control Functions
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Drive "Straight"
     * @param dir direction of travel
     * @param holdTime time to hold
     */
    public void drive(double dir, double holdTime) {
        double[] motorSpeed = holonomicAuto(1, dir, 0);
        holonomicHold(motorSpeed, holdTime);
    }

    /**
     *
     * @param dir -1 or 1, direction of turn
     */
    public void pointTurn(int dir) {
        robot.motorFrontLeft.setPower(dir);
        robot.motorBackLeft.setPower(dir);
        robot.motorFrontRight.setPower(dir);
        robot.motorBackRight.setPower(dir);
    }

    /**
     * Turn the robot
     * @param angle angle to turn at
     */
//    public void turn(double angle) {
//        double desiredHeading = angle < 0 ? 360 - angle : angle;
//        double currentHeading;
//
//        // Calibrate the gyroscope
//        robot.sensorGyro.calibrate();
//        while(robot.sensorGyro.isCalibrating()) {
//            telemetry.addData("> Calibrating:", "Gyro");
//            telemetry.update();
//            sleep(50);
//        }
//
//        currentHeading = robot.sensorGyro.getHeading();
//        if (currentHeading > desiredHeading) {
//            while (robot.sensorGyro.getHeading() < desiredHeading) {
//                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
//                telemetry.update();
//                pointTurn(-1); // turn left
//            }
//            stopMotors();
//        } else {
//            while (robot.sensorGyro.getHeading() < desiredHeading) {
//                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
//                telemetry.update();
//                pointTurn(1);
//            }
//            stopMotors();
//        }
//    }

    public void turn(double angle) {
        // Calibrate the gyroscope
        robot.sensorGyro.calibrate();
        while(robot.sensorGyro.isCalibrating()) {
            telemetry.addData("> Calibrating:", "Gyro");
            telemetry.update();
            sleep(50);
        }

        double heading = robot.sensorGyro.getHeading();
        if (angle == heading || angle == 360) {
            stopMotors();
            return;
        }

        if (heading > angle && angle < 0) {
            while (robot.sensorGyro.getHeading() > Math.abs(angle)) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(-1);
            }
        } else  if (heading > angle && angle > 0) {
            while (robot.sensorGyro.getHeading() > angle) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(1);
            }
        } else if (heading < angle && angle < 0) {
            while (robot.sensorGyro.getHeading() < Math.abs(angle)) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(-1);
            }
        } else if (heading < angle && angle > 0) {
            while (robot.sensorGyro.getHeading() < angle) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(1);
            }
        } else {
            // current angle or heading is 360
            stopMotors();
        }
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
