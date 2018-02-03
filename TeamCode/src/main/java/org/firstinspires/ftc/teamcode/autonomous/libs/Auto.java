package org.firstinspires.ftc.teamcode.autonomous.libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.libs.Robot;
import org.firstinspires.ftc.teamcode.libs.RobotHardwareValues;

public abstract class Auto extends LinearOpMode {
    // Class Variables
    public Robot robot;

    private AllianceColor allianceColor = AllianceColor.UNKNOWN;

    // Function Variables
    private ElapsedTime     runtime                 = new ElapsedTime();


    private static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make
    // it with an integer gyro
    private static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive,
    // but also less stable

    // Vuforia Variables
    private VuforiaLocalizer vuforia;
    private RelicRecoveryVuMark vuMark;

    /**
     * Constructor
     */
    public Auto() {

    }

    protected void instantiate() {
        robot = new Robot(hardwareMap);
        runtime.reset();
    }

    /**
     * This function should be overwritten in all subclasses.
     * @throws InterruptedException Interrupted
     */
    @Override
    public abstract void runOpMode() throws InterruptedException;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Modular Functions
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Initialize Motor and Servo Positions
     * .5 seconds
     */
    protected void initialize() {
        telemetry.addData(">", "Initialized");
        telemetry.update();

        //servo motors initial positions
        robot.servoLeftPaddle.setPosition(RobotHardwareValues.servoLeftPaddleInit);
        robot.servoRightPaddle.setPosition(RobotHardwareValues.servoRightPaddleInit);
        robot.servoRotator.setPosition(RobotHardwareValues.servoRotatorCenter);
        robot.servoSlide.setPosition(RobotHardwareValues.servoSlideOneBlock);

//        robot.servoRelicArm.setPosition(RobotHardwareValues.servoRelicArmInit);
//        robot.servoRelicClaw.setPosition(RobotHardWareValues.servoRelicClawClosed);

        robot.servoJewelArm.setPosition(RobotHardwareValues.servoJewelArmUp);
        robot.servoJewelPutter.setPosition(RobotHardwareValues.servoJewelPutterCenter);

        robot.servoJackClaw.setPosition(RobotHardwareValues.servoJackClawInit);
        robot.motorDeposit.setPower(.15);

        sleep(500);
    }

    /**
     * Detect Alliance Color based off of platform color
     * @param numTries number of retries to use
     * 1 second
     */
    protected void detectAlliance(int numTries) {
        telemetry.addData(">", "Detecting Platform Color");
        if (robot.sensorLine.blue() > robot.sensorLine.red()) {
            allianceColor = AllianceColor.BLUE;
            telemetry.addData(">>", "Alliance Color Blue");
        } else if (robot.sensorLine.red() > robot.sensorLine.blue()) {
            allianceColor = AllianceColor.RED;
            telemetry.addData(">>", "Alliance Color Red");
        } else {
            if (numTries < 3) {
                detectAlliance(numTries++);
            }
        }
        sleep(150);
    }

    /**
     * Open and lift to clear
     */
    protected void blockPickupClear() {
        telemetry.addData(">", "Picking Up Block");
        telemetry.update();

        //set motor lift up
        robot.motorLift.setPower(-0.3);
        sleep(1000);

        //set motor lift down
        robot.motorLift.setPower(0);
        sleep(1000);
    }

    /**
     * Detect Jewel and remove from platform
     */
    protected void jewelDetection() {
        telemetry.addData(">", "Jewel Detection");
        telemetry.update();

        final int numTries = 3;

        //bring down the arm
        robot.servoJewelArm.setPosition(RobotHardwareValues.servoJewelArmDown);
        sleep(1000);

        // color sensor
        for (int i = 0; i < numTries; i++) {
            if (robot.sensorColor.blue() > robot.sensorColor.red()) {
                telemetry.addData(">> Color: ", "Blue");
                telemetry.update();

                if (allianceColor == AllianceColor.BLUE) {
                    robot.servoJewelPutter.setPosition(RobotHardwareValues.servoJewelPutterKickBackward);
                }
                if (allianceColor == AllianceColor.RED) {
                    robot.servoJewelPutter.setPosition(RobotHardwareValues.servoJewelPutterKickForward);
                }
                break;
            } else if (robot.sensorColor.red() > robot.sensorColor.blue()) {
                telemetry.addData(">> Color:", "Red");
                telemetry.update();

                if (allianceColor == AllianceColor.BLUE) {
                    robot.servoJewelPutter.setPosition(RobotHardwareValues.servoJewelPutterKickForward);
                }
                if (allianceColor == AllianceColor.RED) {
                    robot.servoJewelPutter.setPosition(RobotHardwareValues.servoJewelPutterKickBackward);
                }
                break;
            } else {
                sleep(500);
            }
        }
        sleep(500);
        // Restore position
        robot.servoJewelArm.setPosition(RobotHardwareValues.servoJewelArmCenter);
        sleep(250);
        robot.servoJewelPutter.setPosition(RobotHardwareValues.servoJewelPutterCenter);
        sleep(1000);
    }

    /**
     * Detect the Cryptobox
     */
    protected void detectPictograph() {
        telemetry.addData(">", "Initializing Vuforia");
        telemetry.update();

        // Set up camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Set parameters
        parameters.vuforiaLicenseKey = "AXh5WtX/////AAAAmSIZEeuzqUEUv7PT36AAH2VEOgEe3MPEoQzgGVFPetvXT/1xZSd7D0UTfabEvWunLFqDqZUA10XsGkpUisGD4SQoAU1Z0ccpr5zTyEoTkU0kRTVJfTZn73UTpqwrCmkfN+O0/8fPZxaz540oZw1ACSeBgJ6pOrC71tOGelpYLZ4th81YOde7hzk/TvLXqVSfXyRqtWcGyOMKMWBf4/HCepiD5Bix1GEgJ2HgzfbRmJ/9xmxXja0AyhAWerEEgtmpim+TTJTDdjt75vb4OtJQUUjnCNQROU0E6RLT9A9ECWT2g1EHSj61g6zQGFBa7dmJ51t5OWAEr69dBXAqW++U0aE4QpGvJ9W4stvYXu7Q8icX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        // Trackables
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();

        telemetry.addData(">", "Vuforia Initialized");
        telemetry.update();

        sleep(200);
        ElapsedTime hold = new ElapsedTime();

        // keep looping while we have time remaining.
        hold.reset();
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while (vuMark == RelicRecoveryVuMark.UNKNOWN && hold.time() < 5) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }

        telemetry.addData(">", "View Mark %s found", vuMark);
        telemetry.update();
        sleep(500);
    }

    /**
     * Drive and Park
     */
    protected void driveOffTo() {
        telemetry.addData(">", "Driving off Platform");
        if(allianceColor == AllianceColor.BLUE) {
            //drive left
            telemetry.addData(">> Alliance Color Blue", "Drive Left");
            drive(90, 2.75);
            telemetry.addData(">>", "Stable");
        } else if (allianceColor == AllianceColor.RED) {
            //drive right
            telemetry.addData(">> Alliance Color Red", "Drive Right");
            drive(270, 2.75);
            telemetry.addData(">>", "Stable");
        }
        sleep(1000);
    }

    /**
     * Adjust the heading and straighten out the robot
     */
    protected void straighten(double dir, double expectedHeading) {
//        adjustHeading(expectedHeading);
        drive(180, .25, .25);
        if(allianceColor == AllianceColor.BLUE) {
            //drive left
            telemetry.addData(">> Alliance Color Blue", "Straighten Left");
            pointTurnHold(-dir, .5);
        } else if (allianceColor == AllianceColor.RED) {
            //drive right
            telemetry.addData(">> Alliance Color Red", "Straighten Right");
            pointTurnHold(dir, .5);
        }
    }


    /**
     * Drive to locker from corner (back)
     * Gray: Clear > 65000 Red > 43000 Green > 48000 Blue > 46000 Hue 140
     * Blue: Clear > 65000 Red > 40000 Green > 53000 Blue > 65000 Hue 203
     * Red:  Clear > 65000 Red > 60000 Green > 41000 Blue > 40000 Hue 2
     */
    protected void driveToCornerLocker() {
        telemetry.addData(">", "Driving to Corner Locker");
        telemetry.addData(">>", allianceColor);
//        float hsvValues[] = {0F, 0F, 0F};
//        while (robot.sensorRange.getDistance(DistanceUnit.INCH) < 23) {
//            telemetry.addData(">>>Current", robot.sensorRange.getDistance(DistanceUnit.INCH));
//            driveWOHold(180, .5);
//        }
//        stopMotors();
//
//        telemetry.addData(">>", "Drive until color");
//        if (allianceColor == AllianceColor.BLUE) {
//            //drive left
//            Color.RGBToHSV((robot.sensorLine.red() * 255) / 800, (robot.sensorLine.green() * 255) / 800,
//                    (robot.sensorLine.blue() * 255) / 800, hsvValues);
//            while (robot.sensorLine.blue() < 55000 || hsvValues[0] >= 199) {
//                adjustDistance(5);
//                driveWOHold(90,  .5);
//            }
//        } else
//        if (allianceColor == AllianceColor.RED) {
//            //drive right
//            Color.RGBToHSV((robot.sensorLine.red() * 255) / 800, (robot.sensorLine.green() * 255) / 800,
//                    (robot.sensorLine.blue() * 255) / 800, hsvValues);
//            while (robot.sensorLine.red() > 55000 || hsvValues[0] < 50) {
//                adjustDistance(5);
//                driveWOHold(270,  .5);
//            }
//        }
//        stopMotors();
        determineColumn();
    }

    /**
     * Drive to locker from side (front)
     */
    protected void driveToSideLocker() {
        determineColumn();
    }

    /**
     * Deposit block using BDS
     */
    protected void deposit() {
        telemetry.addData(">> Depositing: ", "Drive Forward");
        drive(180, .5, .3);
        sleep(500);
        telemetry.addData(">> Depositing: ", "Dump");
        robot.motorDeposit.setPower(-.2);
        sleep(250);
        telemetry.addData(">> Depositing: ", "Drop");
        robot.motorDeposit.setPower(0);
        robot.servoJackClaw.setPosition(RobotHardwareValues.servoJackClawScore);
        sleep(1500);
    }

    /**
     * Parks Robot
     */
    protected void park() {
        telemetry.addData(">> Parking: ", "Drive Forward/Backwards");
        drive(0, .75, .5);
        sleep(500);
        drive(180, .75, .5);
        sleep(500);
        drive(0,.75, .5);
        sleep(500);
        drive(180, .25, .5);
        stopMotors();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Module Helpers Functions
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Determine which column to travel to based on vuMark
     */
    private void determineColumn() {
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            determineLeft();
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            determineCenter();
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            determineRight();
        }
        else {
            determineCenter();
        }
    }
    /**
     * Determine location of left column
     */
    private void determineLeft() {
        if(allianceColor == AllianceColor.BLUE) {
            //drive left
            drive(90, 1);
            telemetry.addData("> Straightening", "");
            straighten(.25, .06);
            drive(90, .95);
        } else if (allianceColor == AllianceColor.RED) {
            //drive right
            telemetry.addData(">> Alliance Color Red", "Drive Right");
            drive(270, 1.5);
            telemetry.addData("> Straightening", "");
            straighten(.25, .07);
            drive(270, 2.35);
        }
        sleep(1000);
    }

    /**
     * Determine location of center column
     */
    private void determineCenter() {
        if(allianceColor == AllianceColor.BLUE) {
            //drive left
            telemetry.addData(">> Alliance Color Blue", "Drive Left");
            drive(90, 1.5);
            telemetry.addData("> Straightening", "");
            straighten(.25, .06);
            drive(90, 1.05);
        } else if (allianceColor == AllianceColor.RED) {
            //drive right
            telemetry.addData(">> Alliance Color Red", "Drive Right");
            drive(270, 1.5);
            telemetry.addData("> Straightening", "");
            straighten(.25, .06);
            drive(270, 1.75);
        }
        sleep(1000);
    }

    /**
     * Determine location of right column
     */
    private void determineRight() {
        if(allianceColor == AllianceColor.BLUE) {
            //drive left
            telemetry.addData(">> Alliance Color Blue", "Drive Left");
            drive(90, 1.5);
            telemetry.addData("> Straightening", "");
            straighten(.25, .06);
            drive(90, 1.4);
        } else if (allianceColor == AllianceColor.RED) {
            //drive right
            telemetry.addData(">> Alliance Color Red", "Drive Right");
            drive(270, 1.5);
            telemetry.addData("> Straightening", "");
            straighten(.25, .06);
            drive(270, 1.1);
        }
        sleep(1000);
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
     */
    public void driveWOHold(double dir) {
        double[] motorSpeed = holonomicAuto(1, dir, 0);
        holonomicDrive(motorSpeed);
    }

    /**
     * Drive "Straight"
     * 180 deg: Front
     * 0 deg: Back
     * 90 deg: Left
     * 270 deg: Right
     * @param dir direction of travel
     * @param speed speed of travel
     *
     */
    public void driveWOHold(double dir, double speed) {
        double[] motorSpeed = holonomicAuto(speed, dir, 0);
        holonomicDrive(motorSpeed);
    }

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

    /**
     * Drive "Straight" at a variable speed
     * 180 deg: Front
     * 0 deg: Back
     * 90 deg: Left
     * 270 deg: Right
     * @param dir direction of travel
     * @param holdTime time to hold
     * @param speed speed to go
     */
    public void drive(double dir, double holdTime, double speed) {
        double[] motorSpeed = holonomicAuto(speed, dir, 0);
        holonomicHold(motorSpeed, holdTime);
    }

    /**
     * Point turns
     * Left turn, left wheels turn back, right wheels turn forward
     * Right turn, right wheels turn back, left wheels turn forward
     * @param dir -1 or 1, direction of turn
     */
    private void pointTurn(double dir) {
        robot.motorFrontLeft.setPower(dir);
        robot.motorBackLeft.setPower(dir);
        robot.motorFrontRight.setPower(-dir);
        robot.motorBackRight.setPower(-dir);
    }

    private void pointTurnHold(double dir, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            robot.motorFrontLeft.setPower(dir);
            robot.motorBackLeft.setPower(dir);
            robot.motorFrontRight.setPower(-dir);
            robot.motorBackRight.setPower(-dir);
        }
        stopMotors();
    }

    /**
     * Turn the robot
     * @param angle angle to turn at
     */

    public void turn(double angle) {
        /*
        // Calibrate the gyroscope
        robot.sensorGyro.calibrate();
        while(robot.sensorGyro.isCalibrating()) {
            telemetry.addData("> Calibrating:", "Gyro");
            telemetry.update();
            sleep(5);
        }

        double heading = robot.sensorGyro.getHeading();
        if (angle == heading || angle == 360) {
            stopMotors();
            return;
        }

        if (heading > angle && angle < 0) {
            while (robot.sensorGyro.getHeading() <= Math.abs(angle)) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(-1);
            }
            stopMotors();
        } else  if (heading > angle && angle > 0) {
            while (robot.sensorGyro.getHeading() >= angle) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(1);
            }
            stopMotors();
        } else if (heading < angle && angle < 0) {
            while (robot.sensorGyro.getHeading() >= Math.abs(angle)) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(-1);
            }
            stopMotors();
        } else if (heading < angle && angle > 0) {
            while (robot.sensorGyro.getHeading() <= angle) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(1);
            }
            stopMotors();
        } else {
            // catch all case
            stopMotors();
        }
        */
    }

    protected void turnLeft(double angle) {
        double heading = robot.sensorGyro.getHeading();
        robot.sensorGyro.calibrate();
        while(robot.sensorGyro.isCalibrating()) {
            telemetry.addData("> Calibrating:", "Gyro");
            telemetry.update();
            sleep(5);
        }
        if (angle == heading || angle == 360) {
            stopMotors();
            return;
        }
        if (heading > angle) {
            while (robot.sensorGyro.getHeading() >= angle) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(-1);
            }
            stopMotors();
        } else if (heading < angle) {
            while (robot.sensorGyro.getHeading() >= 0) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(-1);
            }
            heading = 360;
            while (heading >= angle) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(-1);
                heading = robot.sensorGyro.getHeading();
            }
            stopMotors();
        } else {
            stopMotors();
        }
    }

    protected void turnRight(double angle) {
        double heading = robot.sensorGyro.getHeading();
        robot.sensorGyro.calibrate();
        while(robot.sensorGyro.isCalibrating()) {
            telemetry.addData("> Calibrating:", "Gyro");
            telemetry.update();
            sleep(5);
        }
        if (angle == heading || angle == 360) {
            stopMotors();
            return;
        }
        if (heading < angle) {
            while (robot.sensorGyro.getHeading() <= angle) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(1);
            }
            stopMotors();
        } else if (heading > angle) {
            while (robot.sensorGyro.getHeading() <= 358) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(1);
            }
            heading = 0;
            while (heading <= angle) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(1);
                heading = robot.sensorGyro.getHeading();
            }
            stopMotors();
        } else {
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
    private double[] holonomicAuto(double speed, double angle, double dirSpeed) {
        double[] motorSpeeds = {0, 0, 0, 0};

        double heading = Math.toRadians(angle) + ((Math.PI)/4);
        motorSpeeds[0] = Range.clip(speed * Math.sin(heading) + dirSpeed, -1, 1);
        motorSpeeds[1] = Range.clip(speed * Math.cos(heading) - dirSpeed, -1, 1);
        motorSpeeds[2] = Range.clip(speed * Math.cos(heading) + dirSpeed, -1, 1);
        motorSpeeds[3] = Range.clip(speed * Math.sin(heading) - dirSpeed, -1, 1);

        return motorSpeeds;
    }

    /**
     * Drive
     * @param speed motor speeds
     */
    private void holonomicDrive(double[] speed) {
        robot.motorFrontLeft.setPower(speed[0]);
        robot.motorFrontRight.setPower(speed[1]);
        robot.motorBackLeft.setPower(speed[2]);
        robot.motorBackRight.setPower(speed[3]);
    }

    /**
     * Hold speed
     * @param speed motor speeds
     * @param holdTime time to hold
     */
    private void holonomicHold(double[] speed, double holdTime) {
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
    private void stopMotors() {
        // Stop all motion;
        telemetry.addData("Stop", "Motors");
        robot.motorFrontLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorBackRight.setPower(0);
    }

    /**
     * Adjust distance of robot when traveling constant distance
     * @param expectedDistance expected distance
     */
    public void adjustDistance(double expectedDistance) {
        double actualDistance = robot.sensorRange.getDistance(DistanceUnit.INCH);
        if (actualDistance > expectedDistance + 1.5) {
            drive(0, .5,.25);
        } else if (actualDistance < expectedDistance - 1.5) {
            drive(180, .5, .25);
        }
    }

    /**
     * Adjust heading of robot to expected angle
     * @param expectedHeading expected angle
     */
    public void adjustHeading(double expectedHeading) {
        double actualHeading = robot.sensorGyro.getHeading();
        telemetry.addData(">> Heading: ", actualHeading);
        if (actualHeading > expectedHeading) {
            telemetry.addData(">> Turning: ", "Left");
            turnLeft(expectedHeading);
        } else if (actualHeading < expectedHeading) {
            telemetry.addData(">> Turning: ", "Right");
            turnRight(expectedHeading);
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Helper Control Functions
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////


    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.motorFrontLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorBackRight.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return onTarget
     */
    private boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.motorFrontLeft.setPower(leftSpeed);
        robot.motorFrontRight.setPower(rightSpeed);
        robot.motorBackLeft.setPower(leftSpeed);
        robot.motorBackRight.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.sensorGyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return steer
     */
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
