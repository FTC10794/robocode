package org.firstinspires.ftc.teamcode.autonomous.y1718.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.libs.MotorFunctions;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.autonomous.libs.AllianceColor;
import org.firstinspires.ftc.teamcode.libs.Robot;
import org.firstinspires.ftc.teamcode.libs.RobotHardwareValues;

@Autonomous(name="Basic Blue Auto", group="Blue Auto")
@Disabled
public class BasicBlueAuto extends LinearOpMode {
    // Class Variables
    public Robot robot;
    public Telemetry telemetry;

    private MotorFunctions motorFunctions;
    private RobotHardwareValues hardwareValues;
    private AllianceColor allianceColor = AllianceColor.UNKNOWN;

    // Function Variables
    boolean isActive;
    private ElapsedTime runtime = new ElapsedTime();

    private double posBlockSlide = 0,
            posRelicArm = 0;

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable

    // Vuforia Variables
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    private RelicRecoveryVuMark vuMark;

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
        detectAlliance(0);
//        blockPickup();
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
        robot.servoLeftPaddle.setPosition(hardwareValues.servoLeftPaddleIn);
        robot.servoRightPaddle.setPosition(hardwareValues.servoRightPaddleIn);
        robot.servoRotator.setPosition(hardwareValues.servoRotatorCenter);
        robot.servoSlide.setPosition(hardwareValues.servoSlideOneBlock);

        robot.servoRelicArm.setPosition(hardwareValues.servoRelicArmInit);
        robot.servoRelicClaw.setPosition(hardwareValues.servoRelicClawClosed);

        robot.servoJewelArm.setPosition(hardwareValues.servoJewelArmUp);
        robot.servoJewelPutter.setPosition(hardwareValues.servoJewelPutterCenter);
        sleep(1000);
    }

    /**
     * Detect Alliance Color based off of platform color
     *
     * @param numTries number of retries to use
     */
    public void detectAlliance(int numTries) {
        if (robot.sensorLine.blue() > robot.sensorLine.red()) {
            allianceColor = AllianceColor.BLUE;
        } else if (robot.sensorLine.red() > robot.sensorLine.blue()) {
            allianceColor = AllianceColor.RED;
        } else {
            if (numTries < 3) {
                detectAlliance(numTries++);
            }
        }
        sleep(150);
    }

    /**
     * Pick up the block (pre-loaded)
     * FIXME for new paddles
     */
    public void blockPickup() {
        telemetry.addData(">", "Picking Up Block");
        telemetry.update();

        // set wheels to pull in
        robot.servoLeftWheel.setPower(hardwareValues.servoLeftWheelIn);
        robot.servoRightWheel.setPower(hardwareValues.servoRightWheelIn);

        //set paddles closed
        robot.servoLeftPaddle.setPosition(hardwareValues.servoLeftPaddleIn);
        robot.servoRightPaddle.setPosition(hardwareValues.servoRightPaddleIn);

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
        robot.servoJewelArm.setPosition(hardwareValues.servoJewelArmDown);
        sleep(1000);

        // color sensor
        for (int i = 0; i < numTries; i++) {
            if (robot.sensorColor.blue() > robot.sensorColor.red()) {
                telemetry.addData(">> Color: ", "Blue");
                telemetry.update();

                if (allianceColor == AllianceColor.BLUE) {
                    robot.servoJewelPutter.setPosition(hardwareValues.servoJewelPutterKickBackward);
                }
                if (allianceColor == AllianceColor.RED) {
                    robot.servoJewelPutter.setPosition(hardwareValues.servoJewelPutterKickForward);
                }
                break;
            } else if (robot.sensorColor.red() > robot.sensorColor.blue()) {
                telemetry.addData(">> Color:", "Red");
                telemetry.update();

                if (allianceColor == AllianceColor.BLUE) {
                    robot.servoJewelPutter.setPosition(hardwareValues.servoJewelPutterKickForward);
                }
                if (allianceColor == AllianceColor.RED) {
                    robot.servoJewelPutter.setPosition(hardwareValues.servoJewelPutterKickBackward);
                }
                break;
            } else {
                sleep(500);
            }
        }
        sleep(500);
        // Restore position
        robot.servoJewelArm.setPosition(hardwareValues.servoJewelArmUp);
        robot.servoJewelPutter.setPosition(hardwareValues.servoJewelPutterCenter);
        sleep(500);
    }

    public void detectPictograph() {
        telemetry.addData(">", "Initializing Vuforia");
        telemetry.update();

        // Set up camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Set parameters
        parameters.vuforiaLicenseKey = "AXh5WtX/////AAAAmSIZEeuzqUEUv7PT36AAH2VEOgEe3MPEoQzgGVFPetvXT/1xZSd7D0UTfabEvWunLFqDqZUA10XsGkpUisGD4SQoAU1Z0ccpr5zTyEoTkU0kRTVJfTZn73UTpqwrCmkfN+O0/8fPZxaz540oZw1ACSeBgJ6pOrC71tOGelpYLZ4th81YOde7hzk/TvLXqVSfXyRqtWcGyOMKMWBf4/HCepiD5Bix1GEgJ2HgzfbRmJ/9xmxXja0AyhAWerEEgtmpim+TTJTDdjt75vb4OtJQUUjnCNQROU0E6RLT9A9ECWT2g1EHSj61g6zQGFBa7dmJ51t5OWAEr69dBXAqW++U0aE4QpGvJ9W4stvYXu7Q8icX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        // Trackables
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();

        telemetry.addData(">", "Vuforia Initialized");
        telemetry.update();

        sleep(200);
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }

        telemetry.addData(">", "View Mark %s found", vuMark);
        telemetry.update();
        sleep(500);
    }

    public void driveToLocker() {

    }

    public void deposit() {
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            drive(90, 1);
        }
        if (vuMark == RelicRecoveryVuMark.CENTER) {
            drive(180, 1);
        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            drive(270, 1);
        }
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
     *
     * @param dir      direction of travel
     * @param holdTime time to hold
     */
    public void drive(double dir, double holdTime) {
        double[] motorSpeed = holonomicAuto(1, dir, 0);
        holonomicHold(motorSpeed, holdTime);
    }

    /**
     * @param dir -1 or 1, direction of turn
     */
    public void pointTurn(int dir) {
        robot.motorFrontLeft.setPower(dir);
        robot.motorBackLeft.setPower(dir);
        robot.motorFrontRight.setPower(-dir);
        robot.motorBackRight.setPower(-dir);
    }

    /**
     * Turn the robot
     *
     * @param angle angle to turn at
     */
    public void turn(double angle) {
        // Calibrate the gyroscope
        robot.sensorGyro.calibrate();
        while (robot.sensorGyro.isCalibrating()) {
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
            while (robot.sensorGyro.getHeading() <= Math.abs(angle)) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(-1);
            }
            stopMotors();
        } else if (heading > angle && angle > 0) {
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
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Helper Functions
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Method to drive the robot using the holonomic drive
     *
     * @param speed    Speed of the robot drive train motors
     * @param angle    Angle of travel
     * @param dirSpeed Speed of directional change
     * @return double[] motor powers to assign
     */
    public double[] holonomicAuto(double speed, double angle, double dirSpeed) {
        double[] motorSpeeds = {0, 0, 0, 0};

        double heading = Math.toRadians(angle) + ((Math.PI) / 4);
        motorSpeeds[0] = Range.clip(speed * Math.sin(heading) + dirSpeed, -1, 1);
        motorSpeeds[1] = Range.clip(speed * Math.cos(heading) - dirSpeed, -1, 1);
        motorSpeeds[2] = Range.clip(speed * Math.cos(heading) + dirSpeed, -1, 1);
        motorSpeeds[3] = Range.clip(speed * Math.sin(heading) - dirSpeed, -1, 1);

        return motorSpeeds;
    }

    /**
     * Hold speed
     *
     * @param speed    motor speeds
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

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

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
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
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
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.sensorGyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}

