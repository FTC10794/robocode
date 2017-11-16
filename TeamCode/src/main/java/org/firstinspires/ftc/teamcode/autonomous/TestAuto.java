package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.libs.MotorFunctions;
import org.firstinspires.ftc.teamcode.libs.Robot;

@TeleOp(name="Test Autonomous", group="Test Autonomous")
//@Disabled
public class TestAuto extends LinearOpMode {
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

        //Wait For Autonomous to Start
        waitForStart();

        initialize();

        telemetry.addData("Current Heading:", "%3d deg", robot.sensorGyro.getHeading());
        telemetry.update();
        sleep(1000);

        telemetry.addData("Turning Right: ", "270");
        telemetry.update();
        turn(270);
        sleep(5000);

        telemetry.addData("Turning Right: ", "180");
        telemetry.update();
        turn(180);
        sleep(5000);

        telemetry.addData("Turning Right: ", "90");
        telemetry.update();
        turn(90);
        sleep(5000);


        telemetry.addData("Turning Left: ", "-270");
        telemetry.update();
        turn(-270);
        sleep(5000);

        telemetry.addData("Turning Left: ", "-180");
        telemetry.update();
        turn(-180);
        sleep(5000);

        telemetry.addData("Turning Left: ", "-90");
        telemetry.update();
        turn(-90);
        sleep(5000);


        telemetry.addData("Turning: ", "360");
        telemetry.addData("Turning Right: ", "360");
        telemetry.update();
        turn(360);
        sleep(5000);

        telemetry.addData("Turning Left: ", "-360");
        telemetry.update();
        turn(-360);
        sleep(5000);
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
        robot.servoJewelArm.setPosition(0);
        sleep(500);
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
        robot.motorFrontRight.setPower(-dir);
        robot.motorBackRight.setPower(-dir);
    }

    /**
     * Turn the robot
     * @param angle angle to turn at
     */
    public void turn(double angle) {
        angle = (angle > 360) ? (angle/360) : angle;
        double desiredHeading = angle < 0 ? 360 + angle : angle;
        double currentHeading;

        // Calibrate the gyroscope
        robot.sensorGyro.calibrate();
        while(robot.sensorGyro.isCalibrating()) {
            telemetry.addData("> Calibrating:", "Gyro");
            telemetry.update();
            sleep(50);
        }

        currentHeading = robot.sensorGyro.getHeading();
        if (desiredHeading == Math.abs(360)) {
            telemetry.addData(">> Current Heading!!", "%3d deg", robot.sensorGyro.getHeading());
            telemetry.update();
            stopMotors();
        } else if (angle < 0 && currentHeading > desiredHeading) {
            while (robot.sensorGyro.getHeading() >= desiredHeading) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(-1); // turn right
            }
            stopMotors();
        } else {
            while (robot.sensorGyro.getHeading() <= desiredHeading) {
                telemetry.addData(">> Heading", "%3d deg", robot.sensorGyro.getHeading());
                telemetry.update();
                pointTurn(1); // turn left
            }
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
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
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
    public double getError(double targetAngle) {

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
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
