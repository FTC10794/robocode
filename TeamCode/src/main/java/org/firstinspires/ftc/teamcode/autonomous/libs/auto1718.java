//package org.firstinspires.ftc.teamcode.autonomous.libs;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.libs.Robot;
//
//public class auto1718 {
//
//    private Robot robot;
//    private Telemetry telemetry;
//
//    public auto1718(Robot _robot, Telemetry _telemetry) {
//        robot = _robot;
//        telemetry = _telemetry;
//    }
//
//    ///////////////////////////////////////////////////////////////////////////////////////////////
//    //
//    // Control Functions
//    //
//    ///////////////////////////////////////////////////////////////////////////////////////////////
//
//    /**
//     * Drive "Straight"
//     * @param dir direction of travel
//     * @param holdTime time to hold
//     */
//    public void drive(double dir, double holdTime) {
//        double[] motorSpeed = holonomicAuto(1, dir, 0);
//        holonomicHold(motorSpeed, holdTime);
//    }
//
//    /**
//     *
//     * @param dir -1 or 1, direction of turn
//     */
//    public void pointTurn(int dir) {
//        dir = dir < 0 ? -1 : 1;
//        robot.motorFrontLeft.setPower(dir);
//        robot.motorBackLeft.setPower(dir);
//        robot.motorFrontRight.setPower(dir);
//        robot.motorBackRight.setPower(dir);
//    }
//
//    /**
//     * Turn the robot
//     * @param angle angle to turn at
//     */
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
//
//    ///////////////////////////////////////////////////////////////////////////////////////////////
//    //
//    // Helper Functions
//    //
//    ///////////////////////////////////////////////////////////////////////////////////////////////
//
//    /**
//     * Method to drive the robot using the holonomic drive
//     * @param speed Speed of the robot drive train motors
//     * @param angle Angle of travel
//     * @param dirSpeed Speed of directional change
//     * @return double[] motor powers to assign
//     */
//    public double[] holonomicAuto(double speed, double angle, double dirSpeed) {
//        double[] motorSpeeds = {0, 0, 0, 0};
//
//        double heading = Math.toRadians(angle) + ((Math.PI)/4);
//        motorSpeeds[0] = Range.clip(speed * Math.sin(heading) + dirSpeed, -1, 1);
//        motorSpeeds[1] = Range.clip(speed * Math.cos(heading) - dirSpeed, -1, 1);
//        motorSpeeds[2] = Range.clip(speed * Math.cos(heading) + dirSpeed, -1, 1);
//        motorSpeeds[3] = Range.clip(speed * Math.sin(heading) - dirSpeed, -1, 1);
//
//        return motorSpeeds;
//    }
//
//    /**
//     * Hold speed
//     * @param speed motor speeds
//     * @param holdTime time to hold
//     */
//    public void holonomicHold(double[] speed, double holdTime) {
//        ElapsedTime holdTimer = new ElapsedTime();
//
//        // keep looping while we have time remaining.
//        holdTimer.reset();
//        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
//            // Update telemetry & Allow time for other processes to run.
//            robot.motorFrontLeft.setPower(speed[0]);
//            robot.motorFrontRight.setPower(speed[1]);
//            robot.motorBackLeft.setPower(speed[2]);
//            robot.motorBackRight.setPower(speed[3]);
//            telemetry.update();
//        }
//
//        stopMotors();
//    }
//
//    /**
//     * Stop all motion
//     */
//    public void stopMotors() {
//        // Stop all motion;
//        robot.motorFrontLeft.setPower(0);
//        robot.motorFrontRight.setPower(0);
//        robot.motorBackLeft.setPower(0);
//        robot.motorBackRight.setPower(0);
//    }
//}
