package org.firstinspires.ftc.teamcode.autonomous.y1718.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.libs.MotorFunctions;
import org.firstinspires.ftc.teamcode.autonomous.libs.Auto;
import org.firstinspires.ftc.teamcode.libs.Robot;

@TeleOp(name="Test Auto", group="Test")
//@Disabled
public class TestAuto extends Auto {
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

        telemetry.addData("Centering: ", "0");
        telemetry.update();
        turn(0);
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
}
