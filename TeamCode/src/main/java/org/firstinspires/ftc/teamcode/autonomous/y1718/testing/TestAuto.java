package org.firstinspires.ftc.teamcode.autonomous.y1718.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.libs.Auto;

@TeleOp(name="Test Auto", group="Test")
@Disabled
public class TestAuto extends Auto {

    public TestAuto() {
        super();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Instantiate variables for super class
        instantiate();

        //Wait for play to be pressed
        waitForStart();
        telemetry.addData("Status", "Running Op Mode");

        initialize();
        robot.motorDeposit.setPower(-1);
        sleep(500);
        robot.motorDeposit.setPower(0);
//        sleep(5000);

//        telemetry.addData("Current Heading:", "%3d deg", robot.sensorGyro.getHeading());
//        telemetry.update();
//        sleep(1000);
//
//        telemetry.addData("Turning Right: ", "270");
//        telemetry.update();
//        turnRight(270);
//        sleep(5000);
//
//        telemetry.addData("Turning Right: ", "180");
//        telemetry.update();
//        turnRight(180);
//        sleep(5000);
//
//        telemetry.addData("Turning Right: ", "90");
//        telemetry.update();
//        turnRight(90);
//        sleep(5000);
//
////        telemetry.addData("Centering: ", "0");
////        telemetry.update();
////        turn(0);
////        sleep(5000);
//
//
//        telemetry.addData("Turning Left: ", "-270");
//        telemetry.update();
//        turnLeft(270);
//        sleep(5000);
//
//        telemetry.addData("Turning Left: ", "-180");
//        telemetry.update();
//        turnLeft(180);
//        sleep(5000);
//
//        telemetry.addData("Turning Left: ", "-90");
//        telemetry.update();
//        turnLeft(90);
//        sleep(5000);
//
//
////        telemetry.addData("Turning: ", "360");
////        telemetry.addData("Turning Right: ", "360");
////        telemetry.update();
////        turn(360);
////        sleep(5000);
////
////        telemetry.addData("Turning Left: ", "-360");
////        telemetry.update();
////        turn(-360);
////        sleep(5000);
    }
}
