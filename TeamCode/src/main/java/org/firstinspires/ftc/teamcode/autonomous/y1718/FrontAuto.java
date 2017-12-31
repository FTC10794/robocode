package org.firstinspires.ftc.teamcode.autonomous.y1718;

import org.firstinspires.ftc.robotcontroller.libs.MotorFunctions;
import org.firstinspires.ftc.teamcode.autonomous.libs.Auto;

/**
 * Basic Auto Class, 30 pts.
 * * Initializes hardware
 * * Detect Alliance Color via platform
 * * Picks up the block
 * * Detects and removes Jewel
 * * FIXME needs the rest
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Front Auto (85pt)", group="Auto")
//@Disabled
public class FrontAuto extends Auto {
    // Class variables
    private MotorFunctions motorFunctions;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Running Op Mode");

        motorFunctions = new MotorFunctions(-1, 1, 0, 1, .01);

        waitForStart();

        initialize();
//        blockPickup();
//        jewelDetection();

        sleep(2500);
    }
}