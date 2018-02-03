package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.libs.Auto;

/**
 * Basic Auto Class, 30 pts.
 * * Initializes hardware
 * * Detect Alliance Color via platform
 * * Picks up the block
 * * Detects and removes Jewel
 * * FIXME needs the rest
 */

@Autonomous(name="Front Auto (85pt)", group="Auto")
@Disabled
public class FrontAuto extends Auto {

    public FrontAuto() {
        super();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Instantiate variables for super class
        instantiate();

        //Wait for play to be pressed
        waitForStart();
        telemetry.addData("Status", "Running Op Mode");

        //Begin Autonomous Routine
        initialize();
        detectAlliance(0);
        blockPickupClear();

        //Detect and Clear Jewel
        jewelDetection();

        //Detect Pictograph
        detectPictograph();

        //Drive and Deposit
//        driveOffPlatform();
        driveToSideLocker();
        deposit();
        park();

        sleep(2500);
    }
}