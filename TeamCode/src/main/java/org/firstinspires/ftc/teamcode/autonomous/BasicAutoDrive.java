package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.libs.Auto;

/**
 * Basic Auto Class, 30 pts.
 * * Initializes hardware
 * * Detect Alliance Color via platform
 * * Clears Block Pickup
 * * Detects and removes Jewel (30pts)
 * * Detects Cryptobox Key
 * * Drives off platform
 * * Drives to Cryptobox
 * * Determines which column to drive to
 * * Places block in correct column (15pts + 30pts = 45pts)
 * * Parks in Safe Zone (10pts)
 */

@Autonomous(name="Basic Auto Drive (40pt)", group="Auto")
//@Disabled
public class BasicAutoDrive extends Auto {

    public BasicAutoDrive() {
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
        sleep(1000);

        //Detect and Clear Jewel
        jewelDetection();

        //Drive and Park
        driveOffTo();
        park();

        sleep(2500);
    }
}