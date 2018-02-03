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

@Autonomous(name="Back Auto (85pt)", group="Auto")
//@Disabled
public class BackAuto extends Auto {

    public BackAuto() {
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

        //Detect Pictograph
        detectPictograph();

        //Drive
        driveToCornerLocker();
        telemetry.addData("> Depositing", "");

        //Deposit
        deposit();
        park();

        sleep(2500);
    }
}