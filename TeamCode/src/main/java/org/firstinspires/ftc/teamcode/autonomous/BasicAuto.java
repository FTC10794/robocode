package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.libs.Auto;

/**
 * Basic Auto Class, 30 pts.
 * * Initializes hardware
 * * Detect Alliance Color via platform
 * * Clears Block Pickup
 * * Detects and removes Jewel (30pt)
 */

@Autonomous(name="Basic Auto (30pt)", group="Auto")
//@Disabled
public class BasicAuto extends Auto {

    public BasicAuto() {
        super();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Instantiate variables for super class
        instantiate();

        //Wait for start to be pressed
        waitForStart();
        telemetry.addData("Status", "Running Op Mode");

        //Begin Autonomous Routine
        initialize();
        detectAlliance(0);
        blockPickupClear();

        //Detect and Clear Jewel
        jewelDetection();

        sleep(2500);
    }
}
