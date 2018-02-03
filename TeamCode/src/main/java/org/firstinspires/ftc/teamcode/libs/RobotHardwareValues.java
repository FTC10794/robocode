package org.firstinspires.ftc.teamcode.libs;

/**
 * Assigns values to all servo positions, for a one change update
 */

public class RobotHardwareValues {

    public static double servoRotatorLeft = 0.165;
    public static double servoRotatorRight = 0.0035;
    public static double servoRotatorCenter = 0.08;

    public static double servoSlideOneBlock = 0.025;
    public static double servoSlideTwoBlock = 0.3;
    public static double servoSlideScoring;

    //This is left if you are facing the robot
    public static double servoLeftPaddleInit = 1;
    public static double servoLeftPaddleIn = .45;
    public static double servoLeftPaddleOut = .75;
    public static double servoLeftPaddleScoring = 0.55;
    public static double servoLeftPaddleCapture;

    public static double servoRightPaddleInit = 0;
    public static double servoRightPaddleIn = .55; //45 //.6
    public static double servoRightPaddleOut = 0.3;
    public static double servoRightPaddleScoring = 0.45;
    public static double servoRightPaddleCapture;

    public static double servoRelicArmInit = 1;
    public static double servoRelicArmDown = 0.2;
    public static double servoRelicArmUp = 0.7;
    public static double servoRelicArmCenter;

    public static double servoRelicClawOpen = 0;
    public static double servoRelicClawClosed = 0.5;

    public static double servoJewelArmUp = .85;
    public static double servoJewelArmDown = 0;
    public static double servoJewelArmCenter = .7;

    public static double servoJewelPutterKickForward = 1;
    public static double servoJewelPutterKickBackward = 0;
    public static double servoJewelPutterCenter = 0.5;

    public static double servoLeftWheelOff = 0;
    public static double servoLeftWheelOut = 1; // > 0
    public static double servoLeftWheelIn = -1; // < 0
    public static double servoLeftWheelGentle = -0.01; // needs checking
    public static double servoLeftWheelFast = -0.5;

    public static double servoRightWheelOff = 0.15;
    public static double servoRightWheelOut = -1; // > 0.1
    public static double servoRightWheelIn = 1; // < 0.1
    public static double servoRightWheelGentle = 0.16;
    public static double servoRightWheelFast = 0.5;

    public static double servoAutoDepositInit = 0;
    public static double servoAutoDepositScore = .5;

    public static double servoJackClawInit = 1;
    public static double servoJackClawScore = 0;

}
