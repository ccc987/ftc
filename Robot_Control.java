package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This example is designed to show how to identify a target, get the robot's position, and then plan
 * and execute an approach path to the target.
 *
 * This OpMode uses two "utility" classes to abstract (hide) the hardware and navigation GUTs.
 * These are:  Robot_OmniDrive and Robot_Navigation.
 *
 * This LinearOpMode uses basic hardware and nav calls to drive the robot in either manual or auto mode.
 * AutoMode is engaged by pressing and holding the Left Bumper.  Release the Bumper to return to Manual Mode.
 *
 *  *ManualMode* simply uses the joysticks to move the robot in three degrees of freedom.
 *  - Left stick X (translate left and right)
 *  - Left Stick Y (translate forward and backwards)
 *  - Right Stick X (rotate CW and CCW)
 *
 *  *AutoMode* will approach the image target and attempt to reach a position directly in front
 *  of the center line of the image, with a predefined stand-off distance.
 *
 *  To simplify this example, a gyro is NOT used.  Therefore there is no attempt being made to stabilize
 *  strafing motions, or to perform field-centric driving.
 *
 */

@TeleOp(name="Robot_Control", group="main")
public class Robot_Control extends LinearOpMode {

    final double TARGET_DISTANCE =  400.0;    // Hold robot's center 400 mm from target

    /* Declare OpMode members. */
    Robot_OmniDrive     robot    = new Robot_OmniDrive();   // Use Omni-Directional drive system
    Robot_Navigation    nav      = new Robot_Navigation();  // Use Image Tracking library
    Robot_Gyro    gyro      = new Robot_Gyro();  // Use Gyro library

    @Override
    public void runOpMode() {

        // Initialize the robot and navigation
        robot.initDrive(this);
        nav.initVuforia(this, robot);
        gyro.imuInit(this, robot);

        // Activate Vuforia (this takes a few seconds)
        nav.activateTracking();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            // Prompt User
            telemetry.addData(">", "Press start");

            // Display any Nav Targets while we wait for the match to start
            nav.targetsAreVisible();
            //nav.addNavTelemetry();
            //gyro.addGyroTelemetry();
            telemetry.update();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData(">", "Press gamepad1 Left Bumper to track target");
            telemetry.addData(">", "Press gamepad1 Right Bumper to Gyro");

            // auto drive or manual drive?
            // In auto drive, the robot will approach any target it can see and then press against it
            // In manual drive the robot responds to the Joystick.
            if (nav.targetsAreVisible() && gamepad1.left_bumper) {
                // Calculate automatic target approach
                nav.cruiseControl(TARGET_DISTANCE);

            } else if (gamepad1.right_bumper) {

                gyro.gyroTurn(0);

            } else {
                // Drive the robot using the joysticks
                robot.manualDrive();
            }

            // Build telemetry messages with Navigation Information;
           // nav.addNavTelemetry();

            //telemetry.addData("Red", robot.color.red());
            //telemetry.addData("Green", robot.color.green());
            //telemetry.addData("Blue", robot.color.blue());
            //  Move the robot according to the pre-determined axis motions

            robot.moveRobot();
            telemetry.update();
        }

        telemetry.addData(">", "Shutting Down. Bye!");
        telemetry.update();
    }
}
