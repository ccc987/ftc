package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestColorSensor extends LinearOpMode {
    // Define a variable for our color sensor
    ColorSensor color;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "Color");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.update();
        }
    }
}