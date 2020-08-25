
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "TestPrint", group = "Opmode Robowatt")
public class TestPrint extends OpMode {
    // called when init button is  pressed.
    @Override
    public void init() {
    }

    @Override
    public void loop() {
        detectButton();
        //sleep(1000);

    }



    private void detectButton() {
        if (gamepad2.dpad_up) {
            telemetry.addLine("dpad up pressed");
        } else if (gamepad2.dpad_down) {
            telemetry.addLine("dpad down pressed");
        }
        if (gamepad2.dpad_left) {
            telemetry.addLine("dpad left pressed");
        } else if (gamepad2.dpad_right) {
            telemetry.addLine("dpad right pressed");
        }
        if (gamepad2.x) {
            telemetry.addLine("x pressed");
        } else if (gamepad2.y) {
            telemetry.addLine("y pressed");
        }
        if (gamepad2.a) {
            telemetry.addLine("a pressed");
        } else if (gamepad2.b) {
            telemetry.addLine("b pressed");
        }
        if (gamepad2.left_bumper) {
            telemetry.addLine("left bumper pressed");
        } else if (gamepad2.right_bumper) {
            telemetry.addLine("right bumper pressed");
        }
        if (gamepad2.left_trigger != 0) {
            telemetry.addLine("left trigger pressed");
        } else if (gamepad2.right_trigger != 0) {
            telemetry.addLine("right trigger pressed");
        }
        if (gamepad2.left_stick_x > 0) {
            telemetry.addLine("left stick right");
        } else if (gamepad2.left_stick_x < 0) {
            telemetry.addLine("left stick left");
        } else if (gamepad2.left_stick_y > 0) {
            telemetry.addLine("left stick down");
        } else if (gamepad2.left_stick_y < 0) {
            telemetry.addLine("left stick up");
        }
        if (gamepad2.right_stick_x > 0) {
            telemetry.addLine("right stick right");
        } else if (gamepad2.right_stick_x < 0) {
            telemetry.addLine("right stick left");
        } else if (gamepad2.right_stick_y > 0) {
            telemetry.addLine("right stick down");
        } else if (gamepad2.right_stick_y < 0) {
            telemetry.addLine("right stick up");
        }
    }
}
