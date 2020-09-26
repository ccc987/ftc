package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "ClawUpDown", group = "Opmode Robowatt")

public class ClawUpDown extends OpMode {

    boolean leftY, rightY;
    double armPosition, linearSlidePosition, clawPosition, flapPosition; //gripPosition, contPower;
    double MIN_POSITION = 0, MAX_POSITION = 1;
    private Servo armServo = null; //Left Wheel Front
    private Servo linearSlideServo = null;
    private Servo clawServo = null;
    private Servo flapServo = null;
    private ElapsedTime runtime = new ElapsedTime();

    // called when init button is  pressed.
    @Override
    public void init() {

        armServo = hardwareMap.servo.get("LRS1");
        linearSlideServo = hardwareMap.servo.get("FBS3"); //FMS2 FBS3
        clawServo = hardwareMap.servo.get("FMS2");

        armPosition = .5; // set arm to half way up.
        linearSlidePosition = .5;
        clawPosition = .5;


    }

    @Override
    public void loop() {
        runtime.reset();
        move();
        //sleep(1000);

    }

    @Override
    public void start() {
        runtime.reset();
    }

    private void move() {
        if (gamepad2.dpad_up) {
            armServo.setPosition(1);
        } else if (gamepad2.dpad_down) {
            armServo.setPosition(0);
        } else
            armServo.setPosition(0.5);

        if (gamepad2.dpad_left) {
            linearSlideServo.setPosition(1);
        } else if (gamepad2.dpad_right) {
            linearSlideServo.setPosition(0);
        } else
            linearSlideServo.setPosition(0.5);

        if (gamepad2.b) {
            clawServo.setPosition(1);
        } else if (gamepad2.x) {
            clawServo.setPosition(0);
        } else
            clawServo.setPosition(0.5);
    }
}
