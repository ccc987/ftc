package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name = "SmallRobotMove", group = "Opmode RoboWatt")
public class SmallRobotMove extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Declare Hardware
    private DcMotor backWheelL = null;               //Left Wheel Front
    private DcMotor backWheelR = null;               //Left Wheel Back
    //Right Wheel Back

    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backWheelL = hardwareMap.get(DcMotor.class, "D1");
        backWheelR = hardwareMap.get(DcMotor.class, "D2");
        //scoopRight.setDirection(Servo.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        runtime.reset();

        //sleep(1000);
        move();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    private void move() {
        double drive;
        // Power for forward and back motion
        double rotateLeft;
        double rotateRight;// Power for rotating the robot



        drive = -gamepad1.left_stick_y;  // Negative because the gamepad is weird
        rotateLeft = gamepad1.left_trigger;
        rotateRight = gamepad1.right_trigger;

        double powerLeftR;
        double powerRightR;

        //if full power on left stick
        powerLeftR = drive + rotateRight - rotateLeft;
        powerRightR = drive - rotateRight + rotateLeft;


        backWheelL.setPower(powerLeftR);
        backWheelR.setPower(powerRightR);
    }

}