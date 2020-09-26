package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name = "StrafingTest", group = "Opmode RoboWatt")
public class StrafingTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Declare Hardware
    private DcMotor leftWheelF = null;               //Left Wheel Front
    private DcMotor leftWheelR = null;               //Left Wheel Back
    private DcMotor rightWheelF = null;              //Right Wheel Front
    private DcMotor rightWheelR = null;
    private DcMotor armMotor = null;                   //Left Wheel Front
    private DcMotor screwMotor = null;
    private Servo clawServo = null;
    private Servo flapServoL = null;//Right Wheel Back
    private Servo flapServoR = null;

    private boolean leftY, rightY;
    private double armPosition, screwPosition, clawPosition, flapLPosition, flapRPosition; //gripPosition, contPower;
    private double MIN_POSITION = 0, MAX_POSITION = 1;

    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftWheelF = hardwareMap.get(DcMotor.class, "D1");
        rightWheelF = hardwareMap.get(DcMotor.class, "D2");
        leftWheelR = hardwareMap.get(DcMotor.class, "D3");
        rightWheelR = hardwareMap.get(DcMotor.class, "D4");
        screwMotor = hardwareMap.get(DcMotor.class, "2B");
        armMotor = hardwareMap.get(DcMotor.class, "3B");//FMS2 FBS3
        clawServo = hardwareMap.servo.get("FMS2");
        flapServoL = hardwareMap.servo.get("LF0");
        flapServoR = hardwareMap.servo.get("RF1");


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
        double strafe;  // Power for left and right motion
        double rotateLeft;
        double rotateRight;
        double screwForward;
        double screwBackward;
        double armUp;
        double armDown;// Power for rotating the robot


        double drive2;
        double strafe2;

        drive = -gamepad1.left_stick_y;  // Negative because the gamepad is weird
        strafe = gamepad1.left_stick_x;
        rotateLeft = gamepad1.left_trigger;
        rotateRight = gamepad1.right_trigger;
        screwForward = gamepad2.right_trigger;
        screwBackward = gamepad2.left_trigger;
        armUp = gamepad2.right_stick_y;
        armDown = gamepad2.right_stick_y;


        drive2 = -gamepad1.right_stick_y;
        strafe2 = gamepad1.right_stick_x;

        double powerLeftF;
        double powerRightF;
        double powerLeftR;
        double powerRightR;
        double powerScrew;
        double powerArm;

        //if full power on left stick
        if (drive != 0 || strafe != 0 || rotateRight != 0 || rotateLeft != 0) {

            powerLeftF = drive/3 + strafe/3 + rotateRight/3 - rotateLeft/5;
            powerLeftR = drive/3 - strafe/3 + rotateRight/3 - rotateLeft/5;

            powerRightF = drive/3 - strafe/3 - rotateRight/3 + rotateLeft/5;
            powerRightR = drive/3 + strafe/3 - rotateRight/3 + rotateLeft/5;

            leftWheelF.setPower(-powerLeftF);
            leftWheelR.setPower(-powerLeftR);

            rightWheelF.setPower(powerRightF);
            rightWheelR.setPower(powerRightR);

        } else {
            // else half power
            powerLeftF = drive2/7 + strafe2/5 + rotateRight;
            powerLeftR = drive2/7 - strafe2/5 + rotateRight;

            powerRightF = drive2/7 - strafe2/5 - rotateLeft;
            powerRightR = drive2/7 + strafe2/5 - rotateLeft;

            leftWheelF.setPower(-powerLeftF);
            leftWheelR.setPower(-powerLeftR);

            rightWheelF.setPower(powerRightF);
            rightWheelR.setPower(powerRightR);
        }

        powerScrew = screwForward - screwBackward;
        screwMotor.setPower(powerScrew);

        powerArm = armUp - armDown;
        armMotor.setPower(powerArm);





        if (gamepad2.b) {
            clawServo.setPosition(1);
        } else if (gamepad2.x) {
            clawServo.setPosition(0);
        }

        if (gamepad2.left_bumper) {
            flapServoL.setPosition(1);
            flapServoR.setPosition(1);
        } else if (gamepad2.right_bumper) {
            flapServoL.setPosition(0);
            flapServoR.setPosition(0);
        }
    }
}
