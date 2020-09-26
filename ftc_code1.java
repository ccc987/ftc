package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name = "StrafingTestWorking", group = "Opmode RoboWatt")
public class StrafingTestWorking extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Declare Hardware
    private DcMotor leftWheelF = null;               //Left Wheel Front
    private DcMotor leftWheelR = null;               //Left Wheel Back
    private DcMotor rightWheelF = null;              //Right Wheel Front
    private DcMotor rightWheelR = null;
    private DcMotor intakeWheel1 = null;
    private Servo wobbleServoHand = null;
    private Servo wobbleServoArm = null;

    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftWheelF = hardwareMap.get(DcMotor.class, "D1");
        rightWheelF = hardwareMap.get(DcMotor.class, "D2");
        leftWheelR = hardwareMap.get(DcMotor.class, "D3");
        rightWheelR = hardwareMap.get(DcMotor.class, "D4");
        intakeWheel1 = hardwareMap.get(DcMotor.class, "I1");
        wobbleServoArm = hardwareMap.get(Servo.class, "S1");
        wobbleServoHand = hardwareMap.get(Servo.class, "S2");
        //scoopRight.setDirection(Servo.Direction.REVERSE);
        
        intakeWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
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
        double rotateRight;// Power for rotating the robot
        //int intake;


        double drive2;
        double strafe2;

        drive = -gamepad1.left_stick_y;  // Negative because the gamepad is weird
        strafe = gamepad1.left_stick_x;
        rotateLeft = gamepad1.left_trigger;
        rotateRight = gamepad1.right_trigger;
        //intake = gamepad2.left_trigger;

        drive2 = -gamepad1.right_stick_y;
        strafe2 = gamepad1.right_stick_x;

        double powerLeftF;
        double powerRightF;
        double powerLeftR;
        double powerRightR;
        // double powerIntake;
        //intakeWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeWheel1.setPower(1);
        //if full power on left stick
        if (drive != 0 || strafe != 0 || rotateRight != 0 || rotateLeft != 0) {
            powerLeftF = drive + strafe + rotateRight - rotateLeft;
            powerLeftR = drive - strafe + rotateRight - rotateLeft;
            //powerIntake = intake;
            powerRightF = drive - strafe - rotateRight + rotateLeft;
            powerRightR = drive + strafe - rotateRight + rotateLeft;

            leftWheelF.setPower(-powerLeftF);
            leftWheelR.setPower(-powerLeftR);

            rightWheelF.setPower(powerRightF);
            rightWheelR.setPower(powerRightR);

            //intakeWheel1.setPower(powerIntake);

        } else {
            // else half power
            powerLeftF = drive2 + strafe2 + rotateRight;
            powerLeftR = drive2 - strafe2 + rotateRight;

            powerRightF = drive2 - strafe2 - rotateLeft;
            powerRightR = drive2 + strafe2 - rotateLeft;

            leftWheelF.setPower(-powerLeftF);
            leftWheelR.setPower(-powerLeftR);

            rightWheelF.setPower(powerRightF);
            rightWheelR.setPower(powerRightR);
        }
        if (gamepad2.x) {
            wobbleServoArm.setPosition(1);
        } else if (gamepad2.b) {
            wobbleServoArm.setPosition(0.5);
        }

        if (gamepad1.a) {
            wobbleServoHand.setPosition(1);
        } else if (gamepad1.y) {
            wobbleServoHand.setPosition(0);
        }
        if (gamepad2.a) {
            //down
            intakeWheel1.setPower(0.15);
            intakeWheel1.setDirection(DcMotor.Direction.FORWARD);
            intakeWheel1.setTargetPosition(175);
            intakeWheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad2.y) {
            
            //up
            intakeWheel1.setPower(0.75);
            intakeWheel1.setDirection(DcMotor.Direction.REVERSE);
            intakeWheel1.setTargetPosition(250);
            intakeWheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}

