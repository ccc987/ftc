package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;

//@Disabled
@TeleOp(name = "StrafingTestWorking", group = "Opmode RoboWatt")
public class StrafingTestWorking extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Declare Hardware
    private DcMotor leftWheelF = null;               //Left Wheel Front
    private DcMotor leftWheelR = null;               //Left Wheel Back
    private DcMotor rightWheelF = null;              //Right Wheel Front
    private DcMotor rightWheelR = null;
    private DcMotor armWheel = null;
    private DcMotor intakeWheel1 = null;
    private DcMotor intakeWheel2 = null;
    //private DcMotor intakeWheel3 = null;
    private Servo wobbleServoHand = null;
    private Servo ringPush = null;
    private DcMotor outtakeWheel1 = null;

    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftWheelF = hardwareMap.get(DcMotor.class, "D1");
        rightWheelF = hardwareMap.get(DcMotor.class, "D2");
        leftWheelR = hardwareMap.get(DcMotor.class, "D3");
        rightWheelR = hardwareMap.get(DcMotor.class, "D4");
        armWheel = hardwareMap.get(DcMotor.class, "A1");
        intakeWheel1 = hardwareMap.get(DcMotor.class, "I1");
        intakeWheel2 = hardwareMap.get(DcMotor.class, "I2");
        wobbleServoHand = hardwareMap.get(Servo.class, "S2");
        ringPush = hardwareMap.get(Servo.class, "P1");
        outtakeWheel1 = hardwareMap.get(DcMotor.class, "O1");
        //scoopRight.setDirection(Servo.Direction.REVERSE);

        //armWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        double intake;
        double raise;
        double lower;
        double outtake;


        double drive2;
        double strafe2;

        drive = -gamepad1.left_stick_y;  // Negative because the gamepad is weird
        strafe = gamepad1.left_stick_x;
        rotateLeft = gamepad1.left_trigger;
        rotateRight = gamepad1.right_trigger;
        intake = gamepad2.left_stick_y;
        raise  = gamepad2.left_trigger;
        lower = gamepad2.right_trigger;
        drive2 = -gamepad1.right_stick_y;
        strafe2 = gamepad1.right_stick_x;
        outtake = gamepad2.right_stick_y;

        double powerLeftF;
        double powerRightF;
        double powerLeftR;
        double powerRightR;
        double powerIntake;
        double powerRaise;
        double powerLower;
        double powerOuttake;

        double voltageFactor = getFactorOfVoltage();
        telemetry.addData("Multiplier", voltageFactor);

        powerOuttake = outtake;
        powerRaise = raise;
        powerLower = -lower;


        outtakeWheel1.setPower(powerOuttake * voltageFactor);
        armWheel.setPower(powerRaise);
        armWheel.setPower(powerLower);
        powerIntake = intake;
        intakeWheel1.setPower(powerIntake * voltageFactor);
        intakeWheel2.setPower(-powerIntake * voltageFactor);
        //if full power on left stick
        if (drive != 0 || strafe != 0 || rotateRight != 0 || rotateLeft != 0) {
            powerLeftF = drive + strafe + rotateRight - rotateLeft;
            powerLeftR = drive - strafe + rotateRight - rotateLeft;

            powerRightF = drive - strafe - rotateRight + rotateLeft;
            powerRightR = drive + strafe - rotateRight + rotateLeft;

            leftWheelF.setPower(-powerLeftF);
            leftWheelR.setPower(-powerLeftR);

            rightWheelF.setPower(powerRightF);
            rightWheelR.setPower(powerRightR);



        } else {
            // else half power
            powerLeftF = drive2 + strafe2 + rotateRight/2 - rotateLeft/2;
            powerLeftR = drive2 - strafe2 + rotateRight/2 - rotateLeft/2;

            powerRightF = drive2 - strafe2 - rotateRight/2 + rotateLeft/2;
            powerRightR = drive2 + strafe2 - rotateRight/2 + rotateLeft/2;

            leftWheelF.setPower(-powerLeftF*0.5);
            leftWheelR.setPower(-powerLeftR*0.5);

            rightWheelF.setPower(powerRightF*0.5);
            rightWheelR.setPower(powerRightR*0.5);
        }
        if (gamepad2.x) {
            wobbleServoHand.setPosition(1);
        } else if (gamepad2.b) {
            wobbleServoHand.setPosition(0.5);
        }
        if (gamepad2.left_bumper) {
            ringPush.setPosition(1);

        } else if (gamepad2.right_bumper) {
            ringPush.setPosition(0.7);
        }

        /*if (gamepad2.a) {
            //down
            armWheel.setPower(0.05);
            armWheel.setDirection(DcMotor.Direction.FORWARD);
            armWheel.setTargetPosition(50);
            armWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad2.y) {

            //up
            armWheel.setPower(0.5);
            armWheel.setDirection(DcMotor.Direction.REVERSE);
            armWheel.setTargetPosition(350);
            armWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }*/
    }
    private double getFactorOfVoltage() {
        double currentVoltage = getBatteryVoltage();
        double mult;
        if (currentVoltage >= 14.3) {
            mult = 0.88;
        } else if (currentVoltage >= 14.2) {
            mult = 0.90;
        } else if (currentVoltage >= 14.1) {
            mult = 0.92;
        } else if (currentVoltage >= 14.0) {
            mult = 0.94;
        } else if (currentVoltage >= 13.9) {
            mult = 0.96;
        } else if (currentVoltage >= 13.8) {
            mult = 0.98;
        } else if (currentVoltage <= 12.5) {
            telemetry.addLine("Change the battery!");
            mult = 1;
        } else {
            mult = 1;
        }
        return mult;
    }


    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}

