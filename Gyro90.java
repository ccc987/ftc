package org.firstinspires.ftc.teamcode;

//import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

// This code executes the 7th test (90 degree turn offset).
// The full specification is in the Engineering Notebook.
@Autonomous(name = "Gyro90", group = "Linear Opmode")
public class Gyro90 extends LinearOpMode {

    private Robot_OmniDrive robot = new Robot_OmniDrive();
    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    private static double TURN_P = 0.005;

    private DcMotor leftWheelF = null;               //Left Wheel Front
    private DcMotor leftWheelR = null;               //Left Wheel Back
    private DcMotor rightWheelF = null;              //Right Wheel Front
    private DcMotor rightWheelR = null;


    @Override
    public void runOpMode() {

        leftWheelF = hardwareMap.get(DcMotor.class, "D0");
        rightWheelF = hardwareMap.get(DcMotor.class, "D1");
        leftWheelR = hardwareMap.get(DcMotor.class, "D2");
        rightWheelR = hardwareMap.get(DcMotor.class, "D3");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuInit();

        waitForStart();
        runtime.reset();

        // Turn the robot for 90 degrees
        gyroTurn(90);

        // Stop the robot
    }

    private void gyroTurn(double deg) {
        double target_angle = getHeading() - deg;
        while (Math.abs((target_angle - getHeading())% 360) > 3 && opModeIsActive()) {
            double error_degrees = (target_angle - getHeading()) % 360; //Compute Error
            double motor_output = Range.clip(error_degrees * TURN_P, -.6 ,.6); //Get Correction
            // Send corresponding powers to the motors
            leftWheelF.setPower(0.5 * motor_output);
            leftWheelR.setPower(0.5 * motor_output);
            rightWheelF.setPower(0.5*motor_output);
            rightWheelR.setPower(0.5*motor_output);

            Orientation angles = imu.getAngularOrientation (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Spin Target : ",target_angle);
            telemetry.addData("Spin Degree : ",String.format(Locale.getDefault(), "%.1f", angles.firstAngle*-1));
            telemetry.update();
        }

        //extra 6 seconds to display the values for reading
        while (runtime.milliseconds() < 6000 && opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Spin Target : ",target_angle);
            telemetry.addData("Spin Degree : ",String.format(Locale.getDefault(), "%.1f", angles.firstAngle*-1));
            telemetry.update();
        }
    }

    private float getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }


    /* Initializes Rev Robotics IMU */
    private void imuInit() {
        // Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "imu";
        imu.initialize(parameters);
    }
}