/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "BlueSideAutoRightPark", group = "Concept")
//@Disabled
public class BlueSideAutoRightPark extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";


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
    public void runOpMode() {


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

        waitForStart();
        sleep(1000);
        park();
    }

    private void move(double drive,
                      double strafe,
                      double rotate) {

        double powerLeftF;
        double powerRightF;
        double powerLeftR;
        double powerRightR;

        powerLeftF = drive + strafe + rotate;
        powerLeftR = drive - strafe + rotate;

        powerRightF = drive - strafe - rotate;
        powerRightR = drive + strafe - rotate;

        leftWheelF.setPower(-powerLeftF);
        leftWheelR.setPower(-powerLeftR);

        rightWheelF.setPower(powerRightF);
        rightWheelR.setPower(powerRightR);
    }


    private void park() {
        sleep(2000);
        move(0,0.5,0);
        sleep(1000);
        move(0, 0,0);
        move(0.75, 0, 0);
        sleep(4000);
        move(0, 0, 0);
    }
}
