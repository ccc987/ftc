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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoRedPark", group = "Opmode Robowatt")
//@Disabled
public class AutoRedPark extends LinearOpMode {

    // Class Members
    private DcMotor leftWheelF = null;               //Left Wheel Front
    private DcMotor leftWheelR = null;               //Left Wheel Back
    private DcMotor rightWheelF = null;              //Right Wheel Front
    private DcMotor rightWheelR = null;
    private Servo flapServo = null;



    @Override
    public void runOpMode() {

        leftWheelF = hardwareMap.get(DcMotor.class, "D1");
        rightWheelF = hardwareMap.get(DcMotor.class, "D2");
        leftWheelR = hardwareMap.get(DcMotor.class, "D3");
        rightWheelR = hardwareMap.get(DcMotor.class, "D4");
        flapServo = hardwareMap.servo.get("LCS4");

        waitForStart();
        sleep(1000);
        park();
    }

    private void move(double drive,
                      double strafe,
                      double rotate) {
        //drive = -gamepad1.left_stick_y;  // Negative because the gamepad is weird
        //strafe = gamepad1.left_stick_x;
        //rotate = gamepad1.right_stick_x;

        double powerLeftF;
        double powerRightF;
        double powerLeftR;
        double powerRightR;

        powerLeftF = drive/5 + strafe/5 - rotate / 15;
        powerLeftR = drive/5 - strafe/5 - rotate / 15;

        powerRightF = drive/5 - strafe/5 - rotate;
        powerRightR = drive/5 + strafe/5 - rotate;

        leftWheelF.setPower(-powerLeftF);
        leftWheelR.setPower(-powerLeftR);

        rightWheelF.setPower(powerRightF);
        rightWheelR.setPower(powerRightR);
    }
    private void park() {
        //move forward
        sleep(5000);
        move(0,1.1,0);
        sleep(3000);
        //strafe left
        move(1.5,0,0);
        sleep(1000);
        //stop
        move(0,0,0);
        sleep(1000);
    }
}














