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

@Autonomous(name = "AutoRedMovePlatform", group = "Opmode Robowatt")
//@Disabled
public class AutoRedMovePlatform extends LinearOpMode {

    // Class Members
    private DcMotor leftWheelF = null;               //Left Wheel Front
    private DcMotor leftWheelR = null;               //Left Wheel Back
    private DcMotor rightWheelF = null;              //Right Wheel Front
    private DcMotor rightWheelR = null;


    @Override
    public void runOpMode() {

        leftWheelF = hardwareMap.get(DcMotor.class, "D1");
        rightWheelF = hardwareMap.get(DcMotor.class, "D2");
        leftWheelR = hardwareMap.get(DcMotor.class, "D3");
        rightWheelR = hardwareMap.get(DcMotor.class, "D4");

        waitForStart();
        sleep(1000);
        pushPlatform();
    }

    private void moveSpecial(double drive,
                             double strafe,
                             double rotate) {
        //drive = -gamepad1.left_stick_y;  // Negative because the gamepad is weird
        //strafe = gamepad1.left_stick_x;
        //rotate = gamepad1.right_stick_x;

        double powerLeftF;
        double powerRightF;
        double powerLeftR;
        double powerRightR;

        powerLeftF = drive + strafe - rotate / 15;
        powerLeftR = drive - strafe - rotate / 15;

        powerRightF = drive - strafe - rotate;
        powerRightR = drive + strafe - rotate;

        leftWheelF.setPower(-powerLeftF);
        leftWheelR.setPower(-powerLeftR);

        rightWheelF.setPower(powerRightF);
        rightWheelR.setPower(powerRightR);
    }


    private void pushPlatform() {
        //robot facing building station tape
        //1. strafe right
        moveSpecial(0, 1.8, 0);
        telemetry.addLine("1. strafe right");
        sleep(2000);
        moveSpecial(0, 0, 0);
        sleep(2000);

        //2. move forward
        moveSpecial(0, 0, -0.7);
        telemetry.addLine("2. move forward");
        sleep(1000);
        moveSpecial(0, 0, 0);
        sleep(2000);

        //3. strafe left
        moveSpecial(0, 0.35, 0);
        sleep(1000);
        moveSpecial(0, 0, 0);
        sleep(2000);

        // 4. move forward
        moveSpecial(0.7, 0, 0);
        telemetry.addLine("4. strafe left");
        sleep(2000);

        moveSpecial(0, 0, 0);
        sleep(2000);
        //5. park under bridge

        moveSpecial(0, -0.7, 0);
        telemetry.addLine("5. park under bridge");
        sleep(2000);

        //6. stop
        moveSpecial(0, 0, 0);
        telemetry.addLine("6. stop");
    }
    /*private void pushClaw() {
        //robot facing building piece
        //1. strafe left
        move(0,-3,0);
        //2. move forward
        move(7,0,0);
        //3. claw down
        //flapServoL.setPosition(0);
        //flapServoR.setPosition(0);
        //4. rotate left
        move(0,0,-3);
        //5. strafe left
        move(0,-5,0);
        //6. release claw
        //flapServoL.setPosition(1);
        //flapServoR.setPosition(1);
        //7. park under bridge
        move(-10,0,0);
        //8. stop
        //move(0,0,0);
    }*/
}












