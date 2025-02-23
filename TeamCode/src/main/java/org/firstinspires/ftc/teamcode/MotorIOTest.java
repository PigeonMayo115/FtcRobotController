/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.custom.ArmMotor;
import org.firstinspires.ftc.teamcode.custom.Drivetrain;
import org.firstinspires.ftc.teamcode.custom.Lift;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp
public class MotorIOTest extends OpMode {
    Drivetrain myDriveTrain;
    DcMotor LSMLeft = null;
    DcMotor LSMRight = null;
    ArmMotor myArmMotor;
    @Override
    public void init() {
        myDriveTrain = new Drivetrain(hardwareMap, 0);
        myArmMotor = new ArmMotor(hardwareMap);
        LSMLeft = hardwareMap.dcMotor.get("LSMLeft");
        LSMRight = hardwareMap.dcMotor.get("LSMRight");

        LSMLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        LSMRight.setDirection(DcMotorSimple.Direction.FORWARD);

        LSMLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSMRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LSMLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LSMRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LSMLeft.setPower(0);
        LSMRight.setPower(0);





    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.a){
            myDriveTrain.singleMot(1);
            telemetry.addData("motor: ","front left");
        }
        else if (gamepad1.b){
            myDriveTrain.singleMot(2);
            telemetry.addData("motor: ","back left");
        }
        else if (gamepad1.x){
            myDriveTrain.singleMot(3);
            telemetry.addData("motor: ","front right");
        }
        else if (gamepad1.y){
            myDriveTrain.singleMot(4);
            telemetry.addData("motor: ", "back right");

        }else if ( gamepad1.dpad_left){
            LSMLeft.setPower(1);
            LSMRight.setPower(1);
        }else if (gamepad1.dpad_right){
            LSMLeft.setPower(-1);
            LSMRight.setPower(-1);
        }else if (gamepad1.left_stick_y != 0){
            LSMLeft.setPower(gamepad1.left_stick_y);
        }else if (gamepad1.right_stick_y != 0){
            LSMRight.setPower(gamepad1.right_stick_y);
        }else if (gamepad1.dpad_down){
            myArmMotor.armMot.setPower(1);
        } else if (gamepad1.dpad_up){
            myArmMotor.armMot.setPower(-1);
        }else{
            myDriveTrain.setMotPow(0,0,0,0,0);
            LSMLeft.setPower(0);
            LSMRight.setPower(0);
            myArmMotor.armMot.setPower(0);
            telemetry.addData("motor: ", "none");
        }





        telemetry.addData("front left position",myDriveTrain.flMot.getCurrentPosition());
        telemetry.addData("back left position",myDriveTrain.blMot.getCurrentPosition());
        telemetry.addData("front right position",myDriveTrain.frMot.getCurrentPosition());
        telemetry.addData("back right position",myDriveTrain.brMot.getCurrentPosition());
        telemetry.addData("armMotor position",myArmMotor.armMot.getCurrentPosition());
        telemetry.addData("liftMotLeft position",LSMLeft.getCurrentPosition());
        telemetry.addData("liftMotRight position",LSMRight.getCurrentPosition());
        telemetry.addData("liftMotLeft power",LSMLeft.getPower());
        telemetry.addData("liftMotRight power",LSMRight.getPower());
        telemetry.addData("=== CONTROLS ===", null);
        telemetry.addData("drivetrain:","a = fl,b = bl,x = fr,y = br");
        telemetry.addData("armMotor","dpad up = up, dpad down = down");
        telemetry.addData("linear slides","dpad left = down, dpad right = up");


        if (gamepad1.dpad_down){
            myDriveTrain.setMotSRE();
        }else{
            myDriveTrain.setMotRUE();
        }


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
