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

import static org.firstinspires.ftc.teamcode.custom.Drivetrain.Robot.BOGG;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.custom.ArmMotor;
import org.firstinspires.ftc.teamcode.custom.Drivetrain;
import org.firstinspires.ftc.teamcode.custom.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.custom.Lift;

import java.util.Locale;

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

/*
This opmode is used to calibrate the AntiFlipCalibration system so that we can prevent coach eric from adding 
20 lbs to the robot to prevent it from flipping

use controller 1 to move the robot and move the lift
controller 2 controls the minimum height for when the system actually acivates and slows down the movement
also calibrates the modifier and how strong it is

 */

@TeleOp
public class AntiFlipCalibration extends OpMode
{
    // Declare OpMode members (initlaize primitve variables and set up servos that we don't initlaize in the class).
    private ElapsedTime runtime;
    private Drivetrain myDrivetrain;
    private Lift myLift;
    private ArmMotor myArmMotor;
    double liftMin;
    double liftMult;
    double liftCmd;
    double liftMultMod;
    double armCmd;

    



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
         runtime = new ElapsedTime();
         myDrivetrain = new Drivetrain(hardwareMap, BOGG);
         myLift = new Lift(hardwareMap);
         myArmMotor = new ArmMotor(hardwareMap);
         liftMin = 1200;
         liftMultMod = 1;

        
        

        // Initialize the hardware variables. Note that the strings used here as parameters
       



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        runtime.reset();
       

    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //formula for anti flip modifier
        if (myLift.LSMLeft.getCurrentPosition() > liftMin){
            liftMult = (myLift.LSMLeft.getCurrentPosition()-liftMin)/(myLift.posLSMMaxLeft-liftMin);
        } else {
            liftMult = 1;
        }

        //converts doad up and dpad down on gamepad 1 to to be doubles like a joystick is
        if (gamepad1.dpad_up){
            liftCmd = 0.5;
        } else if (gamepad1.dpad_down){
            liftCmd = -0.5;
        }else{
            liftCmd = 0;
        }

        //converts left and right on dpad to be doubles like for the lifr
        if (gamepad1.dpad_left){
            armCmd = 0.5;
        } else if (gamepad1.dpad_right){
            armCmd = -0.5;
        }else{
            armCmd = 0;
        }

        //calls the methods to move the arm, lift, and the drivetrain
        myDrivetrain.stickDrive(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                gamepad1.left_trigger*liftMult,
                0);

        myLift.moveSlide(liftCmd);
        myArmMotor.armMotStickControl(armCmd,1);

        //adjustment for the liftmin on gamepad 2
        if (gamepad2.dpad_up){
            liftMin = liftMin + 1;
        } else if (gamepad2.dpad_down){
            liftMin = liftMin - 1;
        }

        //adjustment for liftMultMod on gamepad 2
        //note this might not have to be used at all I just wanted to put it in in case
        //I found i needed a little bit more adjustment for the speed limiter
        if (gamepad2.a){
            liftMultMod = liftMultMod + 0.05;
        } else if (gamepad2.y){
            liftMultMod = liftMultMod - 0.05;
        }

        telemetry.addData("=====CONTROLS=====","");
        telemetry.addData("Gamepad 1","");
        telemetry.addData("Left Stick", " Strafe");
        telemetry.addData("Right Stick"," Rotate");
        telemetry.addData("Up/Down on Dpad"," lift adjustment");
        telemetry.addData("Left/Right on Dpad"," Arm Adjustment");
        telemetry.addData("Left Trigger"," 'brake'");
        telemetry.addData("Gamepad 2","");
        telemetry.addData("Up/Down on Dpad"," Up/Down liftMin");
        telemetry.addData("a/y","up/down liftMultMod");
        telemetry.addData("=======DATA=======","");
        telemetry.addData("liftMult",liftMult);
        telemetry.addData("liftMin",liftMin);
        telemetry.addData("liftMultMod",liftMultMod);
        telemetry.addData("LSMLeft position", myLift.LSMLeft.getCurrentPosition());
        telemetry.addData("LSMRight position", myLift.LSMRight.getCurrentPosition());
        telemetry.addData("ArmMot position", myArmMotor.armMot.getCurrentPosition());


        
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
