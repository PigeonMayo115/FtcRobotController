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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.custom.ArmMotor;
import org.firstinspires.ftc.teamcode.custom.CrServo;
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

@Autonomous
public class ITDMainAutonomousLeftSampleV2 extends OpMode
{
    private Drivetrain myDrivetrain;
    private CrServo myCrServo;
    private Servo wristServo;
    private ArmMotor myArmMotor;
    private Lift myLift;
    int step = 0;
    boolean stepDone = false;
    boolean stepDone2 = false;
    boolean stepDone3 = false;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        myDrivetrain = new Drivetrain(hardwareMap, 0);
        myCrServo = new CrServo(hardwareMap);
        myLift = new Lift(hardwareMap);
        myArmMotor = new ArmMotor(hardwareMap);
        wristServo = hardwareMap.servo.get("wristServo");



    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("heading",myDrivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("stepButtonLift: ",step);
        telemetry.addData("fl motor target", myDrivetrain.flMot.getTargetPosition());
        telemetry.addData("bl motor target", myDrivetrain.blMot.getTargetPosition());
        telemetry.addData("fr motor target", myDrivetrain.frMot.getTargetPosition());
        telemetry.addData("br motor target", myDrivetrain.brMot.getTargetPosition());
        telemetry.addData("armMotor",myArmMotor.armMot.getCurrentPosition());

    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        myArmMotor.armMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //myDrivetrain.moveForwardInches(18);
        //myDrivetrain.setTargetHeading(-90);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        telemetry.addData("heading",myDrivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("stepButtonLift: ",step);
        telemetry.addData("fl motor target", myDrivetrain.flMot.getTargetPosition());
        telemetry.addData("bl motor target", myDrivetrain.blMot.getTargetPosition());
        telemetry.addData("fr motor target", myDrivetrain.frMot.getTargetPosition());
        telemetry.addData("br motor target", myDrivetrain.brMot.getTargetPosition());
        telemetry.addData("armMotor",myArmMotor.armMot.getCurrentPosition());
        telemetry.addData("servo pos",wristServo.getPosition());



        /* move forward, turn left, move forward, turn towards the basket, move forward,
         * extend arm motor, extend linear slide, spit out block, turn, drive to chamber,
         * turn, keep going, turn, move towards rung, move arm to touch low rung */

        switch(step){
            case 0:
                // stops and resets all encoders
                myDrivetrain.setMotSRE();       // clear the encoders
                step = 5;
                break;
            case 5:                            //forward 65 inches
                stepDone = myDrivetrain.moveForwardInches(65);
                if(stepDone){
                    step = 15;
                }
                break;
            case 15:                             // retract the wrist
                wristServo.setPosition(0.9);
                stepDone = (wristServo.getPosition() == 0.9);
                if(stepDone){
                    step = 16;
                }
                break;
            case 16:                             // lift the arm some
                stepDone = myArmMotor.armGoToAngle(-600);
                if (stepDone){
                    step = 17;
                }
                break;
            case 17:                             // center the wrist
                wristServo.setPosition(0.5);
                stepDone = (wristServo.getPosition() == 0.5);
                if (stepDone){
                    step = 20;
                }
                break;
            case 20:                            // turn to basket
                stepDone = myDrivetrain.turnToHeading(135, Drivetrain.Turn.LEFT);
                if (stepDone){
                    step = 30;
                }
                break;
            case 30:
                //electric slide on over to the basket diagonally while extending the lift and the arm
                stepDone = myDrivetrain.moveDiagonal(59);
                stepDone2 = myLift.liftTransit(4200);
                stepDone3 = myArmMotor.armGoToAngle(-3220);
                if (stepDone && stepDone2 && stepDone3){
                    step = 40;
                }
                break;
            case 40:                            //spit
                stepDone = myCrServo.spit(1,time);
                if (stepDone){
                    step = 50;
                }
                break;
            case 50:                            //turn 20 block #2 and move up the block again
                stepDone = myArmMotor.armGoToAngle(-3000);
                stepDone2 = myDrivetrain.turnToHeading(0, Drivetrain.Turn.RIGHT);
                if (stepDone && stepDone2){
                    step = 60;
                }
                break;
            case 60:                            //move the arm and lift into block pickup position
                stepDone = myArmMotor.armGoToAngle(4000);
                stepDone2 = myLift.liftTransit(0);
                if (stepDone && stepDone2){
                    step = 70;
                }
                break;
            case 70:                            //scoot into grabbing position
                stepDone = myDrivetrain.moveForwardInches(6);
                if (stepDone){
                    step = 80;
                }
                break;
            case 80:                            //grab that thang
                stepDone = myCrServo.suck(3, time);
                if (stepDone){
                    step = 85;
                }
                break;
            case 85:
                stepDone = myDrivetrain.moveForwardInches(-6);
                if (stepDone) {
                    step = 90;
                }
                break;
            case 90:                            //turn back to basket and go back to spit position
                stepDone = myDrivetrain.turnToHeading(135, Drivetrain.Turn.LEFT);
                stepDone2 = myLift.liftTransit(4200);
                stepDone3 = myArmMotor.armGoToAngle(-3220);
                if (stepDone && stepDone2 && stepDone3){
                    step = 100;
                }
                break;
            case 100:
                stepDone = myCrServo.spit(1,time);
                if (stepDone){
                    step = 110;
                }
                break;
            case 110:
                stepDone = myDrivetrain.turnToHeading(-35, Drivetrain.Turn.RIGHT);
                stepDone2 = myLift.liftTransit(0);
                if (stepDone && stepDone2){
                    step = 120;
                }
                break;
            case 120:
                stepDone = myDrivetrain.moveForwardInches(35);
                if (stepDone){
                    step = 130;
                }
                break;
            case 130:
                stepDone = myDrivetrain.turnToHeading(-90, Drivetrain.Turn.RIGHT);
                if (stepDone){
                    step = 140;
                }
            case 140:
                stepDone = myDrivetrain.moveForwardInches(6);
                if (stepDone){
                    step = 150;
                }
                break;
            case 150:
                stepDone = myArmMotor.armGoToAngle(4000);
                if (stepDone){
                    step = 160;
                }
                break;
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
