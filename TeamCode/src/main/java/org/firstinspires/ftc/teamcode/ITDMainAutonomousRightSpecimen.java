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
import com.qualcomm.robotcore.hardware.Servo;

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
public class ITDMainAutonomousRightSpecimen extends OpMode
{
    private Drivetrain myDrivetrain;
    private ArmMotor myArmMotor;
    private Lift myLift;
    private Servo wristServo = null;
    private CrServo myCrServo;
    int step = 0;
    boolean stepDone = false;
    @Override
    public void init() {
        myDrivetrain = new Drivetrain(hardwareMap, 0);
        myArmMotor = new ArmMotor(hardwareMap);
        myLift = new Lift(hardwareMap);
        myCrServo = new CrServo(hardwareMap);
        wristServo = hardwareMap.servo.get("wristServo");


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("heading",myDrivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
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

        switch(step){
            case 0:
                myDrivetrain.setMotSRE();       // clear the encoders
                step = 10;
                break;
            case 10://forwards to the bar
                stepDone = myDrivetrain.moveForwardInches(4,0.3);
                if(stepDone){
                    step = 15;
                }
                break;
            case 15:                             // retract the wrist
                wristServo.setPosition(0.5);
                stepDone = (wristServo.getPosition() == 0.5);
                if(stepDone){
                    step = 16;
                }
                break;
            case 16:                             // lift the arm some
                stepDone = myArmMotor.armGoToAngle(600);
                if (stepDone){
                    step = 20;
                }
                break;
            case 20: //move up the lift
                stepDone = myLift.liftTransit(2070);
                if (stepDone){
                    step = 30;
                }
                break;
            case 30: //move arm to the bar to hang the specimen
                stepDone = myArmMotor.armGoToAngle(4200);
                if(stepDone){
                    step = 33;
                }
                break;
                //TODO: change step back to like 40
            case 33:
                stepDone = myDrivetrain.moveForwardInches(17,0.3);
                if (stepDone){
                    step = 40;
                }
                break;
            case 40: //put the arm back
                stepDone = myCrServo.spit(1, time);
                if(stepDone){
                    step = 43;
                }
            case 43:
                stepDone = myDrivetrain.moveReverseInches(15);
                if (stepDone){
                    step = 50;
                }
                break;
            case 50: //put the lift to the bottom
                stepDone = myLift.liftTransit(0);
                if(stepDone){
                    step = 55;
                }
                break;
            case 55:
                stepDone = myArmMotor.armGoToAngle(400);
                if (stepDone){
                    step = 60;
                }
                break;
            case 60: // turn towards corner
                stepDone = myDrivetrain.turnToHeading(-90, Drivetrain.Turn.RIGHT);
                if(stepDone){
                    step = 70;
                }
                break;
            case 70: // move towards the corner
                stepDone = myDrivetrain.moveForwardInches(48,0.3);
                if (stepDone){
                    step = 80;
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
