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

package org.firstinspires.ftc.teamcode.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.custom.ArmMotor;
import org.firstinspires.ftc.teamcode.custom.CrServo;
import org.firstinspires.ftc.teamcode.custom.Drivetrain;
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

@Autonomous
@Disabled
public class ITDMainAutonomousLeftSampleV3 extends OpMode
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

    double firstPickupX = 12;
    double firstPickupY = 16.45;

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
        telemetry.addData("step: ",step);
        telemetry.addData("fl motor target", myDrivetrain.flMot.getTargetPosition());
        telemetry.addData("bl motor target", myDrivetrain.blMot.getTargetPosition());
        telemetry.addData("fr motor target", myDrivetrain.frMot.getTargetPosition());
        telemetry.addData("br motor target", myDrivetrain.brMot.getTargetPosition());
        telemetry.addData("armMotor",myArmMotor.armMot.getCurrentPosition());
        telemetry.addData("servo pos",wristServo.getPosition());

        myDrivetrain.odo.update();

        Pose2D pos = myDrivetrain.odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);



        /* move forward, turn left, move forward, turn towards the basket, move forward,
         * extend arm motor, extend linear slide, spit out block, turn, drive to chamber,
         * turn, keep going, turn, move towards rung, move arm to touch low rung */

        switch(step) {
            case 0:
                // stops and resets all encoders
                myDrivetrain.setMotSRE();       // clear the encoders
                step = 5;
                break;
            case 5:                            //forward 65 inches
                stepDone = myDrivetrain.moveForwardInches(12, 0.3);
                wristServo.setPosition(0.5);
                if (stepDone) {
                    step = 16;
                }
                break;
            case 16:                             // lift the arm some
                stepDone = myArmMotor.armGoToAngle(600);
                if (stepDone) {
                    step = 30;
                }
                break;
            case 30:
                stepDone = myDrivetrain.turnToHeading(90, Drivetrain.Turn.LEFT);
                if (stepDone) {
                    step = 40;
                }
                break;
            case 40:
                stepDone = myDrivetrain.moveForwardInches(26, 0.3);
                if (stepDone) {
                    step = 43;
                }
                break;
            case 43:
                stepDone = myDrivetrain.turnToHeading(135, Drivetrain.Turn.LEFT);
                if (stepDone) {
                    step = 44;
                }
                break;
            case 44:
                stepDone = myDrivetrain.moveForwardInches(4,0.3);
                if (stepDone) {
                    step = 45;
                }
                break;
            case 45:
                myLift.liftTransit(2550);
                stepDone = myArmMotor.armGoToAngle(3000,0.3);
                if (stepDone) {
                    step = 50;
                }
                break;
            case 50:
                stepDone = myCrServo.spit(1, time);
                if (stepDone) {
                    step = 55;
                }
                break;
            case 55:
                stepDone = myDrivetrain.turnToHeading(0, Drivetrain.Turn.RIGHT);
                myLift.liftTransit(0);
                if (stepDone) {
                    step = 60;
                }
                break;
            case 60:
                stepDone = myDrivetrain.moveForwardInches(8, 0.3);
                if (stepDone) {
                    step = 80;
                }
                break;
            case 80:
                stepDone = myDrivetrain.strafeRightInches(3);
                if (stepDone) {
                    step = 82;
                }
                break;

            //position check/correction before pickup of the next block
            case 82:
                //check x
                if (pos.getX(DistanceUnit.INCH) < firstPickupX){
                    //forward
                    step = 83;
                }else{
                    //reverse
                    step = 84;
                }
                break;
            case 83:
                stepDone = myDrivetrain.forwardToX(pos,firstPickupX);
                if (stepDone){
                    step = 87;
                }
                break;
            case 84:
                stepDone = myDrivetrain.reverseToX(pos, firstPickupX);
                if (stepDone){
                    step = 87;
                }
                break;
            case 87:
                //check Y
                if (pos.getY(DistanceUnit.INCH) > firstPickupY){
                    //right
                    step = 89;
                }else{
                    //left
                    step = 88;
                }
                break;
            case 88:
                stepDone = myDrivetrain.leftToY(pos, firstPickupY);
                if (stepDone){
                    step = 93;
                }
                break;
            case 89:
                stepDone = myDrivetrain.rightToY(pos, firstPickupY);
                if (stepDone){
                    step = 93;
                }
                break;
            case 93:
                stepDone = myArmMotor.armGoToAngle(5000,0.3);
                if(stepDone){
                    step=95;
                }
                break;
            case 95:
                stepDone = myCrServo.suck(3, time);
                if (stepDone) {
                    step = 100;
                }
                break;
            case 100:
                stepDone = myDrivetrain.turnToHeading(135, Drivetrain.Turn.LEFT);
                myLift.liftTransit(2500);
                myArmMotor.armGoToAngle(3000);
                if (stepDone){
                    step = 110;
                }
                break;
            case 110:
                stepDone = myDrivetrain.moveForwardInches(9,0.3);
                if (stepDone){
                    step = 120;
                }
                break;
            case 120:
                stepDone = myCrServo.spit(2,time);
                if (stepDone){
                    step = 130;
                }
                break;
            case 130:
                stepDone = myDrivetrain.turnToHeading(0, Drivetrain.Turn.RIGHT);
                myLift.liftTransit(0);
                if (stepDone){
                    step = 140;
                }
                break;
            case 140:
                stepDone = myDrivetrain.moveForwardInches(8,0.3);
                if (stepDone){
                    step = 180;
                }
                break;
            case 180:
                stepDone = myArmMotor.armGoToAngle(5000, 0.3);
                if (stepDone){
                    step = 190;
                }
                break;
            case 190:
                stepDone = myDrivetrain.turnToHeading(135, Drivetrain.Turn.LEFT);
                myLift.liftTransit(2500);
                myArmMotor.armGoToAngle(3000);
                if (stepDone){
                    step = 200;
                }
                break;
            case 200:
                stepDone = myDrivetrain.moveForwardInches(9,0.3);
                if (stepDone){
                    step = 210;
                }
                break;
            case 210:
                stepDone = myCrServo.spit(3,time);
                if (stepDone){
                    step = 220;
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
