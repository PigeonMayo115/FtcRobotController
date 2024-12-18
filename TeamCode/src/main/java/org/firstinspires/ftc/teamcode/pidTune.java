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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
welcome!

This is the 3rd iteration of the main TeleOp program for the 2024-25
into the deep season. the main differences between this mode and V2 is that
I added in some functionality to give us the position relative to the starting position of the robot using out new fancy
odometry computer and wheels, I re adapted the opmode to be compatible with the new wormdrives, and I added henry's request to do
whenever you do the dpad on gamepad 2 it goes up or down as a button for added consistency when scoring.

Last updated on 12/5/2024

 */

@TeleOp
public class pidTune extends OpMode
{
    // Declare OpMode members (initlaize primitve variables and set up servos that we don't initlaize in the class).
    private ElapsedTime runtime = new ElapsedTime();
    CRServo crServo = null;
    Servo wristServo = null;
    DcMotor armMotor = null;
    int positionArmMotor = 0;
    double frontLeftPower = 0;
    double backLeftPower = 0;
    double frontRightPower = 0;
    double backRightPower = 0;
    private Lift myLift;

    private Drivetrain myDrivetrain;
    private ArmMotor myArmMotor;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        //this is where we initialize all of our classes and motors and such
        crServo = hardwareMap.crservo.get("crServoRubberWheel");
        wristServo = hardwareMap.servo.get("wristServo");
        armMotor = hardwareMap.dcMotor.get("armMotor");

        myLift = new Lift(hardwareMap);     // New instance of the "lift" class
        // true = school, false = home
        myDrivetrain = new Drivetrain(hardwareMap,0); // New instance of the "Drivetrain" class
        myArmMotor = new ArmMotor(hardwareMap);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        positionArmMotor = armMotor.getCurrentPosition();
        armMotor.setTargetPosition(positionArmMotor);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // all of this stuff is for the odometry computer, this stuff is all copied over from SensorGoBildaPinpointExample,
        // look there for more questions

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo"); //initiallize odometry computer
        //TODO: figure out where the odometry computer is relative to the center of the robot
        //feeds the pinpoint computer the position in relation to the center of the robot
        //for geometry purposes
        odo.setOffsets(40, 0);
        //feeds odometry computer the model of odometry pods you are using
        //do not change this for the ITD season unless we decide to change what odometry wheels we are using
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //sets the directions for the odometry wheels so they don't encode backwards
        //TODO: figure out what way is forwards and backwards on the pod
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //resets the position of the odo computer
        odo.resetPosAndIMU();

        myLift.LSMRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myLift.LSMLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



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
        //makes the arm motor and linear slide motors hold their positions when start is pressed
        positionArmMotor = armMotor.getCurrentPosition();
        armMotor.setTargetPosition(positionArmMotor);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        myLift.holdBottom();    // Hang on!

    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // set variables for controller inputs
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
        double ly2 = gamepad2.left_stick_y;
        double ry2 = gamepad2.right_stick_y;
        boolean a_button = gamepad2.a;
        boolean b_button = gamepad2.b;
        double leftTriggerRaw = gamepad2.left_trigger;
        double rightTriggerRaw = gamepad2.right_trigger;
        boolean dpadUp1 = gamepad1.dpad_up;
        boolean dpadDown1 = gamepad1.dpad_down;
        boolean dpadLeft1 = gamepad1.dpad_left;
        boolean dpadRight1 = gamepad1.dpad_right;
        double speedModifier = 1-(gamepad1.left_trigger*0.8);


        //arm motor logic!
        //myArmMotor.armMotStickControl(ly2);
        if(gamepad2.x){
        myArmMotor.pidControl(myArmMotor.armMot.getCurrentPosition(), 2000);
        } else{
            myArmMotor.armMotStickControl(ly2);
        }
        if (gamepad2.dpad_up) {
            myArmMotor.KpTuneUp();
        }
        if (gamepad2.dpad_down){
            myArmMotor.KpTuneDown();
        }
        if (gamepad2.dpad_right){
            myArmMotor.KiTuneUp();
        }
        if (gamepad2.dpad_left){
            myArmMotor.KiTuneDown();
        }
        if (gamepad2.a){
            myArmMotor.KpTuneUp();
        }
        if (gamepad2.b){
            myArmMotor.KdTuneDown();
        }
        telemetry.addData("armMotor",armMotor.getCurrentPosition());
        telemetry.addData("dpad up=kp up", "");
        telemetry.addData("dpad right=ki up", "");
        telemetry.addData("a=kd up", "");
        telemetry.addData("KP value", myArmMotor.Kp);
        telemetry.addData("KI value", myArmMotor.Ki);
        telemetry.addData("KD value", myArmMotor.Kd );
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
