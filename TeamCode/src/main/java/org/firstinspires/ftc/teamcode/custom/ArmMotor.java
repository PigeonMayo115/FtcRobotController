package org.firstinspires.ftc.teamcode.custom;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmMotor {
    // initialize variables

    public DcMotor armMot = null;
    private int armMotPos = 0;
    private int armMotPosMax = 0;
    private int armMotPosMin = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private boolean setPointReached = false;
    public double Kp = 0;
    public double Ki = 0;
    public double Kd = 0;
    //resets encoders, sets mins and maxes
    public ArmMotor(HardwareMap hwMap) {
        armMot = hwMap.dcMotor.get("armMotor");
        armMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMot.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotPosMin = armMot.getCurrentPosition();
        armMotPosMax = armMot.getCurrentPosition() + 300;


    }

    public void armMotStickControl(double cmd){
        if (cmd == 0){
            armMot.setTargetPosition(armMotPos);
            armMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
            armMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMot.setPower(-cmd*0.75);
            armMotPos = armMot.getCurrentPosition();
        }
    }
    public void armMotHoldPos(int pos){
        armMot.setPower(1);
        armMot.setTargetPosition(pos);
        armMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // sets the target position of the arm motor to the given value
    public boolean  armGoToAngle(int targetTicks) {
        armMot.setPower(1);
        armMot.setTargetPosition(targetTicks);
        armMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (targetTicks == armMot.getCurrentPosition()) {
            return true;
        } else {
            return false;
        }

    }
    public void pidControl(int armMotPos, int targetPos){
        /*

         * Proportional Integral Derivative Controller

         */





        if (armMotPos == targetPos){
            setPointReached = true;
        }else{
            setPointReached = false;
        }
// Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();

        while (setPointReached == false) {


            // calculate the error
            double error = targetPos - armMotPos;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            armMot.setPower(out);

            lastError = error;

            // reset the timer for next time
            timer.reset();

        }
    }
    public void KpTuneUp(){
        Kp = Kp + .05;
    }
    public void KpTuneDown(){
        Kp = Kp - .05;
    }
    public void KiTuneUp(){
        Ki = Ki + .05;
    }
    public void KiTuneDown(){
        Ki = Ki - .05;
    }
    public void KdTuneUp(){
        Kd = Kd + .05;
    }
    public void KdTuneDown(){
        Kd = Kd - .05;
    }
}
