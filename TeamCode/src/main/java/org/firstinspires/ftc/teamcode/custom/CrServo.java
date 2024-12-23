package org.firstinspires.ftc.teamcode.custom;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CrServo {
    public CRServo crServoR = null;
    public CRServo crServoL = null;
    double spitStart = 0;
    double spitEnd = 0;
    double suckStart = 0;
    double suckEnd = 0;

    public CrServo(HardwareMap hwMap) {
        
        crServoR = hwMap.crservo.get("crServoR");
        crServoL = hwMap.crservo.get("crServoL");
    }

    public boolean suck (double howLong, double currentTime){
        if (suckEnd == 0){
            suckStart = currentTime;
            suckEnd = currentTime + howLong;
        }
        if (currentTime > suckStart + howLong){
            suckStart = 0;
            suckEnd = 0;
            crServoR.setPower(0);
            crServoL.setPower(0);
            return true;
        } else{
            crServoL.setPower(-1);
            crServoR.setPower(-1);
            return false;
        }
    }

    public boolean spit (double howLong, double currentTime){
       if (spitEnd == 0){
           spitStart = currentTime;
           spitEnd = currentTime + howLong;
       }
       if (currentTime > spitStart + howLong){
           spitStart = 0;
           spitEnd = 0;
           crServoL.setPower(0);
           crServoR.setPower(0);
           return true;
       } else{
           crServoL.setPower(0.3);
           crServoR.setPower(-0.3);
           return false;
       }
    }
    
    public void doubleCrServo (boolean a, boolean b){
        //input for continuous rotation servo with rubber wheel
        if (a && b ) {
             crServoL.setPower(0);
             crServoR.setPower(0);
        }
        else if (a ){
             crServoL.setPower(-1);
             crServoR.setPower(1);
        }
        else if (b ){
             crServoL.setPower(1);
             crServoR.setPower(-1);
        }
        else {
             crServoL.setPower(0);
             crServoR.setPower(0);
        }
    }

    public boolean stop(){
        crServoL.setPower(0);
        crServoR.setPower(0);
        return true;
    }
}
