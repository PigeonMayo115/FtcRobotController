
package org.firstinspires.ftc.teamcode.custom;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Drivetrain {
    public DcMotor flMot;
    public DcMotor blMot;
    public DcMotor frMot;
    public DcMotor brMot;
    public IMU imu;
    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    public double targetHeading;
    public int targetDistance;
    int encoderResolution;
    double ticksPerInch;
    double ticksPerInchStrafe;
    double xDifference;
    double yDifference;
    double xPower;
    double yPower;
    int moveToCoordinateState = 0;

    // Enum used to track which robot we're running on now
    // TODO: Select the robot programmatically.
    public enum Robot{
        BOGG, ELIOT, MERRICK
    }

    // Enum for turn direction
    public enum Turn{
        LEFT, RIGHT
    }

    // Overload constructor for robotConfig by int
    public Drivetrain(HardwareMap hwMap, int robotConfig){
        this(hwMap, getWhichRobot(robotConfig));
    }

    // Constructor
    public Drivetrain(HardwareMap hwMap, Robot robotConfig) {

        flMot = hwMap.dcMotor.get("frontLeftMotor");
        blMot = hwMap.dcMotor.get("backLeftMotor");
        frMot = hwMap.dcMotor.get("frontRightMotor");
        brMot = hwMap.dcMotor.get("backRightMotor");

        frMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RevHubOrientationOnRobot revOrientation;

        odo = hwMap.get(GoBildaPinpointDriver.class,"odo"); //initiallize odometry computer
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

        // Motor Setup
        switch (robotConfig){
            case BOGG:
                flMot.setDirection(DcMotorSimple.Direction.REVERSE);
                blMot.setDirection(DcMotorSimple.Direction.FORWARD);
                frMot.setDirection(DcMotorSimple.Direction.FORWARD);
                brMot.setDirection(DcMotorSimple.Direction.REVERSE);
                encoderResolution= 550; // for bogg (yellowJacket) the resolution is 550
                break;
            case MERRICK:
                flMot.setDirection(DcMotorSimple.Direction.REVERSE);
                blMot.setDirection(DcMotorSimple.Direction.REVERSE);
                frMot.setDirection(DcMotorSimple.Direction.FORWARD);
                brMot.setDirection(DcMotorSimple.Direction.FORWARD);
                encoderResolution= 550;
                break;
            case ELIOT:
                flMot.setDirection(DcMotorSimple.Direction.REVERSE);
                blMot.setDirection(DcMotorSimple.Direction.FORWARD);
                frMot.setDirection(DcMotorSimple.Direction.FORWARD);
                brMot.setDirection(DcMotorSimple.Direction.FORWARD);
                encoderResolution= 440; // for the rev motors, the resolution is 440
                break;
        }
        ticksPerInchStrafe = encoderResolution*1.2/(4.1*Math.PI);
        //ticksPerInchStrafe = 41.34;
        ticksPerInch = encoderResolution/(4.1*Math.PI);
        frMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        blMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        flMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        brMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        // IMU Setup
        switch(robotConfig){
            case ELIOT:
                imu = hwMap.get(IMU.class, "imu");
                revOrientation = new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
                imu.initialize(new IMU.Parameters(revOrientation));
                break;
            case BOGG:
                imu = hwMap.get(IMU.class, "imu");
                revOrientation = new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
                imu.initialize(new IMU.Parameters(revOrientation));
                break;
            case MERRICK:       // Uses the Sparkfun OTOS IMU
                /*revOrientation = new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
                IMU sfIMU = new SparkFunIMU(hwMap, "sensor_otos", revOrientation);
                hwMap.put("imu", sfIMU);
                imu = hwMap.get(IMU.class, "imu");
                imu.initialize(new IMU.Parameters(revOrientation)); */
                break;
        }

    }

    private static Robot getWhichRobot(int robotNum){
        Robot whichRobot;
        switch (robotNum) {       // 0 = bogg, 1 = home, 2 = eliot
            case 0:
                whichRobot = Robot.BOGG;
                break;
            case 1:
                whichRobot = Robot.MERRICK;
                break;
            case 2:
                whichRobot = Robot.ELIOT;
                break;
            default:
                whichRobot = Robot.BOGG;
                break;
        }
        return whichRobot;
    }

    public void driveLeft(double spdMult) {
        setMotPow(1, 1, 1, 1, spdMult);
    }

    public void driveRight(double spdMult) {
        setMotPow(-1, 1, -1, 1, spdMult);
    }

    public void driveForward(double spdMult) {
        setMotPow(1, -1, 1, -1, spdMult);
    }

    public void driveReverse(double spdMult) {
        setMotPow(-1, 1, -1, -1, spdMult);
    }

    public double getHeading(AngleUnit angleUnit){
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    // TODO: Get rid of this
    public void setTargetHeading(double heading){
        return;
    }

    // Overload: If direction is not specified, always turn left.
    public boolean turnToHeading(double heading){
        return turnToHeading(heading, Turn.LEFT);
    }

    //turn to the specified heading by turning in the specified direction
    //  If turn is left, done when heading is > target
    //  If turn is right, done when heading is < target
    //  Special case when crossing over 180:
    //  - If Turning Left, and target < starting or if turning right, and target > starting
    //  - Then must turn past 180 before looking for done.
    public boolean turnToHeading(double heading, Turn direction){

        double overShootAdjuster = 11;        // seems to overshoot by 11 degrees
        double currentHeading = getHeading(AngleUnit.DEGREES);
        setMotRUE();                            // Run Using Encoder

        if(direction == Turn.LEFT && heading < currentHeading){
            setMotPow(-0.5,-0.5,0.5,0.5,1);
            return false;
        }
        if (direction == Turn.RIGHT && heading > currentHeading) {
            setMotPow(0.5,0.5,-0.5,-0.5,1);
            return false;
        }

        if(direction == Turn.LEFT){
            //we are turning left
            if(getHeading(AngleUnit.DEGREES)<(heading-overShootAdjuster)){
                setMotPow(-0.5,-0.5,0.5,0.5,1);
                return false;
            } else {
                setMotPow(0,0,0,0,0);
                return true;
            }
        }
        if (direction == Turn.RIGHT){
            //we are turning right
            if(getHeading(AngleUnit.DEGREES)>(heading+overShootAdjuster)){
                setMotPow(0.5,0.5,-0.5,-0.5,1);
                return false;
            } else {
                setMotPow(0,0,0,0,0);
                return true;
            }
        } else {
            return false;
        }
    }

    public boolean turnToHeadingV2(double heading, Turn direction){

        double overShootAdjuster = 11;        // seems to overshoot by 11 degrees
        double currentHeading = getHeading(AngleUnit.DEGREES);
        setMotRUE();                            // Run Using Encoder

        if(direction == Turn.LEFT && heading < currentHeading){
            setMotPow(-0.5,-0.5,0.5,0.5,1);
            return false;
        }
        if (direction == Turn.RIGHT && heading > currentHeading) {
            setMotPow(0.5,0.5,-0.5,-0.5,1);
            return false;
        }

        if(direction == Turn.LEFT){
            //we are turning left
            if(getHeading(AngleUnit.DEGREES)<(heading-5)){
                setMotPow(-0.05,-0.05,0.05,0.05,1);
                return false;
            } else if (getHeading(AngleUnit.DEGREES)<(heading-10)){
                setMotPow(-0.1,-0.1,0.1,0.1,1);
                return false;
            }else if (getHeading(AngleUnit.DEGREES)<(heading-30)) {
                setMotPow(-0.3, -0.3, 0.3, 0.3, 1);
                return false;
            } else if (getHeading(AngleUnit.DEGREES)<(heading)) {
                setMotPow(-0.6, -0.6, 0.6, 0.6, 1);
                return false;
            }else {
                setMotPow(0,0,0,0,0);
                return true;
            }
        }
        if (direction == Turn.RIGHT){
            //we are turning right
            if(getHeading(AngleUnit.DEGREES)<(heading+5)){
                setMotPow(0.05,0.05,-0.05,-0.05,1);
                return false;
            } else if (getHeading(AngleUnit.DEGREES)<(heading+10)){
                setMotPow(0.1,0.1,-0.1,-0.1,1);
                return false;
            }else if (getHeading(AngleUnit.DEGREES)<(heading+30)) {
                setMotPow(0.3, 0.3, -0.3, -0.3, 1);
                return false;
            } else if (getHeading(AngleUnit.DEGREES)<(heading)) {
                setMotPow(0.6, 0.6, -0.6, -0.6, 1);
                return false;
            }else {
                setMotPow(0,0,0,0,0);
                return true;
            }
        } else {
            return false;
        }
    }

    //All the <direction> to <X or Y> methods require you to be at heading 0 degrees
    //DO NOT TRY TO DO IT OTHERWISE OR IT WONT WORK
    public boolean forwardToX (Pose2D pose2D, double targetX){
        if (pose2D.getX(DistanceUnit.INCH) < targetX){
            setMotPow(0.3,0.3,0.3,0.3,1);
            return false;
        } else if (pose2D.getX(DistanceUnit.INCH) >= targetX){
            setMotPow(0,0,0,0,1);
            return true;
        }
        return false;
    }

    public boolean reverseToX (Pose2D pose2D, double targetX){
        if (pose2D.getX(DistanceUnit.INCH) > targetX){
            setMotPow(-0.3,-0.3,-0.3,-0.3,1);
            return false;
        } else if (pose2D.getX(DistanceUnit.INCH) <= targetX){
            setMotPow(0,0,0,0,1);
            return true;
        }
        return false;

    }

    public boolean leftToY (Pose2D pose2D,double targetY){
        if (pose2D.getY(DistanceUnit.INCH) > targetY){
            setMotPow(-0.3,0.3,0.3,-0.3,1);
            return false;
        } else if (pose2D.getY(DistanceUnit.INCH) <= targetY){
            setMotPow(0,0,0,0,1);
            return true;
        }
        return false;
    }

    public boolean rightToY (Pose2D pose2D,double targetY){
        if (pose2D.getY(DistanceUnit.INCH) < targetY){
            setMotPow(0.3,-0.3,-0.3,0.3,1);
            return false;
        } else if (pose2D.getY(DistanceUnit.INCH) >= targetY){
            setMotPow(0,0,0,0,1);
            return true;
        }
        return false;

    }

    public boolean dumbTurn(double degrees){
        setMotRUE();
        if (getHeading(AngleUnit.DEGREES)==degrees){
            setMotPow(0,0,0,0,0);
            return true;
        }else{
            setMotPow(0.1,0.1,-0.1,-0.1,1 );
            return false;
        }
    }


    public void stickDrive(double xCmd, double yCmd, double rxCmd, double spdMult, int robotConfig)
    // 0 = bogg, 1 = home robot, 2 = eliot
    {
        double denominator = Math.max(Math.abs(yCmd) + Math.abs(xCmd) + Math.abs(rxCmd), 1);
        xCmd = xCmd*-1;

        if (robotConfig == 0) {
            setMotPow(
                    (yCmd + xCmd - rxCmd) / denominator,
                    (yCmd - xCmd - rxCmd) / denominator,
                    (yCmd - xCmd + rxCmd) / denominator,
                    (yCmd + xCmd + rxCmd) / denominator,
                    spdMult);
        } else if (robotConfig == 1){
            setMotPow(
                    (yCmd + xCmd + rxCmd) / denominator,
                    (yCmd - xCmd - rxCmd) / denominator,
                    (yCmd - xCmd + rxCmd) / denominator,
                    (yCmd + xCmd - rxCmd) / denominator,
                    spdMult);
        } else if (robotConfig == 2){
            setMotPow(
                    (yCmd + xCmd + rxCmd) / denominator,
                    (yCmd - xCmd - rxCmd) / denominator,
                    (yCmd - xCmd + rxCmd) / denominator,
                    (yCmd + xCmd - rxCmd) / denominator,
                    spdMult);
        }


    }


    public void setMotPow(double flMotPow, double blMotPow, double frMotPow, double brMotPow, double spdMult) {
        flMot.setPower(flMotPow * spdMult);
        blMot.setPower(blMotPow * spdMult);
        frMot.setPower(frMotPow * spdMult);
        brMot.setPower(brMotPow * spdMult);

    }

    public void fullDrive(double x, double y, double rx, double spdMult, boolean up, boolean down, boolean left, boolean right) {
        if (!(y == 0) || !(x == 0) || !(rx == 0)) {
            this.stickDrive(x, y, rx, spdMult, 0);
        } else if (up || down || left || right) {
            if (up) {
                this.driveForward(spdMult);
            } else if (down) {
                this.driveReverse(spdMult);
            } else if (right) {
                this.driveRight(spdMult);
            } else if (left) {
                this.driveLeft(spdMult);
            }
        } else {
            this.setMotPow(0, 0, 0, 0, 0);
        }
    }
    public boolean moveForwardInches(int distance, double power) {
        int distanceTicks;
        if (power == 0){
            power = 0.3;
        }
        if (targetDistance == 0) {        // Move not started yet
            setMotSRE();                // Clear the encoders
            targetDistance = distance;
            return false;
        } else {
            distanceTicks = (int) (distance * ticksPerInch);
            flMot.setTargetPosition(distanceTicks);
            blMot.setTargetPosition(distanceTicks);
            frMot.setTargetPosition(distanceTicks);
            brMot.setTargetPosition(distanceTicks);
            setMotRTP();
            if (flMot.getCurrentPosition() >= distanceTicks) {       // all done
                setMotPow(0, 0, 0, 0, 0);
                targetDistance = 0;
                return true;
            } else {                                                // run it forward
                this.setMotPow(0.3, 0.3, 0.3, 0.3, 1);
                return false;
            }
        }
    }
    //input a positive value for this method
    public boolean moveReverseInches(int distance) {
        int distanceTicks;
        if (targetDistance == 0) {        // Move not started yet
            setMotSRE();                // Clear the encoders
            targetDistance = distance;
            return false;
        } else {
            distanceTicks = (int) (-distance * ticksPerInch);
            flMot.setTargetPosition(distanceTicks);
            blMot.setTargetPosition(distanceTicks);
            frMot.setTargetPosition(distanceTicks);
            brMot.setTargetPosition(distanceTicks);
            setMotRTP();
            if (flMot.getCurrentPosition() <= distanceTicks) {       // all done
                setMotPow(0, 0, 0, 0, 0);
                targetDistance = 0;
                return true;
            } else {                                                // run it forward
                this.setMotPow(-0.3, -0.3, -0.3, -0.3, 1);
                return false;
            }
        }
    }

        public boolean moveDiagonal(int distance){
            int distanceTicks;
            if (targetDistance == 0) {        // Move not started yet
                setMotSRE();                // Clear the encoders
                targetDistance = distance;
                return false;
            } else {
                distanceTicks = (int) (distance * ticksPerInch);
                flMot.setTargetPosition(distanceTicks);
                blMot.setTargetPosition(blMot.getTargetPosition());
                frMot.setTargetPosition(frMot.getCurrentPosition());
                brMot.setTargetPosition(distanceTicks);
                setMotRTP();
                if (flMot.getCurrentPosition() >= distanceTicks) {       // all done
                    setMotPow(0, 0, 0, 0, 0);
                    targetDistance = 0;
                    return true;
                } else {                                                // run it diagonal
                    setMotPow(0.3, 0, 0, 0.3, 1);
                    return false;
                }
            }
        }

        public boolean strafeRightInches (int distance){
            if (targetDistance == 0) {        // Move not started yet
                setMotSRE();                // Clear the encoders
                targetDistance = distance;
                return false;
            } else {
                int distanceTicks = (int) (distance * ticksPerInchStrafe);
                flMot.setTargetPosition(distanceTicks);
                blMot.setTargetPosition(-distanceTicks);
                frMot.setTargetPosition(-distanceTicks);
                brMot.setTargetPosition(distanceTicks);
                setMotRTP();
                if ((flMot.getCurrentPosition()) >= distanceTicks) {       // all done
                    setMotPow(0, 0, 0, 0, 0);
                    targetDistance = 0;
                    return true;
                } else {                                                // run it diagonal
                    setMotPow(0.3, 0.3, 0.3, 0.3, 1);
                    return false;
                }
            }

        }

    public boolean strafeLeftInches (int distance) {
        int distanceTicks;
        if (targetDistance == 0) {        // Move not started yet
            setMotSRE();                // Clear the encoders
            targetDistance = distance;
            return false;
        } else {
            distanceTicks = (int) (distance * ticksPerInchStrafe);
            flMot.setTargetPosition(-distanceTicks);
            blMot.setTargetPosition(distanceTicks);
            frMot.setTargetPosition(distanceTicks);
            brMot.setTargetPosition(-distanceTicks);
            setMotRTP();
            if (-(flMot.getCurrentPosition()) >= distanceTicks) {       // all done
                setMotPow(0, 0, 0, 0, 0);
                targetDistance = 0;
                return true;
            } else {
                setMotPow(0.3, 0.3, 0.3, 0.3, 1);
                return false;
            }
        }
    }

            //This is a method that uses our highly anticipated odometry pods to translate our
            //position to a new location. this is in INCHES. NOT UNITS BUT INCHES
        public boolean moveToCoordinates (double xTarget, double yTarget, Pose2D pos, double maxSpeed){
            //here is what the method needs to do
            //get our coordinates (x,y) and compare them to the target coordinates
            //find the x distance and the y distance
            pos = odo.getPosition();
            maxSpeed = 0.3;
            if (moveToCoordinateState == 0) {
                xDifference = xTarget - pos.getX(DistanceUnit.INCH);
                yDifference = yTarget - pos.getY(DistanceUnit.INCH);
                moveToCoordinateState = 1;
            }

            //make a ratio of x to y
            //to do this i could
            //1. compare the x from the y
            if (Math.abs(xDifference) > Math.abs(yDifference)){
                if (xDifference > 0){
                    xPower = 1;
                    yPower = yDifference/xDifference;
                }else if (xDifference < 0){
                    xPower = -1;
                    yPower = yDifference/xDifference;
                }
                stickDrive(xPower,yPower,0,maxSpeed,0);
            } else if (Math.abs(xDifference) < Math.abs(yDifference)){
                if (yDifference > 0){
                    yPower = 1;
                    xPower = xDifference/yDifference;
                }else if (xDifference < 0){
                    yPower = -1;
                    xPower = xDifference/yDifference;
                }
                stickDrive(xPower,yPower,0,maxSpeed,0);
            } else if (Math.abs(xDifference) == Math.abs(yDifference)){
                if (xDifference > 0 && yDifference > 0){
                    stickDrive(0.5,0.5,0,maxSpeed,0);
                } else if (xDifference < 0 && yDifference > 0) {
                    stickDrive(-0.5, 0.5, 0, maxSpeed, 0);
                } else if (xDifference < 0 && yDifference < 0) {
                    stickDrive(-0.5, -0.5, 0, maxSpeed, 0);
                } else if (xDifference > 0 && yDifference < 0) {
                    stickDrive(0.5, -0.5, 0, maxSpeed, 0);
                }
            }
            //2. if x is greater, divide y by x and vice versa
            //3. input both of those into stick drive
            //check if our current location is farther to our target location
            if (xDifference > 0 && yDifference > 0){
                if (pos.getX(DistanceUnit.INCH) > xTarget && pos.getY(DistanceUnit.INCH) > yTarget){
                    moveToCoordinateState = 0;
                    return true;
                }else {
                    return false;
                }
            } else if (xDifference < 0 && yDifference > 0) {
                if (pos.getX(DistanceUnit.INCH) < xTarget && pos.getY(DistanceUnit.INCH) > yTarget){
                    moveToCoordinateState = 0;
                    return true;
                }else {
                    return false;
                }
            } else if (xDifference < 0 && yDifference < 0) {
                if (pos.getX(DistanceUnit.INCH) < xTarget && pos.getY(DistanceUnit.INCH) < yTarget){
                    moveToCoordinateState = 0;
                    return true;
                }else {
                    return false;
                }
            } else if (xDifference > 0 && yDifference < 0) {
                if (pos.getX(DistanceUnit.INCH) > xTarget && pos.getY(DistanceUnit.INCH) < yTarget){
                    moveToCoordinateState = 0;
                    return true;
                }else {
                    return false;
                }
            }
            //if at or past location, return true
            //if not return false
            return false;
        }

    //TODO: make this method xD
    public void moveToPosition (double positionX, double positionY, double heading ) {
        //so to do this one we need to do:
        //1. Get our position information
        //2.
    }


    public void setMotRTP(){
        flMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setMotRUE(){
        flMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        blMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        frMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        brMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }

    public void setMotSRE(){
        flMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void singleMot (int whichMotor){
        if (whichMotor == 1 ){
            flMot.setPower(1);
        }
        if (whichMotor == 2 ){
            blMot.setPower(1);
        }
        if (whichMotor == 3 ){
            frMot.setPower(1);
        }
        if (whichMotor == 4 ){
            brMot.setPower(1);
        }
    }



}
