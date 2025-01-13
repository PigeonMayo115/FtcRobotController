package org.firstinspires.ftc.teamcode.custom;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/** This is a dummy motor class meant to "simulate" or "fake" a motor in the case that we want
 *  to run code the works with a motor without the motor actually installed.
 *  The main purpose was to be able to run Bruce's ITD code on a robot that doesn't have
 *  a lift or an arm.
 *
 *  In order to use it, we'll have to add it to the hardware map before any calls to the
 *  associated get() call for the motor object.
 *
 *  JM 11/10/24
 */
public class DummyMotor implements DcMotor {

    private int currentPosition = 0;
    private int targetPosition = 0;
    private boolean powerFloat = false;
    private RunMode runMode = RunMode.RUN_WITHOUT_ENCODER;
    private Direction motDirection = Direction.FORWARD;
    private double motPower = 0.0;

    @Override
    public MotorConfigurationType getMotorType() {
        return null;
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {

    }

    @Override
    public DcMotorController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {

    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return null;
    }

    @Override
    public void setPowerFloat() {
        powerFloat = true;
    }

    @Override
    public boolean getPowerFloat() {
        return powerFloat;
    }

    @Override
    public void setTargetPosition(int position) {
        targetPosition = position;
    }

    @Override
    public int getTargetPosition() {
        return targetPosition;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public int getCurrentPosition() {
        return currentPosition;
    }

    @Override
    public void setMode(RunMode mode) {
        runMode = mode;
    }

    @Override
    public RunMode getMode() {
        return runMode;
    }

    @Override
    public void setDirection(Direction direction) {
        motDirection = direction;
    }

    @Override
    public Direction getDirection() {
        return motDirection;
    }

    @Override
    public void setPower(double power) {
        motPower = power;
        // Simulate motor movement, very crudely:
        switch (runMode){
            case RUN_USING_ENCODER:
            case RUN_WITHOUT_ENCODER:
                if (motPower != 0) {
                    currentPosition += (int)(10 * power);
                }
                break;
            case RUN_TO_POSITION:
                currentPosition = targetPosition;
                break;
            case STOP_AND_RESET_ENCODER:
                motPower = 0;
                currentPosition = 0;
                break;
        }

    }

    @Override
    public double getPower() {
        return motPower;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return "";
    }

    @Override
    public String getConnectionInfo() {
        return "";
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
