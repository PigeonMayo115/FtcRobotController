package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.custom.Drivetrain;

public class OutreachEliot extends OpMode {

    private Drivetrain myDrivetrain;

    @Override
    public void init() {
        myDrivetrain = new Drivetrain(hardwareMap, Drivetrain.Robot.ELIOT);
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        myDrivetrain.stickDrive(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,1,2);

    }

}
