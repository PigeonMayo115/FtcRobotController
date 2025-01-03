package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.custom.Drivetrain;
@Autonomous
public class NewTurnTest extends OpMode {
    private Drivetrain myDrivetrain;
    int step = 0;
    boolean stepDone;

    @Override
    public void init() {
        myDrivetrain = new Drivetrain(hardwareMap,0);
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void loop() {

        switch (step){
            case 0:
                stepDone = myDrivetrain.turnToHeadingV2(90, Drivetrain.Turn.LEFT);
                if (stepDone){
                    step = 10;
                }
                break;
        }

    }
}
