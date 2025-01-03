package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.custom.Drivetrain;

import java.util.Locale;

@Autonomous
public class SimpleOdoTest extends OpMode {
    Drivetrain myDrivetrain;
    private int step = 0;
    private boolean stepDone;
    @Override
    public void init() {
        myDrivetrain = new Drivetrain(hardwareMap,0);

    }

    @Override
    public void loop() {

        Pose2D pos = myDrivetrain.odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        switch (step){
            case 0:
                stepDone = myDrivetrain.rightToY(pos,5);
                if (stepDone){
                    step = 10;
                }
                break;
            case 10:
                stepDone = myDrivetrain.leftToY(pos,0);
                if (stepDone){
                    step = 20;
                }
                break;
            case 20:
                stepDone = myDrivetrain.forwardToX(pos,5);
                if (stepDone){
                    step = 30;
                }
                break;
            case 30:
                stepDone = myDrivetrain.reverseToX(pos,0);
                if (stepDone){
                    step = 40;
                }
                break;
        }

    }
}
