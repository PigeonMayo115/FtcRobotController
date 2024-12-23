package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;

@TeleOp
public class ColorSensorTest extends OpMode {

    ColorSensor colorSensor = null;
    boolean strobe = false;

    @Override
    public void init() {
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){
    colorSensor.enableLed(true);
    strobe = true;
    }

    @Override
    public void loop() {

        telemetry.addData("red",colorSensor.red());
        telemetry.addData("green",colorSensor.green());
        telemetry.addData("blue",colorSensor.blue());
        telemetry.addData("alpha",colorSensor.alpha());

        if (strobe){
            strobe = false;
            colorSensor.enableLed(false);

        } else if (!strobe){
            strobe = true;
            colorSensor.enableLed(true);
        }


    }
}
