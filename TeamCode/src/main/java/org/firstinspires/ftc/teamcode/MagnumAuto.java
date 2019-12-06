package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="Magnum Auto")
public class MagnumAuto extends LinearOpMode {

    public final int BLUE_IDEAL = 300;
    public final int BLUE_TOLERANCE = 1;
    ColorSensor cs;
    public void runOpMode(){
        cs = hardwareMap.colorSensor.get("color");
        waitForStart();
        while(opModeIsActive()){
            double sum = cs.red() + cs.blue() + cs.green();
            telemetry.addData("red", cs.red() + ", " + (cs.red()/sum));
            telemetry.addData("blue", cs.blue() + ", " + (cs.green()/sum));
            telemetry.addData("green", cs.green() + ", " + (cs.green()/sum));
            telemetry.addData("intensity", cs.alpha());
            telemetry.addData("argb", cs.argb());
            telemetry.update();
        }
    }
}
