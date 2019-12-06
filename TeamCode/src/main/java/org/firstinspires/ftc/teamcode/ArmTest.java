package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "ArmTest", group = "Test")
public class ArmTest extends LinearOpMode {
    DcMotor angle;
    DcMotor arm;
    @Override
    public void runOpMode() {
        angle = (DcMotor) hardwareMap.get("mot0");
        arm = (DcMotor) hardwareMap.get("mot1");
        /*angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angle.setTargetPosition(0);
        arm.setTargetPosition(0);*/
        angle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        //angle.setPower(0.1);
        //arm.setPower(0.25);
        while (opModeIsActive()) {

            /*if(gamepad1.a){
                angle.setPower(0.1);
                angle.setTargetPosition(280);
            }else if(gamepad1.b){
                angle.setPower(0.1);
                angle.setTargetPosition(-280);
            }else{
                angle.setTargetPosition(angle.getCurrentPosition());
                angle.setPower(0.0);
            }

            if(gamepad1.x){
                arm.setPower(0.25);
                arm.setTargetPosition(-280);
            }else if(gamepad1.y){
                arm.setPower(0.25);
                arm.setTargetPosition(280);
            }else{
                arm.setTargetPosition(arm.getCurrentPosition());
                arm.setPower(0.0);
            }*/

            telemetry.addData("basePos", "CUR:"+angle.getCurrentPosition()+" TAR:"+angle.getTargetPosition());
            telemetry.addData("armPos", "CUR:"+arm.getCurrentPosition()+" TAR:"+arm.getTargetPosition());
            telemetry.update();
        }
    }
}
