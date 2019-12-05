
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name="Magnum W0ng", group="Test")
public class MagnumD0ng extends LinearOpMode {
    final double sumSineCosineWheelHypo = 2.0 / Math.sqrt(2.0); //Sum of the sine and cosine of the triangle formed by the distance of the wheel to the center of the robot

    DcMotor flmot;
    DcMotor blmot;
    DcMotor frmot;
    DcMotor brmot;
    DcMotor lowarm;
    DcMotor uparm;

    CRServo grip0;
    CRServo grip1;
    Servo wrist;
    CRServo slide;
    Servo waffle0;
    Servo waffle1;


    @Override
    public void runOpMode() {

        flmot = hardwareMap.dcMotor.get("mot0");
        blmot = hardwareMap.dcMotor.get("mot1");
        brmot = hardwareMap.dcMotor.get("mot2");
        frmot = hardwareMap.dcMotor.get("mot3");
        lowarm = hardwareMap.dcMotor.get("barm");
        uparm = hardwareMap.dcMotor.get("farm");

        grip0 = hardwareMap.crservo.get("fgrip");
        grip1 = hardwareMap.crservo.get("bgrip");
        wrist = hardwareMap.servo.get("wrist");
        slide = hardwareMap.crservo.get("slide");
        waffle0 = hardwareMap.servo.get("waffle0");
        waffle1 = hardwareMap.servo.get("waffle1");

        flmot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frmot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blmot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brmot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        uparm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        flmot.setDirection(DcMotorSimple.Direction.FORWARD);
        frmot.setDirection(DcMotorSimple.Direction.FORWARD);
        blmot.setDirection(DcMotorSimple.Direction.REVERSE);
        brmot.setDirection(DcMotorSimple.Direction.REVERSE);
        lowarm.setDirection(DcMotorSimple.Direction.FORWARD);
        uparm.setDirection(DcMotorSimple.Direction.FORWARD);


        flmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uparm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        //OnStart

        while (opModeIsActive()) {
            //Loop-
            double[] pows = mechanumPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            flmot.setPower(pows[0]);
            blmot.setPower(pows[1]);
            brmot.setPower(pows[2]);
            frmot.setPower(pows[3]);

            if (gamepad1.a) {
                lowarm.setPower(0.5d);
            } else if ((gamepad1.b)) {
                lowarm.setPower(-0.5d);
            } else {
                lowarm.setPower(0);
            }

            if (gamepad1.y) {
                uparm.setPower(0.5d);
            } else if (gamepad1.x) {
                uparm.setPower(-0.5d);
            } else {
                uparm.setPower(0);
            }


            if (gamepad1.left_bumper) {
                grip0.setPower(0.25);
                grip1.setPower(-0.25);
            } else if (gamepad1.right_bumper) {
                grip0.setPower(-0.25);
                grip1.setPower(0.25);
            } else {
                grip0.setPower(0.0);
                grip1.setPower(0.0);
            }

            if (gamepad1.dpad_up) {
                slide.setPower(0.25);
            } else if (gamepad1.dpad_down) {
                slide.setPower(-0.25);
            } else {
                slide.setPower(0.0);
            }

            double pos = 0.5 + 0.5 * (gamepad1.left_trigger - gamepad1.right_trigger);
            wrist.setPosition(pos);
            telemetry.addData("FL", pows[0]);
            telemetry.addData("BL", pows[1]);
            telemetry.addData("BR", pows[2]);
            telemetry.addData("FR", pows[3]);
            telemetry.addData("rotServ", wrist.getPosition());
            telemetry.update();
        }
    }

    public double[] mechanumPower(double x, double y, double rot) {
        //Starting from the front left (0), counterclockwise (1,2,3), ratios for motor powers in range of [-1,1]

        double[] coeffs = new double[4];
        coeffs[0] = y + x - rot * sumSineCosineWheelHypo;
        coeffs[1] = y - x - rot * sumSineCosineWheelHypo;
        coeffs[2] = y + x + rot * sumSineCosineWheelHypo;
        coeffs[3] = y - x + rot * sumSineCosineWheelHypo;

        double largest = Math.max(Math.max(Math.abs(coeffs[0]), Math.abs(coeffs[1])), Math.max(Math.abs(coeffs[2]), Math.abs(coeffs[3])));
        if (largest > 1.0d) {
            coeffs[0] = coeffs[0] / largest;
            coeffs[1] = coeffs[1] / largest;
            coeffs[2] = coeffs[2] / largest;
            coeffs[3] = coeffs[3] / largest;
        }
        return coeffs;

    }

}

























































/*
        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }        Hafdoifwa5a4$$r fwaef wWA_-rw fg ijoiewf -wfa - f g-  doaif jid a;
        ResQ.dxd -fns [34.5];
        if(ResQ.dxd == TRUE){
            Delet.comp/everything
        }
 */