package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
@TeleOp(name = "arts and crafts", group = "Competition")
public class LifeIsPain extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor mot0 = null;
    private DcMotor mot1 = null;
    private DcMotor mot2 = null;
    private DcMotor mot3 = null;
    private CRServo ser0 = null;

    private double turnmult = 1.0;

    @Override
    public void init() {
        mot0 = hardwareMap.get(DcMotor.class, "mot0");
        mot1 = hardwareMap.get(DcMotor.class, "mot1");
        mot2 = hardwareMap.get(DcMotor.class, "mot2");
        mot3 = hardwareMap.get(DcMotor.class, "mot3");
        ser0 = hardwareMap.get(CRServo.class, "ser0");


        mot0.setDirection(DcMotorSimple.Direction.FORWARD);
        mot1.setDirection(DcMotorSimple.Direction.FORWARD);
        mot2.setDirection(DcMotorSimple.Direction.FORWARD);
        mot3.setDirection(DcMotorSimple.Direction.FORWARD);


        mot0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mot3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {
    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        mot0.setPower(-gamepad1.right_stick_y * turnmult);
        mot1.setPower(gamepad1.left_stick_y * turnmult);
        ser0.setPower(gamepad1.left_trigger-gamepad1.right_trigger);

        mot2.setPower(gamepad2.right_stick_y);
        mot3.setPower(gamepad2.left_stick_y);

    }


}
