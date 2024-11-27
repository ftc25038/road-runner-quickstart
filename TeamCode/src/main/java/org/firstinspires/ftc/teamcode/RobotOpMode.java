package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class RobotOpMode extends OpMode {

    //Variables

    // Declare our motors
    // Make sure your ID's match your configuration
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    DcMotor rightLift;
    DcMotor leftLift;

    public static DcMotorSimple.Direction FLM = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction BLM = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction FRM = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction BRM = DcMotorSimple.Direction.FORWARD;

    public static DcMotorSimple.Direction rLift = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction lLift = DcMotorSimple.Direction.FORWARD;

    DigitalChannel  touch;

    public static double speed = 0.5;
    public static double correction = 1.1;

    //Servos


    @Override
    public void init() {

        // Motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        rightLift = hardwareMap.dcMotor.get("rightLift");
        leftLift = hardwareMap.dcMotor.get("leftLift");


        frontRightMotor.setDirection(FRM);
        backRightMotor.setDirection(BRM);
        frontLeftMotor.setDirection(FLM);
        backLeftMotor.setDirection(BLM);

        rightLift.setDirection(rLift);
        leftLift.setDirection(lLift);

        touch = hardwareMap.get(DigitalChannel.class, "touch_sensor");


    }
    @Override
    public void loop() {

        // Remember, Y stick value is reversed
        double y = (-gamepad1.left_stick_y - gamepad1.right_stick_y)/2;
        double x = -gamepad1.left_stick_x * correction * speed; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x * speed;


        double balance = 0.00; //C.G corrector
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator);
        double backLeftPower = ((y - x+ rx + balance) / denominator);
        double frontRightPower = ((y - x - rx) / denominator);
        double backRightPower = ((y + x - rx + balance) / denominator);

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        if (gamepad1.right_bumper && touch.getState()) {
            rightLift.setPower(1);
        }
        else if (gamepad1.right_trigger != 0 && touch.getState()) {
            rightLift.setPower(-1);
        }
        else {
            rightLift.setPower(0);
        }

        if (gamepad1.left_bumper) {
            leftLift.setPower(1);
        }
        else if (gamepad1.left_trigger != 0) {
            leftLift.setPower(-1);
        }
        else {
            leftLift   .setPower(0);
        }
        

        // telemetry data

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        telemetry.addData("speed: ", speed);
        dashboardTelemetry.addData("speed: ", speed);

        telemetry.addData("left joystick y: ", -gamepad1.left_stick_y);
        dashboardTelemetry.addData("left joystick y: ", -gamepad1.left_stick_y);

        telemetry.addData("right joystick y: ", -gamepad1.right_stick_y);
        dashboardTelemetry.addData("right joystick y: ", -gamepad1.right_stick_y);

        telemetry.addData("touch sensor state: ", touch.getState());

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", 3.7);
        packet.put("status", "alive");
        dashboardTelemetry.addData("x", 3.7);
        dashboardTelemetry.update();
    }
}