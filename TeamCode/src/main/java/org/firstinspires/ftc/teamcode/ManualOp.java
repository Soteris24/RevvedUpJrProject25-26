package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ManualOp")
public class ManualOp extends LinearOpMode {

    // PID constants for gaylestas
    public static double Kp1 = 0.01;
    public static double Ki1 = 0.0;
    public static double Kd1 = 0.0001;

    // PID constants for gayandrikos
    public static double Kp2 = 0.01;
    public static double Ki2 = 0.0;
    public static double Kd2 = 0.0001;

    // PID constants for gayargiris
    public static double Kp3 = 0.02 ;
    public static double Ki3 = 0.0;
    public static double Kd3 = 0.0001;

    // PID constants for straightsoteris
    public static double Kp4 = 0.01;
    public static double Ki4 = 0.0;
    public static double Kd4 = 0.0001;

    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx motor3;
    private DcMotorEx motor4;
    private DcMotorEx baseLeft;
    private DcMotorEx baseRight;

    private PIDController pid1;
    private PIDController pid2;
    private PIDController pid3;
    private PIDController pid4;

    // Servos

    private Servo claw; double clawPos = 0;
    private CRServo drill; double drillPower = 0;
    private Servo clawMove; double clawMovePos = 0;
    private Servo drillMove; double drillMovePos = 1;
    private Servo servoSoteris; double servoSoterisPos = 0;
    private Servo gaytommys; double gaytommysPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize Servos
        claw = hardwareMap.get(Servo.class, "claw");
        clawMove = hardwareMap.get(Servo.class, "clawMove");
        drill = hardwareMap.get(CRServo.class, "drill");
        drillMove = hardwareMap.get(Servo.class, "drillMove");
        servoSoteris = hardwareMap.get(Servo.class, "servoSoteris");
        gaytommys = hardwareMap.get(Servo.class, "shades");

        // Initialize motors
        motor1 = hardwareMap.get(DcMotorEx.class, "gaylestas");
        motor2 = hardwareMap.get(DcMotorEx.class, "gayandrikos");
        motor3 = hardwareMap.get(DcMotorEx.class, "gayargiris");
        motor4 = hardwareMap.get(DcMotorEx.class, "straightsoteris");
        baseLeft = hardwareMap.get(DcMotorEx.class, "baseLeft");
        baseRight = hardwareMap.get(DcMotorEx.class, "baseRight");

        
        // Setup motor1 (gaylestas)
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setup motor2 (gayandrikos)
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setup motor3 (gayargiris)
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setup motor4 (straightsoteris)
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize PID controllers
        pid1 = new PIDController(Kp1, Ki1, Kd1);
        pid2 = new PIDController(Kp2, Ki2, Kd2);
        pid3 = new PIDController(Kp3, Ki3, Kd3);
        pid4 = new PIDController(Kp4, Ki4, Kd4);


        waitForStart();

        while (opModeIsActive()) {
            baseLeft.setPower(gamepad1.left_stick_y);
            baseRight.setPower(gamepad1.right_stick_y);

            if (gamepad1.dpad_up) {
                pid1.changeTargetPosition(15);
            } else if (gamepad1.dpad_down) {
                pid1.changeTargetPosition(-15);
            }

            if (gamepad1.dpad_right) {
                pid2.changeTargetPosition(10);
            } else if (gamepad1.dpad_left) {
                pid2.changeTargetPosition(-10);
            }

            if (gamepad1.a) {
                pid3.changeTargetPosition(2);
            } else if (gamepad1.b) {
                pid3.changeTargetPosition(-2);
            }

            if (gamepad1.x) {
                pid4.changeTargetPosition(10);
            } else if (gamepad1.y) {
                pid4.changeTargetPosition(-10);
            }

            double power1 = pid1.update(motor1.getCurrentPosition());
            double power2 = pid2.update(motor2.getCurrentPosition());
            double power3 = pid3.update(motor3.getCurrentPosition());
            double power4 = pid4.update(motor4.getCurrentPosition());

            motor1.setPower(power1);
            motor2.setPower(power2);
            motor3.setPower(power3);
            motor4.setPower(power4);

            // Gamepad 2 Servo Control
            if (gamepad2.a) clawPos += 0.01;
            else if (gamepad2.b) clawPos -= 0.01;
            clawPos = Range.clip(clawPos, 0, 1);
            claw.setPosition(clawPos);

            if (gamepad2.x) {
                drillPower = 1.0;
            } else if (gamepad2.y) {
                drillPower = 0.0;
            }
            drill.setPower(drillPower);

            if (gamepad2.dpad_up) clawMovePos += 0.01;
            else if (gamepad2.dpad_down) clawMovePos -= 0.01;
            clawMovePos = Range.clip(clawMovePos, 0, 1);
            clawMove.setPosition(clawMovePos);

            if (gamepad2.dpad_right) drillMovePos += 0.01;
            else if (gamepad2.dpad_left) drillMovePos -= 0.01;
            drillMovePos = Range.clip(drillMovePos, 0, 1);
            drillMove.setPosition(drillMovePos);

            if (gamepad2.left_bumper) servoSoterisPos += 0.01;
            else if (gamepad2.right_bumper) servoSoterisPos -= 0.01;
            servoSoterisPos = Range.clip(servoSoterisPos, 0, 1);
            servoSoteris.setPosition(servoSoterisPos);

            if (gamepad2.left_trigger > 0.5) gaytommysPos += 0.01;
            else if (gamepad2.right_trigger > 0.5) gaytommysPos -= 0.01;
            gaytommysPos = Range.clip(gaytommysPos, 0, 1);
            gaytommys.setPosition(gaytommysPos);

            // Telemetry
            telemetry.addData("Gaylestas Target", pid1.getTargetPosition());
            telemetry.addData("Gaylestas Current", motor1.getCurrentPosition());


            telemetry.addData("Gayandrikos Target", pid2.getTargetPosition());
            telemetry.addData("Gayandrikos Current", motor2.getCurrentPosition());


            telemetry.addData("Gayargiris Target", pid3.getTargetPosition());
            telemetry.addData("Gayargiris Current", motor3.getCurrentPosition());

            telemetry.addData("Straightsoteris Target", pid4.getTargetPosition());
            telemetry.addData("Straightsoteris Current", motor4.getCurrentPosition());

            telemetry.addData("Claw Pos", clawPos);
            telemetry.addData("Drill Power", drillPower);
            telemetry.addData("Claw Move Pos", clawMovePos);
            telemetry.addData("Drill Move Pos", drillMovePos);
            telemetry.addData("Soteris Servo Pos", servoSoterisPos);
            telemetry.addData("Tommys Servo Pos", gaytommysPos);

            telemetry.update();
        }
    }
}
