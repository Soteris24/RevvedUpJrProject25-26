package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AutonOp")
public class AutonOp extends LinearOpMode {

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

    // PID constants for coreSoteris
    public static double Kp5 = 0.01;
    public static double Ki5 = 0.0;
    public static double Kd5 = 0.0001;

    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx motor3;
    private DcMotorEx motor4;
    private DcMotorEx coreSoteris;
    private DcMotorEx baseLeft;
    private DcMotorEx baseRight;

    private PIDController pid1;
    private PIDController pid2;
    private PIDController pid3;
    private PIDController pid4;
    private PIDController pid5;

    private DistanceSensor leftDistanceSensor, rightDistanceSensor;

    // Servos

    private Servo claw; double clawPos = 0;
    private CRServo drill; double drillPower = 0;
    private Servo clawMove; double clawMovePos = 0;
    private Servo drillMove; double drillMovePos = 1;
    private Servo gaytommys; double gaytommysPos = 0;

    public enum currentState {SET_POSITION, IDLE, MOVE_FORWARD, COLLECT, REST_POS}
    private currentState state = currentState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize Servos
        claw = hardwareMap.get(Servo.class, "claw");
        clawMove = hardwareMap.get(Servo.class, "clawMove");
        drill = hardwareMap.get(CRServo.class, "drill");
        drillMove = hardwareMap.get(Servo.class, "drillMove");
        coreSoteris = hardwareMap.get(DcMotorEx.class, "coreSoteris");
        gaytommys = hardwareMap.get(Servo.class, "shades");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        // Initialize motors
        motor1 = hardwareMap.get(DcMotorEx.class, "gaylestas");
        motor2 = hardwareMap.get(DcMotorEx.class, "gayandrikos");
        motor3 = hardwareMap.get(DcMotorEx.class, "gayargiris");
        motor4 = hardwareMap.get(DcMotorEx.class, "straightsoteris");
        baseLeft = hardwareMap.get(DcMotorEx.class, "baseLeft");
        baseRight.setDirection(DcMotor.Direction.REVERSE);
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

        // Setup coreSoteris
        coreSoteris.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        coreSoteris.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        coreSoteris.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize PID controllers
        pid1 = new PIDController(Kp1, Ki1, Kd1);
        pid2 = new PIDController(Kp2, Ki2, Kd2);
        pid3 = new PIDController(Kp3, Ki3, Kd3);
        pid4 = new PIDController(Kp4, Ki4, Kd4);
        pid5 = new PIDController(Kp5, Ki5, Kd5);


        waitForStart();

        while (opModeIsActive()) {

            switch (state) {
                case SET_POSITION:
                    break;
                case IDLE:
                    break;
                case MOVE_FORWARD:
                    if (leftDistanceSensor.getDistance(DistanceUnit.CM) < 10) {
                        baseLeft.setPower(1);
                        baseRight.setPower(1);
                    } else {
                        baseLeft.setPower(0);
                        baseRight.setPower(0);
                        break;
                    }

                    break;
                case COLLECT:
                    break;
                case REST_POS:
                    break;

            }

            double power1 = pid1.update(motor1.getCurrentPosition());
            double power2 = pid2.update(motor2.getCurrentPosition());
            double power3 = pid3.update(motor3.getCurrentPosition());
            double power4 = pid4.update(motor4.getCurrentPosition());
            double power5 = pid5.update(coreSoteris.getCurrentPosition());

            motor1.setPower(power1);
            motor2.setPower(power2);
            motor3.setPower(power3);
            motor4.setPower(power4);
            coreSoteris.setPower(power5);

            // Telemetry
            telemetry.addData("Gaylestas Target", pid1.getTargetPosition());
            telemetry.addData("Gaylestas Current", motor1.getCurrentPosition());


            telemetry.addData("Gayandrikos Target", pid2.getTargetPosition());
            telemetry.addData("Gayandrikos Current", motor2.getCurrentPosition());


            telemetry.addData("Gayargiris Target", pid3.getTargetPosition());
            telemetry.addData("Gayargiris Current", motor3.getCurrentPosition());

            telemetry.addData("Straightsoteris Target", pid4.getTargetPosition());
            telemetry.addData("Straightsoteris Current", motor4.getCurrentPosition());

            telemetry.addData("CoreSoteris Target", pid5.getTargetPosition());
            telemetry.addData("CoreSoteris Current", coreSoteris.getCurrentPosition());

            telemetry.addData("Claw Pos", clawPos);
            telemetry.addData("Drill Power", drillPower);
            telemetry.addData("Claw Move Pos", clawMovePos);
            telemetry.addData("Drill Move Pos", drillMovePos);
            telemetry.addData("Tommys Servo Pos", gaytommysPos);

            telemetry.update();
        }
    }
}
