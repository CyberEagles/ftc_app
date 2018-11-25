package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp
public class Driving extends OpMode
{
//Declare motors and variables//
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftMotor = null;
    private DcMotor armBase = null;
    private DcMotor armPivot = null;
    private DcMotor intake = null;


    @Override
    public void init()  {
        //Declare variables for phone to recognise//
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get (DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get (DcMotor.class, "right_back_drive");
        liftMotor = hardwareMap.get (DcMotor.class, "lift_motor");
        armBase = hardwareMap.get (DcMotor.class, "arm_base");
        armPivot = hardwareMap.get (DcMotor.class, "arm_pivot");
        intake = hardwareMap.get (DcMotor.class, "intake");

//Set the Direction for the motors to turn when the robot moves forward//
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        armBase.setDirection(DcMotor.Direction.FORWARD);
        armPivot.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setMode (DcMotor.RunMode.RUN_TO_POSITION);

//Tells drivers that robot is ready//
        telemetry.addData("status", "Initialized");
    }

    @Override
    public void start() {
        telemetry.addData("status", "start");
    }
//Set variables//
    @Override
    public void loop() {
        telemetry.addData("status", "loop 1");
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;
        double liftPower;
        double arm1Power;
        double arm2Power;
        double armServoPower;
        double intakePower;
//Drive, turning, and strafe//
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;
        boolean liftUp = gamepad1.dpad_up;
        boolean liftDown = gamepad1.dpad_down;
        double lowerArm = gamepad2.left_stick_y;
        double upperArm = gamepad2.right_stick_y;
        boolean intakeSpin = gamepad2.a;


//        double driveTurn = -gamepad1.left_stick_y + gamepad1.right_stick_x;
        leftFrontPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        rightFrontPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
        leftBackPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        rightBackPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        arm1Power = Range.clip(upperArm, -1.0, 1.0);
        arm2Power = Range.clip (lowerArm, -1.0, 1.0);


       /* while (drive ++ turn = true)  {
            rightBackDrive.setPower(0.2);
        }
        while (gamepad1.left_stick_y + gamepad1.right_stick_x); rightBackDrive.setPower(0.2);*/
//front right goes backwards, front left goes forwards, back right goes forwards, back left goes backwards. For strafe right//

        // not locking the wheels while turning
        if (gamepad1.right_stick_x >= 0.1 && gamepad1.left_stick_y <= -0.1) {
            rightFrontPower = -0.2;
            rightBackPower = -0.2;
        }

        else if(gamepad1.right_stick_x <=-0.1 && gamepad1.left_stick_y <= -0.1) {
            leftFrontPower = -0.2;
            leftBackPower = -0.2;
        }
        else if(gamepad1.right_stick_x >=0.1 && gamepad1.left_stick_y <= -0.1) {
            leftFrontPower = 0.3;
            leftBackPower = 0.3;
        }
        else if(gamepad1.right_stick_x <= -0.1 && gamepad1.left_stick_y <= -0.1) {
            rightFrontPower = 0.3;
            rightBackPower = 0.3;
        }

        else {
            rightFrontPower = rightFrontPower;
            rightBackPower = rightBackPower;
            leftFrontPower = leftFrontPower;
            leftBackPower = leftBackPower;
        }

        // Activating intake and lift//
        if (gamepad2.a)intake.setPower(1.0);
        else intake.setPower(0.0);
        if (gamepad1.dpad_up)liftMotor.setTargetPosition(1440);
        else liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//Setting the power of the motor//
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        telemetry.addData("status", "loop 2");
    }
    //Stop the robot//
    @Override
    public void stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        armBase.setPower(0);
        armPivot.setPower(0);}
}