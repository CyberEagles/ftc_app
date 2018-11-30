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
    public DcMotor lowerArmMotor = null;
    private DcMotor middleArmMotor = null;
    private CRServo intake_flip = null;
    private DcMotor intake = null;
  //  public CRServo intakePivot;
  //  public CRServo intakePivot2;

    public final static double ZERO_POWER = 0.0;
    public final static double MIN_POWER = -1.0;
    public final static double MAX_POWER = 1.0;

    final double INTAKE_PIVOT_SPEED = 1.0;
            double IntakeFlipPower = ZERO_POWER;


    @Override
    public void init()  {
        //Declare variables for phone to recognise//
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get (DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get (DcMotor.class, "right_back");
        liftMotor = hardwareMap.get (DcMotor.class, "lift_motor");
        lowerArmMotor = hardwareMap.get (DcMotor.class, "lower_arm");
        middleArmMotor = hardwareMap.get (DcMotor.class, "middle_arm");
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        intake_flip = hardwareMap.crservo.get("intake_flip");
        //intakePivot = hardwareMap.get (CRServo.class, "intake_servo");
        //intakePivot2 = hardwareMap.get(CRServo.class, "intake_servo2");
      //  intakePivot.setPower(ZERO_POWER);

//Set the Direction for the motors to turn when the robot moves forward//
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        lowerArmMotor.setDirection(DcMotor.Direction.FORWARD);
        middleArmMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection (DcMotor.Direction.REVERSE);

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
        double lowerArmPower;
        double middleArmPower;
        double armFlipPower;
        double intakePower;
//Drive, turning, and strafe//
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_x;
        double liftUp = gamepad1.left_trigger;
        double liftDown = gamepad1.right_trigger;
        double lowerArm = gamepad2.left_stick_y;
        double middleArm = gamepad2.right_stick_y;
        int half = 2;


//        double driveTurn = -gamepad1.left_stick_y + gamepad1.right_stick_x;
        leftFrontPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        rightFrontPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
        leftBackPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        rightBackPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        lowerArmPower = Range.clip(lowerArm / 2, -1.0, 1.0);
        middleArmPower = Range.clip (middleArm / 2, -1.0, 1.0);



       /* while (drive ++ turn = true)  {
            rightBackDrive.setPower(0.2);
        }
        while (gamepad1.left_stick_y + gamepad1.right_stick_x); rightBackDrive.setPower(0.2);*/
//front right goes backwards, front left goes forwards, back right goes forwards, back left goes backwards. For strafe right//

        // not locking the wheels while turning
        if (gamepad1.right_stick_x >= 0.1 && gamepad1.left_stick_y <= -0.1) {
            rightFrontPower = 0.2;
            rightBackPower = 0.2;
        }

        else if(gamepad1.right_stick_x <=-0.1 && gamepad1.left_stick_y <= -0.1) {
            leftFrontPower = 0.2;
            leftBackPower = 0.2;
        }
        else if(gamepad1.right_stick_x >=0.1 && gamepad1.left_stick_y <= -0.1) {
            leftFrontPower = -0.3;
            leftBackPower = -0.3;
        }
        else if(gamepad1.right_stick_x <= -0.1 && gamepad1.left_stick_y <= -0.1) {
            rightFrontPower = -0.3;
            rightBackPower = -0.3;
        }

        else {
            rightFrontPower = rightFrontPower;
            rightBackPower = rightBackPower;
            leftFrontPower = leftFrontPower;
            leftBackPower = leftBackPower;
        }

        // Activating intake and lift//
        if (gamepad2.a)intake.setPower(1.0);
        else if (gamepad2.b)intake.setPower(-1.0);
        else intake.setPower(0.0);

        if (gamepad1.left_trigger>0)liftMotor.setPower(-1.0);
        else liftMotor.setPower (0.0);
        if (gamepad1.right_trigger>0)liftMotor.setPower(1.0);
        else liftMotor.setPower(0.0);

        /*
        if (gamepad2.x) {
            lowerArmPower = lowerArmPower / 2;
            middleArmPower = middleArmPower / 2;
        }
        else {
            lowerArmPower = lowerArmPower;
            middleArmPower = middleArmPower;
        }
        */
        if(gamepad2.right_trigger>0){
            IntakeFlipPower+= INTAKE_PIVOT_SPEED;}
            else if (gamepad2.left_trigger>0){
            IntakeFlipPower -= INTAKE_PIVOT_SPEED;
        }
        else IntakeFlipPower = ZERO_POWER;

        if(gamepad2.right_trigger>0){
            IntakeFlipPower += INTAKE_PIVOT_SPEED;}
        else if (gamepad2.left_trigger>0){
            IntakeFlipPower -= INTAKE_PIVOT_SPEED;
        }
        else IntakeFlipPower =ZERO_POWER;

        IntakeFlipPower = Range.clip(IntakeFlipPower, MIN_POWER, MAX_POWER);

        intake_flip.setPower(IntakeFlipPower);
//Setting the power of the motor//
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        lowerArmMotor.setPower(lowerArmPower + 0);
        middleArmMotor.setPower(middleArmPower + 0);

        telemetry.addData("status", "loop 2");
    }
    //Stop the robot//
    @Override
    public void stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        lowerArmMotor.setPower(0);
        middleArmMotor.setPower(0);}
}