/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


/**
Basic autonomous */

@Autonomous(name="Close 2 Depot")

public class Close2Depot extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftMotor = null;
    private CRServo teamMarker = null;


    public void main() throws InterruptedException {
        //initialize motors
        leftFrontDrive = hardwareMap.dcMotor.get("motorLeftFront");
        rightFrontDrive = hardwareMap.dcMotor.get("motorRightFront");
        leftBackDrive = hardwareMap.dcMotor.get("motorLeftBack");
        rightBackDrive = hardwareMap.dcMotor.get("motorRightBack");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        teamMarker = hardwareMap.crservo.get("team_marker");
        //    leftFrontDrive.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //    rightFrontDrive.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //    leftBackDrive.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //    rightBackDrive.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        // Most robots need the motor on one side to be reversed to drive forward

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        teamMarker = hardwareMap.crservo.get ("team_marker");
        liftMotor = hardwareMap.get (DcMotor.class, "lift_motor");


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
//what it does
        LowerDownTime (1, 2950);
        LowerDown (0);
        sleep(500);
        DriveForwardTime(-1,50);
        DriveForward(0);
        sleep(1000);
        StrafeRightTime(-1,300);
        StrafeRight(0);
        sleep (450);
        RotateTime(-1, 1200);
        DriveForwardTime(-1,1700);
        // In Depot//
        DriveForwardTime(1,200);
        DriveForward(0);
        RotateTime(-1,1200);
        sleep(500);
        teamMarker.setPower(0.2);
        sleep(300);
        StrafeRightTime(-1, 500);
        StrafeRight(0);
        sleep(1000);
        RotateTime(-1,800);
        Rotation(0);
        sleep(500);
        ParkTime(1,500);

        teamMarker.setPower(0);

        StrafeRightTime(-1, 300);
        StrafeRight(0);
        DriveForwardTime(-1,2000);


        /*DriveForwardTime(-1, 300);
        DriveForward(0);
        RotateTime(1, 100);
        Rotation(0);
        DriveForwardTime(-1,400);
        DriveForward(0);
        ParkTime(1,500);
        RotateTime(1,100);
        DriveForwardTime(-1, 750);*/

        /*
        RotateTime(1, 100);
        Rotation(0);
        DriveForwardTime(-1,1100);
*/
//        MarkerDropTime(1, 500);
        //Straighten out on wall
        //StrafeRightTime(1,500);
       // StrafeRight(0);
        //DriveForwardTime(1,3000);
        //DriveForward(0);


    }
    double ParkTime (double power, long time) throws InterruptedException {
        Park(power);
        Thread.sleep(time);
        return 0;
    }
    double LowerDownTime (double power,long time) throws InterruptedException {
        LowerDown(-power);
        Thread.sleep(time);
        return 0;
    }
    double RotateTime (double power, long time) throws InterruptedException {
        Rotation(power);
        Thread.sleep(time);
        return 0;
    }

    double DriveForwardTime(double power, long time) throws InterruptedException {
        DriveForward(power);
        Thread.sleep(time);
        return 0;
    }
    double StrafeRightTime (double power, long time) throws InterruptedException {
        StrafeRight(power);
        Thread.sleep(time);
        return 0;
    }
    double MarkerDropTime (double power, long time) throws InterruptedException {
        DropMarker(power);
        Thread.sleep(time);
        return 0;
    }
    public void LowerDown(double power) {
        liftMotor.setPower(-power);
    }
   public void DriveForward(double power) {
        leftFrontDrive.setPower(power);
       leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
    }
    public void Rotation (double power){
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }
    public void StrafeRight (double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(-power);
        rightFrontDrive.setPower(power);
    }
    public void DropMarker (double power) {
        teamMarker.setPower(-power);
    }

    public void Park (double power){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}

