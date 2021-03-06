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
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


/**
 Basic autonomous */

@Autonomous(name="Color Sensor auto that should work")

public class ColorSensorTestAutoWorks extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftMotor = null;
    private Servo teamMarker = null;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    public void main() throws InterruptedException {
        //initialize motors
        leftFrontDrive = hardwareMap.dcMotor.get("left_front");
        rightFrontDrive = hardwareMap.dcMotor.get("right_front");
        leftBackDrive = hardwareMap.dcMotor.get("left_back");
        rightBackDrive = hardwareMap.dcMotor.get("right_back");
        liftMotor = hardwareMap.dcMotor.get("lift_motor");
        teamMarker = hardwareMap.servo.get("team_marker");
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
        teamMarker = hardwareMap.servo.get("team_marker");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // wait for the start button to be pressed.

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Saturation", hsvValues[1]);
            telemetry.addData("hsv 2", hsvValues[2]);
            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();


            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
/**robot does stuff here
 * Yay
 */


//            DriveForwardTime(1,100);
//            ParkTime(1,100);
//            DriveForwardTime(-1,100);
//            ParkTime(1,100);
            teamMarker.setPosition(-0.1);
            LowerDownTime(1, 2900);
            LowerDown(0);
            //lowering down
            ParkTime(1,1000);
            //wait after
            DriveForwardTime(-1, 100);
            DriveForward(0);
            //move out from lander
            ParkTime(1,500);
            StrafeRightTime(-1, 1200);
            StrafeRight(0);
            // strafe out from lander
            ParkTime(1,500);
            RotateTime(1, 800);
            Rotation(0);
            ParkTime(0, 500);
            StrafeRightTime(1, 500);
            StrafeRight(0);
            ParkTime(1, 500);
            DriveForwardTime(1, 900);
            DriveForward(0);
            ParkTime(1,500);
            RotateTime(-1, 800);
            Rotation(0);
            ParkTime(1, 500);
            teamMarker.setPosition(0.5);
            //DriveForwardTime(-1, 500);
            //turn towards minerals
            RotateTime(-1, 400);
            Rotation(0);
            ParkTime(1,500);
            DriveForwardTime(1, 100);
            DriveForward(0);
            ParkTime(1,500);
            StrafeRightTime(-1,750);
            StrafeRight(0);
            ParkTime(1,500);
            RotateTime(1,50);
            Rotation(0);
            ParkTime(1,500);
            DriveForwardTime(1,2100);
            DriveForward(0);
            ParkTime(1, 30000);
//line up to the right side sample
//insert the proper numbers in when the build is done
// these are approximate values based on last competition

          /*  if ((hsvValues[1] < 0.17)) {
                telemetry.addData("Color", "Silver");
                telemetry.update();
                DriveForwardTime(-1, 200);
                ParkTime(1, 3000);
            } else if ((hsvValues[0] > 110)) {
                telemetry.addData("Color", "Floor");
                telemetry.update();
                DriveForwardTime(1, 300);
                ParkTime(1, 3000);
            } else {
                telemetry.addData("Color", "Yellow");
                telemetry.update();
                //do rest of auto here
                RotateTime(1, 500);
                ParkTime(1, 500);
                DriveForwardTime(1, 500);
                ParkTime(1, 500);
                RotateTime(1,500);
                ParkTime(1,500);
                // insert servo here
                DriveForwardTime(1,500);
                ParkTime(1, 30000); //to stop the robot on the crater
            }


            if ((hsvValues[1] < 0.17)) {
                telemetry.addData("Color", "Silver");
                telemetry.update();
                StrafeRightTime(-1, 200);
                ParkTime(1, 300);
            } else if ((hsvValues[0] > 110)) {
                telemetry.addData("Color", "Floor");
                telemetry.update();
                StrafeRightTime(-1, 200);
                ParkTime(1, 300);
            } else {
                telemetry.addData("Color", "Yellow");
                telemetry.update();
                //do rest of auto here
                RotateTime(1, 500);
                ParkTime(1, 500);
                DriveForwardTime(1, 500);
                ParkTime(1, 500);
                RotateTime(1,500);
                ParkTime(1,500);
                // insert servo here
                DriveForwardTime(1,500);
                ParkTime(1, 30000); //to stop robot on the crater
            }


            if ((hsvValues[1] < 0.17)) {
                telemetry.addData("Color", "Silver");
                telemetry.update();
                RotateTime(1, 500);
                ParkTime(1, 500);
                DriveForwardTime(1, 500);
                ParkTime(1, 500);
                RotateTime(1,500);
                ParkTime(1,500);
                // insert servo here
                DriveForwardTime(1,500);
            } else if ((hsvValues[0] > 110)) {
                telemetry.addData("Color", "Floor");
                telemetry.update();
                RotateTime(1, 500);
                ParkTime(1, 500);
                DriveForwardTime(1, 500);
                ParkTime(1, 500);
                RotateTime(1,500);
                ParkTime(1,500);
                // insert servo here
                DriveForwardTime(1,500); */            }

            ParkTime(1, 30000); //to stop robot on the crater and so auto does not loop.
//            DriveForwardTime(1,100);
//            ParkTime(1,100);
//            DriveForwardTime(-1,100);
//            ParkTime(1,100);

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
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-power);
        rightFrontDrive.setPower(-power);
    }
    public void Rotation (double power){
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }
    public void StrafeRight (double power) {
        leftFrontDrive.setPower(-power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
    }
    public void DropMarker (double power) {
        teamMarker.setPosition( -power);
    }

    public void Park (double power){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}

