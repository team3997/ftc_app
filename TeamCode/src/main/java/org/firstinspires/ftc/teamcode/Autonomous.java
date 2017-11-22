/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Forward&Backward", group="Routines")  // @Autonomous(...) is the other common choice
//@Disabled
public class Autonomous extends LinearOpMode {
    int TICKS_PER_ROTATION = 1120;
    double TICKS_PER_DEGREE = 0;
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor frontLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor backRightMotor = null;

    void driveForward(double power) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    void rightPower(double power) {
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
    }
    void leftPower(double power) {
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
    }


    void stopDriving() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    void ratioCheck(int test_ticks, double power) {
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftMotor.setTargetPosition(test_ticks);
        backRightMotor.setTargetPosition(test_ticks);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //turn right
        leftPower(power);
        rightPower(-power);

        while(backLeftMotor.isBusy() && backRightMotor.isBusy()) {

        }

        stopDriving();

        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Might use above ^ code in a function because I use it a lot
        //                |

        //Now measure angle it goes and store in variable
        double angleTurned = /*lets say*/ 56.4;

        double encoderAngleRatio = test_ticks/angleTurned; //e.g 5 ticks per degree
        TICKS_PER_DEGREE = encoderAngleRatio;

        telemetry.addData("EncoderAngleRatio", encoderAngleRatio);

    }

    //distance and diameter in inches
    void driveDistance(double power, double diameter, double distance) {
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // rotations = (distance/circumference)
        int total_ticks = (int) Math.round((distance / (diameter*Math.PI) * TICKS_PER_ROTATION));
        telemetry.addData("ticks_forward", total_ticks);
        telemetry.update();
        //Fixing andymark tolerance 89%
        total_ticks = (int)Math.round(total_ticks * 0.89);
        backLeftMotor.setTargetPosition(total_ticks);
        backRightMotor.setTargetPosition(total_ticks);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveForward(power);

        while(backLeftMotor.isBusy() && backRightMotor.isBusy()) {

        }

        stopDriving();
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void turnRight(double power, double degrees) {
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticks = (int) Math.round(TICKS_PER_DEGREE * degrees);

        //Turn to the right
        backLeftMotor.setTargetPosition(ticks);
        backRightMotor.setTargetPosition(ticks);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //turn right
        leftPower(power);
        rightPower(-power);
        while(backLeftMotor.isBusy() && backRightMotor.isBusy()) {

        }

        stopDriving();
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void turnLeft(double power, double degrees) {
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticks = (int) Math.round(TICKS_PER_DEGREE * degrees);

        //Turn to the right
        frontRightMotor.setTargetPosition(ticks);
        backRightMotor.setTargetPosition(ticks);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //turn right
        leftPower(-power);
        rightPower(power);

        while(backLeftMotor.isBusy() && backRightMotor.isBusy()) {

        }

        stopDriving();
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");

        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left_drive");
        // rightMotor = hardwareMap.dcMotor.get("right_drive");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        //leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //drive 5 inches
        //driveDistance(1.0, 4, 60);
        //sleep(5000);
        ratioCheck(2000, 0.5);

    }


}