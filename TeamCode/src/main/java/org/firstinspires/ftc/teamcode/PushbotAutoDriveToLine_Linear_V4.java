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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "sensor_light")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Beacon Try 4", group="Auto")
@Disabled
public class PushbotAutoDriveToLine_Linear_V4 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareGompersV0_Integrated robot = new HardwareGompersV0_Integrated();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    // could also use HardwarePushbotMatrix class.
    ColorSensor ColorSensor;      // Primary LEGO Light sensor,
    // OpticalDistanceSensor   lightSensor;   // Alternative MR ODS sensor

    static final double     COUNTS_PER_MOTOR_REV   = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     REVERSE_SPEED            =0.6;
    static final double     armMotor = .5;
    static final double WHITE_THRESHOLD = 16;  // spans between 0.1 - 0.5 from dark to light
    static final double APPROACH_SPEED = 0.5;
    static final int SEARCHING = 1;
    static final int DING = 2;
    static final int TURN = 3;
    static final int PRESS = 4;
    int state, newState;

    @Override
    public void runOpMode() {
        if (state == SEARCHING) {
            float hsvValues[] = {0F, 0F, 0F};
            final float values[] = hsvValues;
            final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
            robot.init(hardwareMap);

            // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotor.getCurrentPosition();
            robot.rightMotor.getCurrentPosition();

            // get a reference to our Light Sensor object.
            ColorSensor = hardwareMap.colorSensor.get("Color_Sensor");                // Primary LEGO Light Sensor
            //  lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.
            boolean bLedOn = true;
            boolean bPrevState = false;
            boolean bCurrState = false;
            // turn on LED of light sensor.
            ColorSensor.enableLed(bLedOn);
            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Ready to run");    //
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            // Abort this loop is started or stopped.
            while (!(isStarted() || isStopRequested())) {
                Color.RGBToHSV(ColorSensor.red() * 8, ColorSensor.green() * 8, ColorSensor.blue() * 8, hsvValues);
                // check the status of the x button on either gamepad.
                bCurrState = gamepad1.x;
                // update previous state variable.
                bPrevState = bCurrState;
                // check for button state transitions.
                if ((bCurrState == true) && (bCurrState != bPrevState)) {

                    // button is transitioning to a pressed state. So Toggle LED
                    bLedOn = !bLedOn;
                    ColorSensor.enableLed(bLedOn);
                }
// send the info back to driver station using telemetry function.
                telemetry.addData("LED", bLedOn ? "On" : "Off");
                telemetry.addData("Clear", ColorSensor.alpha()); //white
                telemetry.addData("Red  ", ColorSensor.red());
                telemetry.addData("Green", ColorSensor.green());
                telemetry.addData("Blue ", ColorSensor.blue());
                telemetry.addData("Hue", hsvValues[0]);
                // Display the light level while we are waiting to start
                // telemetry.addData("Light Level", ColorSensor.getLightDetected());
                // telemetry.update();
                //idle();
            }

            // Start the robot moving forward, and then begin looking for a white line.
            robot.leftMotor.setPower(APPROACH_SPEED);
            robot.rightMotor.setPower(APPROACH_SPEED);
            newState = DING;
            if (state == DING) {
                while (opModeIsActive() && (ColorSensor.alpha() < WHITE_THRESHOLD)) {
//  if (opModeIsActive() && (ColorSensor.alpha() < WHITE_THRESHOLD)) {
                    // Display the light level while we are looking for the line
                    telemetry.addData("Light Level", ColorSensor.alpha());
                    telemetry.update();


                    // Stop all motors
                    int inches = 0;
                    double timeout = 5.0;
                    encoderDrive(DRIVE_SPEED, inches, inches, timeout);  // S1: Forward 24 Inches with 5 Sec timeout
                    newState = TURN;
                }
            }
        }  else if (newState == TURN) {
            int inches = 10;
            double timeout = 5.0;
            encoderDrive(DRIVE_SPEED, -inches, inches, timeout);  // S1: Forward 24 Inches with 5 Sec timeout
            // State Transition:
            //      - state = STOP
            newState = PRESS;
        } else if (newState == PRESS) {
            int inches = 5;
            double timeout = 5.0;
            encoderDrive(DRIVE_SPEED, inches, inches, timeout);  // S1: Forward 24 Inches with 5 Sec timeout
        }
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        robot.leftMotor.setPower(Math.abs(speed));
        robot.rightMotor.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while ((runtime.seconds() < timeoutS) &&
                (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }
}

//if statement now in SEARCHING STATEMENT