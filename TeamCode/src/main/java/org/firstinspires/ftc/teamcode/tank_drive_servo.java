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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;



/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TankDrive+1Servo", group="Pushbot")
//@Disabled
public class tank_drive_servo extends OpMode {

	/* Declare OpMode members. */
	HardwareGompersV1_Servos robot_S       = new HardwareGompersV1_Servos();
	double          armPosition     = robot_S.ARM_HOME;                   // Servo safe position
	double          clawPosition    = robot_S.CLAW_HOME;                  // Servo safe position
	final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
	final double    ARM_SPEED       = 0.01 ;

	// use the class created to define a Pus                       hbot's hardware
	// could also use HardwarePushbotMatrix class.
//    double          clawOffset  = 0.0 ;                  // Servo mid position
//    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
	DcMotor leftMotor;
	DcMotor rightMotor;
	DcMotor ballPusher;
	Servo hand;
	//    DcMotor Catapult;
//    CRServo hand;
//    static final double DOWN = -.9;
//    static final double UP = .9;
	// * Code to run ONCE when the driver hits INIT
	//   */
	@Override
	public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */


		robot_S.init(hardwareMap);

		// Send telemetry message to signify robot waiting;
		telemetry.addData("Say", "Hello Driver");    //
	}

	/*
	 * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
	 */
	@Override
	public void init_loop() {
	}

	/*
	 * Code to run ONCE when the driver hits PLAY
	 */
	@Override
	public void start() {
	}

	/*
	 * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
	 */
	@Override
	public void loop() {
		double left;
		double right;
		double ballPusher;    //I dont know I should add these (May be junk)
		double Catapult;
		// Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
		//Tank Controls
//        left = gamepad1.left_stick_y;
//        right = -gamepad1.right_stick_y;
		//Linear Controls
		left  = -gamepad1.left_stick_y + gamepad1.right_stick_x;
		right = -gamepad1.left_stick_y - gamepad1.right_stick_x;
        /*Stick Configuration
        *           +
        *         +   -
        *           -          CONTROLLER CONFIGURATION^^^*/
		//ballPusher   = -gamepad1.right_trigger - gamepad1.left_trigger;


		if(gamepad1.left_bumper) { //feed out
			robot_S.ballPusher.setPower(-0.9);
		}else if (gamepad1.right_bumper) {  //feed in
			robot_S.ballPusher.setPower(0.9);
		}else
		{
			robot_S.ballPusher.setPower(0.0);
		}
		if (gamepad1.y){
			//robot_S.hand.setPosition(0.01);
			armPosition -= ARM_SPEED;
		}
		else if (gamepad1.a){
			//robot_S.hand.setPosition(0.90);
			armPosition += ARM_SPEED;
		}
		// Move both servos to new position.
		armPosition  = Range.clip(armPosition, robot_S.ARM_MIN_RANGE, robot_S.ARM_MAX_RANGE);
		robot_S.elevator.setPosition(armPosition);
/*        clawPosition = Range.clip(clawPosition, robot_S.CLAW_MIN_RANGE, robot_S
                .CLAW_MAX_RANGE);
        robot_S.hand.setPosition(clawPosition);*/

		// For CR Servos on MR/HiTechnic hardware, internal positions relate to speed as follows:
		//
		//      0   == full speed reverse
		//      128 == stopped
		//      255 == full speed forward
		//If no work try this ^^^^^^^^^
//        if(gamepad1.y) { //hand up
//            robot.hand.setPower(UP);
//        }else if (gamepad1.a) {  //hand Down (Ejection if Grabbed wrong ball)
//            robot.ballPusher.setPower(DOWN);
//        }else {
//            robot.ballPusher.setPower(0.0);
//        }
//
//
//        if(gamepad1.y) { //Catapult in launch Mode
//            robot.Catapult.setPower(UP);
//        }else if (gamepad1.a) {  //Reload Catapult
//            robot.Catapult.setPower(DOWN);
//        }else {
//            robot.Catapult.setPower(0.0);
//        }


		// Normalize the values so neither exceed +/- 1.0
		double max = Math.max((Math.abs(left)), Math.abs(right));

		if (max > 1.0)

		{
			left /= max;
			right /= max;

		}
		robot_S.leftMotor.setPower(left);
		robot_S.rightMotor.setPower(right);
//        robot.ballPusher.setPower(ballPusher);
		// Use gamepad left & right Bumpers to open and close the claw
//        if (gamepad1.right_bumper)
//            clawOffset += CLAW_SPEED;
//        else if (gamepad1.left_bumper)
//            clawOffset -= CLAW_SPEED;

		// Move both servos to new position.  Assume servos are mirror image of each other.
//        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//        robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
//        robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);

		// Use gamepad buttons to move the arm up (Y) and DOWN (A)
//        if (gamepad1.y)
//            robot.ballPusher.setPower(robot.ARM_UP_POWER);
//        else if (gamepad1.a)
//            robot.ballPusher.setPower(robot.ARM_DOWN_POWER);
//        else
//            robot.ballPusher.setPower(0.0);

		// Send telemetry message to signify robot running;
//        telemetry.addData("claw",  "Offset = %.2f", clawOffset);
		telemetry.addData("left",  "%.2f", left);
		telemetry.addData("right", "%.2f", right);
		telemetry.update();
		//telemetry.addData("ballPusher", "%.2f", ballPusher);
	}

	/*
	 * Code to run ONCE after the driver hits STOP
	 */
	@Override
	public void stop() {
	}

}