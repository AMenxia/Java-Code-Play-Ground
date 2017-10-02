package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by amenx on 10/1/2017.
 */
@Autonomous (name = "Simple sensor test teaching self java", group = "Sensor")
//@Disabled
public class Sensor_test_2 extends LinearOpMode {
	SensorMRColor sensorMRColor= new SensorMRColor();
	HardwareGompersV0_Integrated robot = new HardwareGompersV0_Integrated();
	ColorSensor colorSensor;    // Hardware Device Object
	 @Override
	public void runOpMode(){
		if (colorSensor.red()> 2) {
			robot.ballPusher.setPower(.5);
		}
		 while (robot.ballPusher.isBusy()) {
			 robot.elevator.setPosition(.9);
		 }
		 if (colorSensor.red() >2);{
			 robot.elevator.setPosition(.01);
		 }}}

