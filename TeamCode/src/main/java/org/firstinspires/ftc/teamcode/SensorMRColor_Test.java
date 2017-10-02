package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by amenx on 9/30/2017.
 * This is meant to update motors based on information
 */

@Autonomous(name = "Color_Sensor activate motor", group = "Sensor")
//@Disabled
public class SensorMRColor_Test extends LinearOpMode {
	HardwareGompersV0_Integrated robot   = new HardwareGompersV0_Integrated();
	ColorSensor colorSensor;    // Hardware Device Object
	DeviceInterfaceModule CDI;
	@Override
	public void runOpMode() {
		robot.init(hardwareMap);
		CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
		float hsvValues[] = {0F,0F,0F};
		final float values[] = hsvValues;
		boolean bPrevState = false;
		boolean bCurrState = false;
		boolean bLedOn = true;
		colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
		colorSensor.enableLed(bLedOn);
		waitForStart();
		while (opModeIsActive()) {
			bCurrState = gamepad1.x;
			if (bCurrState && (bCurrState != bPrevState))  {
				bLedOn = !bLedOn;
				colorSensor.enableLed(bLedOn);
			}
			bPrevState = bCurrState;

			// convert the RGB values to HSV values.
			Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

			// send the info back to driver station using telemetry function.
			telemetry.addData("LED", bLedOn ? "On" : "Off");
			telemetry.addData("Clear", colorSensor.alpha());
			telemetry.addData("Red  ", colorSensor.red());
			telemetry.addData("Green", colorSensor.green());
			telemetry.addData("Blue ", colorSensor.blue());
			telemetry.addData("Hue", hsvValues[0]);

			// change the background color to match the color detected by the RGB sensor.
			// pass a reference to the hue, saturation, and value array as an argument
			// to the HSVToColor method.
			if(colorSensor.blue() > colorSensor.green() || colorSensor.blue() > colorSensor.red()){
				CDI.setLED(1,true);
				CDI.setLED(0,false);
				robot.leftMotor.setPower(.5);
				robot.rightMotor.setPower(.5);

			}
			double i;
			if((colorSensor.red() > colorSensor.green()) || (colorSensor.red() > colorSensor.blue())){
				CDI.setLED(1,true);
				CDI.setLED(0,false);
				robot.leftMotor.setPower(.5);
				robot.rightMotor.setPower(.5);

			}
			if(colorSensor.blue() > 5 || colorSensor.blue() > 3){
				CDI.setLED(1,true);
				CDI.setLED(0,false);
				robot.ballPusher.setPower(.5);
/*public class  tri = (double sppe, double )*/
			}
			else{}
			while (colorSensor.blue() <= 4 ){
				colorSensor.enableLed(false);
			}


			}

			telemetry.update();
		/*while (opModeIsActive()&& (colorSensor.blue() < 5)){
		}*/
			};



	}