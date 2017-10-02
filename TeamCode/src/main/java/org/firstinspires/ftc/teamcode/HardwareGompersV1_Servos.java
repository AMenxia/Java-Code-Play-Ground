  package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

  /**
   * This is NOT an opmode.
   * This define all components for Future Robot extensions/OpModes
   *
   * This class is all the combinations of ALL hardware Gompers Versions.
   * This class is NOT to be updated unless deemed important updates to main components
   *    If you wish to add new components, make a new Java Class file
   *
   * This hardware class assumes the following device names have been configured on the robot:
   * Note:  All names are lower case and some have single spaces between words.
   *
   * Motor channel:  leftMotor:       "left_Motor"              Left wheel motor
   * Motor channel:  rightMotor:      "right_Motor"             Right wheel motor
   * Motor channel:  ballPusher:      "ball_Pusher"             Ejects Balls onto ramp
   * Servo channel:  elevator:        "elevator"                Lifting up cap ball
*/

  public class HardwareGompersV1_Servos
  {
      /* Public OpMode members. */
      public DcMotor leftMotor;
      public DcMotor rightMotor;
      public Servo elevator;
      public DcMotor ballPusher;
      public ColorSensor colorsensor;


      public final static double ARM_HOME = 0.2;
      public final static double CLAW_HOME = 0.2;
      public final static double ARM_MIN_RANGE  = 0.20;
      public final static double ARM_MAX_RANGE  = 0.90;
      public final static double CLAW_MIN_RANGE  = 0.09;
      public final static double CLAW_MAX_RANGE  = 0.99;

      //public DcMotor ExtraFutureMotor;

      /*Variables*/
      public static final double MID_SERVO       =  0.5 ;


      /* local OpMode members. */
      HardwareMap hwMap           =  null;
      private ElapsedTime period  = new ElapsedTime();

      /* Constructor */
      public HardwareGompersV1_Servos(){

      }

      /* Initialize standard Hardware interfaces */
      public void init(HardwareMap ahwMap) {
          /* Save reference to Hardware map*/
          hwMap = ahwMap;

          /*Define and Initialize Motors*/
          leftMotor    = hwMap.dcMotor.get("left_Motor");
          rightMotor    = hwMap.dcMotor.get("right_Motor");
          //ExtraFutureMotor    = hwMap.dcMotor.get("FutureMotor");
          ballPusher    = hwMap.dcMotor.get("ball_Pusher");
          colorsensor = hwMap.colorSensor.get("color_sensor");


          /*Define and initialize servos*/
          elevator = hwMap.servo.get("elevator");


          /*Adjust Servos to Beginning Position*/
          elevator.setPosition(MID_SERVO);


          /*Set Motors to Foward/Reverse*/
          leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
          rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
          //ExtraFutureMotor.setDirection(DcMotor.Direction.FORWARD);
          ballPusher.setDirection(DcMotor.Direction.FORWARD);

          /*Set all motors to zero power*/
          leftMotor.setPower(0.00);
          rightMotor.setPower(0.00);
          ballPusher.setPower(0.00);
          //ExtraFutureMotor.setPower(0.00);



          /* Set all motors to run without encoders.*/
          /* May want to use RUN_USING_ENCODERS if encoders are installed.*/
          leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //with encoders
          rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //with encoders
          ballPusher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          //ExtraFutureMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  //without encoders


          /*Define and initialize ALL installed servos.*/
          elevator = hwMap.servo.get("elevator");
          //leftClaw.setPosition(MID_SERVO);
          //rightClaw.setPosition(MID_SERVO);
      }

      /***
       *
       * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
       * periodic tick.  This is used to compensate for varying processing times for each cycle.
       * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
       *
       * @param periodMs  Length of wait cycle in mSec.
       */
      public void waitForTick(long periodMs) {

          long  remaining = periodMs - (long)period.milliseconds();
          // sleep for the remaining portion of the regular cycle period.
          if (remaining > 0) {
              try {
                  Thread.sleep(remaining);
              } catch (InterruptedException e) {
                  Thread.currentThread().interrupt();
              }
          }

          // Reset the cycle clock for the next pass.
          period.reset();
      }
  }

