// MADE BY REICHEN MERIT


//* Copyright (c) 2017 FIRST. All rights reserved.
 /*
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

 package org.firstinspires.ftc.teamcode.OverhauledCode;

 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
 import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
 import org.firstinspires.ftc.robotcore.external.JavaUtil;
 import com.qualcomm.hardware.bosch.BNO055IMU;
 import com.qualcomm.hardware.dfrobot.HuskyLens;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.util.ElapsedTime;
 import com.qualcomm.robotcore.util.Range;
 
 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 
 
 import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
 import org.firstinspires.ftc.robotcore.internal.system.Deadline;
 
 import org.firstinspires.ftc.robotcore.external.Func;
 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
 import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
 import org.firstinspires.ftc.robotcore.external.navigation.Position;
 import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
 import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
  import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
 
 import java.util.concurrent.TimeUnit;
 
 
 /*
  * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
  * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
  * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
  * class is instantiated on the Robot Controller and executed.
  *
  * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
  * It includes all the skeletal structure that all linear OpModes contain.
  *
  * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
  * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
  */
 @Autonomous(name="TurnWithGyro", group="Linear OpMode")
 
 public class TurnWithGyro extends LinearOpMode {
 
     // Declare OpMode members.
     private DcMotor LeftFront;
 
     private DcMotor RightFront;
 
     private DcMotor LeftBack;
 
     private DcMotor RightBack;

     private ElapsedTime runtime = new ElapsedTime();

     
    private  BNO055IMU imu;
 
     static final double ENCODER_CLICKS = 28;    // REV 20:1  1120
     static final double DRIVE_GEAR_REDUCTION = 20/1;     // This is < 1.0 if geared UP
     static final double WHEEL_CIRC = 3.78;     // For figuring circumference
     static final double COUNTS_PER_INCH = (ENCODER_CLICKS * DRIVE_GEAR_REDUCTION) /
             (WHEEL_CIRC * 3.1415);
     static final double DRIVE_SPEED = 1.0;
     static final double TURN_SPEED = 0.5;
     static final double speed = 0.5;
     
     double LensPosition = 0;
     
       Orientation angles;
 
 
 //Huskylens variables
     private final int READ_PERIOD = 1;
 
     private HuskyLens huskyLens;
     
      
     private final static int LED_PERIOD = 10;
     private final static int GAMEPAD_LOCKOUT = 500;
     
     //RevBlinkinLedDriver Blinkin;
     //RevBlinkinLedDriver.BlinkinPattern pattern;
 
     @Override
     public void runOpMode() {
      
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
   
       
 
         
         
         telemetry.addData("Status", "Initialized");
 
 //Motor statements
        LeftFront = hardwareMap.get(DcMotor.class, "FrontLeft");
        RightFront = hardwareMap.get(DcMotor.class, "FrontRight");
        LeftBack = hardwareMap.get(DcMotor.class, "BackLeft");
        RightBack = hardwareMap.get(DcMotor.class, "BackRight");
   


        
         RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

 
        
         RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        
         LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
         RightFront.setDirection(DcMotor.Direction.REVERSE);
         RightBack.setDirection(DcMotor.Direction.REVERSE);
         LeftFront.setDirection(DcMotor.Direction.REVERSE);
         LeftBack.setDirection(DcMotor.Direction.REVERSE);
        
         
     //huskylens statements
          huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
          
        
         /*
          * This sample rate limits the reads solely to allow a user time to observe
          * what is happening on the Driver Station telemetry.  Typical applications
          * would not likely rate limit.
          */
         Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
 
         /*
          * Immediately expire so that the first time through we'll do the read.
          */
         rateLimit.expire();
 
         if (!huskyLens.knock()) {
             telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
         } else {
             telemetry.addData(">>", "Press start to continue");
         }
 
         huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
 
         // Wait for the game to start (driver presses PLAY)
         waitForStart();
           angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         runtime.reset();
         
             //left distance, right distance, speed, runtime.
             //Distances are in F  if (opModeIsActive()) {

            
      double IMUxvel = 0;
      double IMUyvel = 0;
      
      double IMUx = 0;
      double IMUy= 0;
            
            while (opModeIsActive()) {
             telemetry.addData("Yaw value", angles.firstAngle);
  

  //Drivetrain
      
    
         if (gamepad1.a){
          turnLeft(30);
         }
          
  
  //telemetry
              telemetry.addData("IMU Status", imu.getSystemStatus());
              telemetry.update();
           


            }
            
            
        }
        
         public void turnLeft(double angle)
    {
        //MOVE RIGHT
      
          LeftFront.setPower(-0.3);
          LeftBack.setPower(-0.3);
          RightFront.setPower(-0.3);
          RightBack.setPower(-0.3);
        // Continue until robot yaws right by 90 degrees or stop is pressed on Driver Station.
        sleep(1000);
        //turns to the right; 90, 180, negative, -90, 0
        while ( !(angles.firstAngle >= angle || isStopRequested()) ) {
         angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // Update Yaw-Angle variable with current yaw.
            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw value", angles.firstAngle);
            telemetry.update();
        }
        // We're done. Turn off motors
       LeftFront.setPower(0);
          LeftBack.setPower(0);
          RightFront.setPower(0);
          RightBack.setPower(0);
        // Pause so final telemetry is displayed.
        sleep(1000);
    }

 }
 
