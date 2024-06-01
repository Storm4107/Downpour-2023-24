package org.firstinspires.ftc.teamcode;
 
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 
@TeleOp
public class Color extends LinearOpMode {
    // Define a variable for our color sensor
    ColorSensor ColorSensorLeft;
    ColorSensor ColorSensorRight;
    
     private DcMotor LeftFront;
 
     private DcMotor RightFront;
 
     private DcMotor LeftBack;
 
     private DcMotor RightBack;
    
    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        ColorSensorLeft = hardwareMap.get(ColorSensor.class, "ColorSensorLeft");
        ColorSensorRight = hardwareMap.get(ColorSensor.class, "ColorSensorRight");
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
        
        
        // Wait for the Play button to be pressed
        waitForStart();
 
        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            telemetry.addData("Red", ColorSensorLeft.red());
            telemetry.addData("Blue", ColorSensorLeft.blue());
            telemetry.update();
            
            telemetry.addData("Red", ColorSensorRight.red());
            telemetry.addData("Blue", ColorSensorRight.blue());
            telemetry.update();
            
            
            
           { 
            if (ColorSensorLeft.red() < 2500){
             LeftFront.setPower(0.5);
             LeftBack.setPower(0.5);
            }
             else 
             LeftFront.setPower(0);
             LeftBack.setPower(0);
           }
           
           }
           
           if (ColorSensorRight.red() < 2500){
           RightBack.setPower(-0.5);
           RightFront.setPower(-0.5);
           }
           else 
           RightBack.setPower(0);
           RightFront.setPower(0);
        }
        
    }
