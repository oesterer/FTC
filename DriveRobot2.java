package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
    
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(name="DriveRobot", group ="Concept")
public class DriveRobot extends LinearOpMode
{
    private DcMotor    motor1   = null;
    private DcMotor    motor2   = null;
    private DcMotor    motor3   = null;
    private DcMotor    motor4   = null;
    private Servo      launcher = null;
    private Servo      claw     = null;
    private Servo      arm      = null;
    private DcMotor    lift     = null;
    private DcMotor    motorTest   = null;
    private boolean    isLiftMoving = false;
    private boolean     isPlaneLaunched = false;  
    private boolean armUp=false;
    private boolean clawClosed=false;
    private ColorSensor colorSensor = null;
    //private DistanceSensor 

  
    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();

    @Override public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward

        updateOrientation();
        
        initTfod();
        
        motor1  = hardwareMap.get(DcMotor.class, "motor1");
        motor2  = hardwareMap.get(DcMotor.class, "motor2");
        motor3  = hardwareMap.get(DcMotor.class, "motor3");
        motor4  = hardwareMap.get(DcMotor.class, "motor4");
        launcher = hardwareMap.get(Servo.class, "launcher");
        claw    = hardwareMap.get(Servo.class, "claw");
        arm     = hardwareMap.get(Servo.class, "arm");
        lift    = hardwareMap.get(DcMotor.class, "lift");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.FORWARD);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        telemetry.addData(">", "Press Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            double motor1Power = 0;
            double motor2Power = 0;
            double motor3Power = 0;
            double motor4Power = 0;
            
            double driveScale=-1;
            double sideScale=1;
            double turnScale=0.75;

            double driveInput = gamepad1.right_stick_y;
            double sideInput  = gamepad1.right_stick_x;
            double turnInput  = gamepad1.left_stick_x;

            motor1Power = driveScale*driveInput
                          +sideScale*sideInput
                          +turnScale*turnInput;
            motor2Power = driveScale*driveInput
                          +sideScale*sideInput
                          -turnScale*turnInput;
            motor3Power = driveScale*driveInput
                          -sideScale*sideInput
                          +turnScale*turnInput;
            motor4Power = driveScale*driveInput
                          -sideScale*sideInput
                          -turnScale*turnInput;
            double scale = 3;
            if (!gamepad1.right_bumper) {
                scale = 1.0;
            }
            
            scale=Math.abs(motor1Power)>scale?Math.abs(motor1Power):scale;
            scale=Math.abs(motor2Power)>scale?Math.abs(motor2Power):scale;
            scale=Math.abs(motor3Power)>scale?Math.abs(motor3Power):scale;
            scale=Math.abs(motor4Power)>scale?Math.abs(motor4Power):scale;

            motor1.setPower(motor1Power/scale);
            motor2.setPower(motor2Power/scale);
            motor3.setPower(motor3Power/scale);
            motor4.setPower(motor4Power/scale);

            telemetry.addData("motor1", motor1Power/scale);
            telemetry.addData("motor2", motor2Power/scale);
            telemetry.addData("motor3", motor3Power/scale);
            telemetry.addData("motor4", motor4Power/scale);
            
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            //telemetry.addData("driveInput", driveInput);
            //telemetry.addData("sideInput",  sideInput);
            //telemetry.addData("turnInput",  turnInput);
            //telemetry.addData("Plane Launched", isPlaneLaunched);
            telemetry.addData("colorSensor", "r:"+ colorSensor.red() +" g:"+ colorSensor.green() +" b:"+ colorSensor.blue());

            if(gamepad1.dpad_right) {
                if(gamepad1.right_bumper) {
                    turn(-15);
                } else {
                    turn(-90);
                }
            }

            if(gamepad1.dpad_left) {
                if(gamepad1.right_bumper) {
                    turn(15);
                } else {
                    turn(90);
                }
            }

            if(gamepad1.dpad_up && launcher.getPosition() == 1){
                launchDrone();
                telemetry.addData("Action", "Launch Drone");
            } else if(gamepad1.dpad_up){
                loadDrone();
                telemetry.addData("Action", "Load Drone");
            }
            
            if(gamepad1.b) {
                if(clawClosed) {
                    release();
                    clawClosed=false;
                    telemetry.addData("Action", "Release Claw");
                } else {
                    grab();
                    clawClosed=true;
                    telemetry.addData("Action", "Close Claw");
                }
                sleep(500);
            }

            if(gamepad1.y) {
                if(armUp) {
                    armDown();
                    armUp=false;
                    telemetry.addData("Action", "Arm Down");
                } else {
                    armUp();
                    armUp=true;
                    telemetry.addData("Action", "Arm Up");
                }
                sleep(500);
            }
            
            if(gamepad1.x) {
                armHalfway();
                telemetry.addData("Action", "Arm Halfway");
            }

            if(gamepad1.a && gamepad1.x) {
                drive2(10);
                //drive(0.1, 0.5);
                //strafe(2.5, 0.6);
                telemetry.addData("Action", "Auto");
            }

            if(gamepad1.right_trigger>0.1) {
                extendLift();
                telemetry.addData("Action", "Lift Extend");
            }

            if(gamepad1.left_trigger>0.1) {
                contractLift();
                telemetry.addData("Action", "Lift Contract");
            }

            if(isLiftMoving &&    
               gamepad1.left_trigger<=0.1 &&
               gamepad1.right_trigger<=0.1) {
               stopLift();
               telemetry.addData("Action", "Lift Stop");
            }

            telemetryTfod();
            telemetry.update();
            sleep(10);
        }
    }

    void turntopixel() {
        motor1.setPower(-0.25);
        motor2.setPower(0.25);
        motor3.setPower(-0.25);
        motor4.setPower(0.25);
        while(true){
         List<Recognition> currentRecognitions = tfod.getRecognitions();

         if(currentRecognitions.size()>0)break;
        
        }

         motor1.setPower(0);
         motor2.setPower(0);
         motor3.setPower(0);
         motor4.setPower(0);

    }

    void grab() {
        claw.setPosition(0.75);
    }

    void release() {
        claw.setPosition(0.4);
    }

    void armUp() {
        arm.setPosition(0.35);
    }

    void armDown() {
        arm.setPosition(1);
    }

    void armHalfway() {
        arm.setPosition(0.85);
    }

    void turn(int angle) {
        imu.resetYaw();
        double currentAngle=getAngle();
        int direction=0;
        double targetAngle=currentAngle+angle;
        double remainingAngle=Math.abs(targetAngle-currentAngle);

        if(angle<0) {
            direction=-1;
        } else {
            direction=1;
        }

        while(remainingAngle>0.5) {

            double power=getPower(remainingAngle);
            motor1.setPower(-1*direction*power);
            motor2.setPower(1*direction*power);
            motor3.setPower(-1*direction*power);
            motor4.setPower(1*direction*power);
            currentAngle=getAngle();
            remainingAngle=Math.abs(targetAngle-currentAngle);
        } 

        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0); 
    }

    double getPower(double remainingAngle) {
        return((remainingAngle+10)/100);
    }

    double getAngle() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return(orientation.getYaw(AngleUnit.DEGREES));
    }

    void loadDrone() {
        launcher.setPosition(1);
    }

    void launchDrone() {
        launcher.setPosition(0);
        isPlaneLaunched = true;
        sleep(1000);
    }

    void extendLift() {
        lift.setPower(.85);
        isLiftMoving = true; 
    }

    void contractLift() {
        lift.setPower(-.85);
        isLiftMoving = true; 
    }

    void stopLift() {
        lift.setPower(0);
        isLiftMoving = false;
    }

    void drive(double distance, double speed) {
        int direction=0;
        if(distance<0) {
            direction=-1;
        } else {
            direction=1;
        }

        motor1.setPower(speed*direction);
        motor2.setPower(speed*direction);
        motor3.setPower(speed*direction);
        motor4.setPower(speed*direction); 
        sleep(distanceToTime(distance));
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }

    void strafe(double distance, double speed) {
        int direction=0;
        if(distance<0) {
            direction=-1;
        } else {
            direction=1;
        }

        motor1.setPower(speed*direction);
        motor2.setPower(speed*direction);
        motor3.setPower(-1*speed*direction);
        motor4.setPower(-1*speed*direction); 
        sleep(distanceToTime(distance));
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }   

    long distanceToTime(double distance) {
        return((long)(Math.abs(distance)*1000));
    } 

    boolean isWhite() {
        return(colorSensor.blue()>=150 &&
               colorSensor.red()>=150 &&
               colorSensor.green()>=150);
    }

    boolean driveToWhite(double distance) {
        int direction=0;
        if(distance<0) {
            direction=-1;
        } else {
            direction=1;
        }
        motor1.setPower(0.5*direction);
        motor2.setPower(0.5*direction);
        motor3.setPower(0.5*direction);
        motor4.setPower(0.5*direction); 

        long maxTime=distanceToTime(distance);
        long elapsedTime=0;
        while(elapsedTime<=maxTime &&
              !isWhite()) {
            sleep(50);
            elapsedTime=elapsedTime+50;
        }
        
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);

        return(isWhite());
    }

      public void drive2(int distance) {
    // Constants to use when driving the robot

    // To convert cm into motor position counter values
    final double DISTANCE_CONSTANT=45;
    // What power to use to drive the robot
    final double DRIVE_POWER=0.5;
    // How long to pause before checking movement
    final int SLEEP_INTERVAL=50;

    // Stop and reset the motor counter
    motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Set the motor into the mode that uses the encoder to keep
    // track of the position
    motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // Set the target position by converting the distance into motor
    // position values
    motor1.setTargetPosition((int)DISTANCE_CONSTANT*distance);
    // Set the motor power
    motor1.setPower(DRIVE_POWER);
    
    // Do the same for for right motor, but opposite direction
    motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor2.setTargetPosition((int)DISTANCE_CONSTANT*distance);
    motor2.setPower(DRIVE_POWER);

    motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor3.setTargetPosition((int)DISTANCE_CONSTANT*distance);
    motor3.setPower(DRIVE_POWER);

    motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor4.setTargetPosition((int)DISTANCE_CONSTANT*distance);
    motor4.setPower(DRIVE_POWER);



    // Sleep a bit to make sure the motor report as "busy"
    sleep(SLEEP_INTERVAL);
    // Loop as long as either motor reports as busy
    while(motor1.isBusy() || motor2.isBusy() || motor3.isBusy() || motor4.isBusy()) {
      // Sleep until next check
      sleep(SLEEP_INTERVAL);
      // Write the telemetry to the console
      //writeTelemetry();
    }
  } 

    void auto() {
        boolean detectedWhite=false;

        strafe(-0.5,0.5);
        detectedWhite=driveToWhite(0.5);
        if(detectedWhite) {
            armDown();
            release();
        } else {
             turn(90);
             detectedWhite=driveToWhite(0.5);
             if(detectedWhite){
                armDown();
                release();
             } else {
                turn(90);
                detectedWhite=driveToWhite(0.5);
                if(detectedWhite){
                armDown();
                release();
                }  
             }
        }
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

            // Use setModelAssetName() if the TF Model is built in as an asset.
            // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
            //.setModelAssetName(TFOD_MODEL_ASSET)
            //.setModelFileName(TFOD_MODEL_FILE)

            //.setModelLabels(LABELS)
            //.setIsModelTensorFlow2(true)
            //.setIsModelQuantized(true)
            //.setModelInputSize(300)
            //.setModelAspectRatio(16.0 / 9.0)

            .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        //if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        //} else {
        //    builder.setCamera(BuiltinCameraDirection.BACK);
        //}

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
    void updateOrientation() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            orientationIsValid = true;
        } catch (IllegalArgumentException e) {
            orientationIsValid = false;
        }
    }

}
