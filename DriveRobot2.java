package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.*;


import java.util.List;
    
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(name="DriveRobot", group ="Concept")
public class DriveRobot extends LinearOpMode
{
    // Array to hold all 4 motors, this can be use in loops
    // such at this  " for(DcMotor motor : motors) {} " to execute 
    // action on all 4 motors.
    DcMotor    motors[]  = new DcMotor[4];

    DcMotor    motor1   = null;
    DcMotor    motor2   = null;
    DcMotor    motor3   = null;
    DcMotor    motor4   = null;
    Servo      launcher = null;
    Servo      claw1    = null;
    Servo      claw2    = null;
    Servo      wrist    = null;
    DcMotor    liftR    = null;
    DcMotor    liftL    = null;
    DcMotor    motorTest   = null;
    boolean    isLiftMoving = false;
    boolean    isPlaneLaunched = false;  
    boolean    wristUp=false;
    boolean    clawClosed=false;
    ColorSensor    colorSensor = null;
    DistanceSensor distanceSensor = null;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    VisionPortal visionPortal;
    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();

    void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward

        updateOrientation();
        
        initTfod();
        
        motor1  = hardwareMap.get(DcMotor.class, "motor1");
        motor2  = hardwareMap.get(DcMotor.class, "motor2");
        motor3  = hardwareMap.get(DcMotor.class, "motor3");
        motor4  = hardwareMap.get(DcMotor.class, "motor4");
        motors[0]=(motor1);
        motors[1]=(motor2);
        motors[2]=(motor3);
        motors[3]=(motor4);

        launcher = hardwareMap.get(Servo.class, "launcher");
        claw1    = hardwareMap.get(Servo.class, "claw1");
        claw2    = hardwareMap.get(Servo.class, "claw2");
        wrist    = hardwareMap.get(Servo.class, "wrist");
        liftR    = hardwareMap.get(DcMotor.class, "liftR");
        liftL    = hardwareMap.get(DcMotor.class, "liftL");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.FORWARD);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        telemetry.addData(">", "Press Start");
        telemetry.update();        
    }

    @Override public void runOpMode() throws InterruptedException {
        init();
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
                          -sideScale*sideInput
                          -turnScale*turnInput;
            motor3Power = driveScale*driveInput
                          -sideScale*sideInput
                          +turnScale*turnInput;
            motor4Power = driveScale*driveInput
                          +sideScale*sideInput
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
            telemetry.addData("distance", getDistance());
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
                if(wristUp) {
                    wristDown();
                    wristUp=false;
                    telemetry.addData("Action", "Wrist Down");
                } else {
                    wristUp();
                    wristUp=true;
                    telemetry.addData("Action", "Wrist Up");
                }
                sleep(500);
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

            if(gamepad1.x) {
                auto2();
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
        claw1.setPosition(0.75);
        claw2.setPosition(0.4);
    }

    void release() {
        claw1.setPosition(0.4);
        claw2.setPosition(0.75);
    }

    void wristUp() {
        wrist.setPosition(0.35);
    }

    void wristDown() {
        wrist.setPosition(1);
    }

    void wristHalfway() {
        wrist.setPosition(0.85);
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

            double power=getTurnPower(remainingAngle);
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

    double getTurnPower(double remainingAngle) {
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
        liftL.setPower(.85);
        liftR.setPower(-.85);
        isLiftMoving = true; 
    }

    void contractLift() {
        liftL.setPower(-.85);
        liftR.setPower(.85);
        isLiftMoving = true; 
    }

    void stopLift() {
        liftL.setPower(0);
        liftR.setPower(0);
        isLiftMoving = false;
    }

    double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.MM);
    }   

    boolean seeBlock(int distance) {
        if(getDistance()<=distance) {
            return true;
        } else {
            return false;
        }
    }

    long distanceToTime(double distance) {
        return((long)(Math.abs(distance)*1000));
    } 

    boolean isBlue() {
        return(colorSensor.blue()>=250 &&
               colorSensor.red()<=100 &&
               colorSensor.green()<=100);
    }

    public void drive(int distance) {
        // Constants to use when driving the robot

        // To convert cm into motor position counter values
        final double DISTANCE_CONSTANT=2;
        // What power to use to drive the robot
        final double DRIVE_POWER=0.8;
        // What power to use to drive the robot
        final double MIN_POWER=0.1;
        // How long to pause before checking movement
        final int SLEEP_INTERVAL=10;
        // Acceleration distance (in encoder clicks). 300mm in this case:
        final double ACCEL_DIST=300.0*DISTANCE_CONSTANT;

        int targetPosition=(int)DISTANCE_CONSTANT*distance;

        for(DcMotor motor : motors) {
            // Stop and reset the motor counter
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Set the motor into the mode that uses the encoder to keep
            // track of the position
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Set the target position by converting the distance into motor
            // position values
            motor.setTargetPosition(targetPosition);            
        }
        
        telemetry.addData("motor1",motor1.getCurrentPosition());
        telemetry.update();

        // Sleep a bit to make sure the motor report as "busy"
        sleep(SLEEP_INTERVAL);
        // Loop as long as either motor reports as busy
        boolean isBusy=false;
        do {

            int currentPosition=motor1.getCurrentPosition();
            telemetry.addData("motor1",currentPosition);
            telemetry.update();
    
            // Determine the closest distiance to either starting position
            // or target. When close to start, we accelerate, when close to 
            // target, we decelerate. When we are far from both, the robot 
            // drives at DRIVE_POWER speed. To avoid not moving at all, the
            // minimum speed is set to MIN_POWER. The distance over which to 
            // accerate or decelerate is ACCEL_DIST. All math is done in 
            // encoder "clicks", 300 mm is about 600 encoder clicks.
            int lengthToTarget=Math.abs(targetPosition-currentPosition);
            if (lengthToTarget>Math.abs(currentPosition)) {
                lengthToTarget=Math.abs(currentPosition);
            }
            
            double power=(DRIVE_POWER-MIN_POWER)*(lengthToTarget/ACCEL_DIST)+MIN_POWER;
            if(lengthToTarget>=ACCEL_DIST) {
                power=DRIVE_POWER;
            }
            
            for(DcMotor motor : motors) {
              motor.setPower(power);
            }
    
            // Sleep until next check
            sleep(SLEEP_INTERVAL);
            isBusy=false;
            for(DcMotor motor : motors) {
                if(motor.isBusy())isBusy=true;
            }
        } while(isBusy);

        for(DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }        
    } 



    public void strafe(int distance) {
        // Constants to use when driving the robot

        // To convert cm into motor position counter values
        final double DISTANCE_CONSTANT=2;
        // What power to use to drive the robot
        final double DRIVE_POWER=0.8;
        // What power to use to drive the robot
        final double MIN_POWER=0.1;
        // How long to pause before checking movement
        final int SLEEP_INTERVAL=10;
        // Acceleration distance (in encoder clicks). 300mm in this case:
        final double ACCEL_DIST=300.0*DISTANCE_CONSTANT;

        int targetPosition=(int)DISTANCE_CONSTANT*distance;
        int motorNumber=0;
        for(DcMotor motor : motors) {
            // Stop and reset the motor counter
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Set the motor into the mode that uses the encoder to keep
            // track of the position
            
            if(motorNumber==1 || motorNumber==2) {
                motor.setTargetPosition(targetPosition*-1);
            } else {
                motor.setTargetPosition(targetPosition);
            }
            
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Set the target position by converting the distance into motor
            // position values
            

            motorNumber++;            
        }
        
        telemetry.addData("motor1",motor1.getCurrentPosition());
        telemetry.update();

        // Sleep a bit to make sure the motor report as "busy"
        sleep(SLEEP_INTERVAL);
        // Loop as long as either motor reports as busy
        boolean isBusy=false;
        do {

            int currentPosition=motor1.getCurrentPosition();
            telemetry.addData("motor1",currentPosition);
            telemetry.update();
    
            // Determine the closest distiance to either starting position
            // or target. When close to start, we accelerate, when close to 
            // target, we decelerate. When we are far from both, the robot 
            // drives at DRIVE_POWER speed. To avoid not moving at all, the
            // minimum speed is set to MIN_POWER. The distance over which to 
            // accerate or decelerate is ACCEL_DIST. All math is done in 
            // encoder "clicks", 300 mm is about 600 encoder clicks.
            int lengthToTarget=Math.abs(targetPosition-currentPosition);
            if (lengthToTarget>Math.abs(currentPosition)) {
                lengthToTarget=Math.abs(currentPosition);
            }
            
            double power=(DRIVE_POWER-MIN_POWER)*(lengthToTarget/ACCEL_DIST)+MIN_POWER;
            if(lengthToTarget>=ACCEL_DIST) {
                power=DRIVE_POWER;
            }
            
            for(DcMotor motor : motors) {
              motor.setPower(power);
            }
    
            // Sleep until next check
            sleep(SLEEP_INTERVAL);
            isBusy=false;
            for(DcMotor motor : motors) {
                if(motor.isBusy())isBusy=true;
            }
        } while(isBusy);

        for(DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }        
    } 



/*
    void auto() {
        boolean detectedBlue=false;

        drive(-250);
        strafe(585);
        sleep(3000);
        detectedBlue=isBlue();
        if(detectedBlue || gamepad1.x) {       
            drive(-100);
            sleep(1000);
        } else {
            drive(300);
            strafe(150);
            sleep(3000);
            detectedBlue=isBlue();
            if(detectedBlue || gamepad1.x) {
                drive(-100);
                sleep(1000);
            } else {
                sleep(3000);
                strafe(-125);
                drive(100);
                sleep(3000);
                drive(-100);
                sleep(1000);
            }
        }
    }
*/



    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    void initTfod() {

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
    void telemetryTfod() {

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


    void auto2() {
        if(seeBlock(900)){
            strafe(100);
            drive(740);/* At position 2 now (the one directly in front of start)*/
            drive(-200);
            park(-540, 0, 90);
        }else{
            strafe(320);
            if(seeBlock(800)){
                drive(550);/* At the edge of position 1 now */
                drive(-200);
                park(-340, 0, 90);
            }else{
                drive(590);
                turn(90);
                drive(440); /* at pos 3 now */
                drive(-200);
                park(0, -700,0);
            }
        }
    }

    void park(int driveDistance, int strafeDistance, int turnAmount) {
        drive(driveDistance);
        strafe(strafeDistance);
        if(turnAmount>0)turn(turnAmount);
        drive(2300);
    }
    
}
