package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.*;
import java.util.List;

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
    DistanceSensor distanceSensor = null;
    IMU imu = null;
 
    void initRobot() {
        imu = hardwareMap.get(IMU.class, "imu");
        
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
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.FORWARD);

        loadDrone();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addData("distance", getDistance());
        telemetry.addData(">", "Press Start");
        telemetry.update();        
    }

    @Override public void runOpMode() throws InterruptedException {
        initRobot();
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
                sleep(1000);
                loadDrone();
                telemetry.addData("Action", "Launch Drone");
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

            telemetry.update();
            sleep(10);
        }
    }

    void grab() {
        claw1.setPosition(0.5);
        claw2.setPosition(0.5);
    }

    void release() {
        claw1.setPosition(0);
        claw2.setPosition(1);
    }

    void wristUp() {
        wrist.setPosition(0.35);
    }

    void wristDown() {
        wrist.setPosition(0.65);
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

        for(DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    double getTurnPower(double remainingAngle) {
        return((remainingAngle+10)/100);
    }

    void driveToDistance(int targetDistance) {
        double currentDistance=getDistance();
        double remainingDistance=currentDistance-targetDistance;

        while(remainingDistance>0) {

            double power=getDrivePower(remainingDistance);
            for(DcMotor motor : motors) {
                motor.setPower(power);
            }
            currentDistance=getDistance();
            remainingDistance=currentDistance-targetDistance;
        } 

        for(DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    double getDrivePower(double remainingDistance) {
        // What power to use to drive the robot
        final double DRIVE_POWER=0.7;
        // What power to use to drive the robot
        final double MIN_POWER=0.1;
        // Deceleration distance
        final double DECEL_DIST=800.0;

        if(remainingDistance>=DECEL_DIST) {
            return DRIVE_POWER;
        } else {
            return((DRIVE_POWER-MIN_POWER)*(remainingDistance/DECEL_DIST)+MIN_POWER);
        }   
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
    
            // Determine the closest distance to either starting position
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
}