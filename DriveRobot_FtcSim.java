public class MyFIRSTJavaOpMode extends LinearOpMode {

    private DcMotor    motor1   = null;
    private DcMotor    motor2   = null;
    private DcMotor    motor3   = null;
    private DcMotor    motor4   = null;

    CRServo leftWheel;
    CRServo rightWheel;
    DcMotor wrist;
    DcMotor leftShoulder;
    DcMotor rightShoulder;
    BNO055IMU imu;
    
    @Override
    public void runOpMode()
    {
        Gamepad gamepad1 = new FTCGamepad();

        leftWheel = hardwareMap.get(CRServo.class, "leftWheel");
        rightWheel = hardwareMap.get(CRServo.class, "rightWheel");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        leftShoulder = hardwareMap.get(DcMotor.class, "leftShoulder");
        rightShoulder = hardwareMap.get(DcMotor.class, "rightShoulder");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        motor1  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        motor2  = hardwareMap.get(DcMotor.class, "frontRightDrive");
        motor3  = hardwareMap.get(DcMotor.class, "backLeftDrive");
        motor4  = hardwareMap.get(DcMotor.class, "backRightDrive");

        // Configure the motors so that a positive power makes the wheel 
        // turn forward
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive())
        {
            // The 4 output values to calculate based on inputs
            double motor1Power = 0;
            double motor2Power = 0;
            double motor3Power = 0;
            double motor4Power = 0;
            
            // Weight or scale we apply from the stick input to the 
            // motor output
            double driveScale=0.75;
            double sideScale=0.75;
            double turnScale=0.5;

            // Get the three stick inputs
            double driveInput = 0; 
            double sideInput  = 0;
            double turnInput  = 0;
            
            // https://www.ascii-code.com/
            
            // y and h
            if(gamepad1.pressKey(121)>0) {
                driveInput=1;
            } else if(gamepad1.pressKey(104)>0) {
                driveInput=-1;
            }
            
            // u and i
            if(gamepad1.pressKey(117)>0) {
                sideInput=-1;
            } else if(gamepad1.pressKey(105)>0) {
                sideInput=1;
            }
            
            // j and k
            if(gamepad1.pressKey(106)>0) {
                turnInput=-1;
            } else if(gamepad1.pressKey(107)>0) {
                turnInput=1;
            }

            // x
            if(gamepad1.pressKey(120)>0) {
                //wrist.setPower(1);
                rightShoulder.setPower(-1);
                sleep(1500);
                rightShoulder.setPower(0);
                drive(1500);
                turn(90);
                drive(-50);
                strafe(-2000);
                sleep(1000);
                auto();
            }

            // Combine the three inputs, with their respective scale, and 
            // compute the corresponsing motor output
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

            // We now could have values greater than 1 or less than -1. Use
            // this block to find the maximum (in absolute terms) value.
            double scale=1.0;
            scale=Math.abs(motor1Power)>scale?Math.abs(motor1Power):scale;
            scale=Math.abs(motor2Power)>scale?Math.abs(motor2Power):scale;
            scale=Math.abs(motor3Power)>scale?Math.abs(motor3Power):scale;
            scale=Math.abs(motor4Power)>scale?Math.abs(motor4Power):scale;

            // Set the output, but divide the value by the max value we determined
            // above. -> no motor will be set above 1 or below -1 and the combination
            // of driving and turning will still be in effect.
            motor1.setPower(motor1Power/scale);
            motor2.setPower(motor2Power/scale);
            motor3.setPower(motor3Power/scale);
            motor4.setPower(motor4Power/scale);

            // For debugging, write all the motor values to the driver station
            // screen
            telemetry.addData("motor1", motor1Power/scale);
            telemetry.addData("motor2", motor2Power/scale);
            telemetry.addData("motor3", motor3Power/scale);
            telemetry.addData("motor4", motor4Power/scale);

            telemetry.addData("driveInput", driveInput);
            telemetry.addData("sideInput",  sideInput);
            telemetry.addData("turnInput",  turnInput);
            
            telemetry.update();
            sleep(10);
        }
    }

  /**
   * Method to drive the robot forward by a specified number of centimeters
   * 
   * @param distance Distance in mm of how far we want the robot to drive
   */
  public void drive(int distance) {
    // Constants to use when driving the robot

    // To convert cm into motor position counter values
    final double DISTANCE_CONSTANT=5;
    // What power to use to drive the robot
    final double DRIVE_POWER=0.5;
    // How long to pause before checking movement
    final int SLEEP_INTERVAL=50;

    // Stop and reset the motor counter
    motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Set the motor into the mode that uses the encoder to keep
    // track of the position
    motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // Set the target position by converting the distance into motor
    // position values
    motor1.setTargetPosition(DISTANCE_CONSTANT*distance);
    motor2.setTargetPosition(DISTANCE_CONSTANT*distance);
    motor3.setTargetPosition(DISTANCE_CONSTANT*distance);
    motor4.setTargetPosition(DISTANCE_CONSTANT*distance);
    // Set the motor power
    motor1.setPower(DRIVE_POWER);
    motor2.setPower(DRIVE_POWER);
    motor3.setPower(DRIVE_POWER);
    motor4.setPower(DRIVE_POWER);

    // Sleep a bit to make sure the motor report as "busy"
    sleep(SLEEP_INTERVAL);
    // Loop as long as either motor reports as busy
    while(motor1.isBusy() || motor2.isBusy() || motor3.isBusy() || motor4.isBusy()) {
      // Sleep until next check
      sleep(SLEEP_INTERVAL);
    }

    motor1.setPower(0);
    motor2.setPower(0);
    motor3.setPower(0);
    motor4.setPower(0);
    motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    
  } 

  /**
   * Method to drive the robot sideways by a specified number of centimeters
   * 
   * @param distance Distance in mm of how far we want the robot to drive
   */
  public void strafe(int distance) {
    // Constants to use when driving the robot

    // To convert cm into motor position counter values
    final double DISTANCE_CONSTANT=5;
    // What power to use to drive the robot
    final double DRIVE_POWER=0.5;
    // How long to pause before checking movement
    final int SLEEP_INTERVAL=50;

    // Stop and reset the motor counter
    motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Set the motor into the mode that uses the encoder to keep
    // track of the position
    motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // Set the target position by converting the distance into motor
    // position values
    motor1.setTargetPosition(DISTANCE_CONSTANT*distance);
    motor2.setTargetPosition(-1*DISTANCE_CONSTANT*distance);
    motor3.setTargetPosition(-1*DISTANCE_CONSTANT*distance);
    motor4.setTargetPosition(DISTANCE_CONSTANT*distance);
    // Set the motor power
    motor1.setPower(DRIVE_POWER);
    motor2.setPower(DRIVE_POWER);
    motor3.setPower(DRIVE_POWER);
    motor4.setPower(DRIVE_POWER);

    // Sleep a bit to make sure the motor report as "busy"
    sleep(SLEEP_INTERVAL);
    // Loop as long as either motor reports as busy
    while(motor1.isBusy() || motor2.isBusy() || motor3.isBusy() || motor4.isBusy()) {
      // Sleep until next check
      sleep(SLEEP_INTERVAL);
    }

    motor1.setPower(0);
    motor2.setPower(0);
    motor3.setPower(0);
    motor4.setPower(0);
    motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    
  } 

  /**
   * Method to turn the robot by a specified number of degrees.
   * 
   * @param degrees Number of degrees to turn the robot by. Provide 
   * a negative number to turn left.
   */
  public void turn(int degrees) {
    final double DEGREES_CONSTANT=32.5;
    final double TURNING_POWER=0.5;
    final int SLEEP_INTERVAL=50;

    // Stop and reset the motor counter
    motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Set the motor into the mode that uses the encoder to keep
    // track of the position
    motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // Set the target position by converting the degrees into motor
    // position values
    motor1.setTargetPosition(DEGREES_CONSTANT*degrees);
    motor2.setTargetPosition(-1*DEGREES_CONSTANT*degrees);
    motor3.setTargetPosition(DEGREES_CONSTANT*degrees);
    motor4.setTargetPosition(-1*DEGREES_CONSTANT*degrees);
    // Set the motor power
    motor1.setPower(TURNING_POWER);
    motor2.setPower(TURNING_POWER);
    motor3.setPower(TURNING_POWER);
    motor4.setPower(TURNING_POWER);

    // Sleep a bit to make sure the motor report as "busy"
    sleep(SLEEP_INTERVAL);
    // Loop as long as either motor reports as busy
    while(motor1.isBusy() || motor2.isBusy() || motor3.isBusy() || motor4.isBusy()) {
      // Sleep until next check
      sleep(SLEEP_INTERVAL);
    }

    motor1.setPower(0);
    motor2.setPower(0);
    motor3.setPower(0);
    motor4.setPower(0);
    motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }


    void auto() {
        boolean detectedBlue=false;

        drive(-250);
        strafe(585);
        sleep(3000);
        //detectedBlue=isBlue();
        if(detectedBlue /*|| gamepad1.x*/) {       
            drive(-100);
            sleep(1000);
        } else {
            drive(300);
            strafe(150);
            sleep(3000);
            //detectedBlue=isBlue();
            if(detectedBlue /*|| gamepad1.x*/) {
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

}
