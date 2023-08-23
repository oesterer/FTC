public class MyFIRSTJavaOpMode extends LinearOpMode {

  // Local variables referencing our two motors. You can find the class definition here:
  // https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotor.html
  DcMotor motorLeft;
  DcMotor motorRight;

  DistanceSensor distance1;
  ColorSensor color1;

  // Main method that gets called once the robot is ready to run
  @Override
  public void runOpMode() {

    // Get a refrence to the left and right motor so we can 
    // easily use it in our program
    motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
    motorRight = hardwareMap.get(DcMotor.class, "motorRight");

    // Get the color and distance sensor references
    color1 = hardwareMap.get(ColorSensor.class, "color1");
    distance1 = hardwareMap.get(DistanceSensor.class, "distance1");

    // Wait for the start button to get pushed, in the simulator this 
    // happens automatically.
    waitForStart();

    // Use the telemetry class to write a status message to the screen
    telemetry.addData("My Status", "Hello, I'm just getting started");
    // Need to call update() method once all data is added / updated
    telemetry.update();
    
    // Modify these movement method calls to solve the challenges
    drive(200);
    turn(90);
    // Add as many drive and turn calls as you need

    // Update the message on the screen.
    telemetry.addData("My Status", "I'm done driving");
    telemetry.update();

    // Done, driving challenge using methods accomplished!
  }  

  /**
   * Method to drive the robot forward by a specified number of centimeters
   * 
   * @param distance Distance in cm of how far we want the robot to drive
   */
  public void drive(int distance) {
    // Constants to use when driving the robot

    // To convert cm into motor position counter values
    final double DISTANCE_CONSTANT=45;
    // What power to use to drive the robot
    final double DRIVE_POWER=0.5;
    // How long to pause before checking movement
    final int SLEEP_INTERVAL=50;

    // Stop and reset the motor counter
    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Set the motor into the mode that uses the encoder to keep
    // track of the position
    motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // Set the target position by converting the distance into motor
    // position values
    motorLeft.setTargetPosition(-1*DISTANCE_CONSTANT*distance);
    // Set the motor power
    motorLeft.setPower(DRIVE_POWER);
    
    // Do the same for for right motor, but opposite direction
    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motorRight.setTargetPosition(DISTANCE_CONSTANT*distance);
    motorRight.setPower(DRIVE_POWER);

    // Sleep a bit to make sure the motor report as "busy"
    sleep(SLEEP_INTERVAL);
    // Loop as long as either motor reports as busy
    while(motorRight.isBusy() || motorLeft.isBusy()) {
      // Sleep until next check
      sleep(SLEEP_INTERVAL);
      // Write the telemetry to the console
      writeTelemetry();
    }
  } 

  /**
   * Method to turn the robot by a specified number of degrees.
   * 
   * @param degrees Number of degrees to turn the robot by. Provide 
   * a negative number to turn left.
   */
  public void turn(int degrees) {
    final double DEGREES_CONSTANT=15;
    final double TURNING_POWER=0.5;
    final int SLEEP_INTERVAL=50;

    // Stop and reset the motor counter
    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Set the motor into the mode that uses the encoder to keep
    // track of the position
    motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // Set the target position by converting the degrees into motor
    // position values
    motorLeft.setTargetPosition(DEGREES_CONSTANT*degrees);
    // Set the motor power
    motorLeft.setPower(TURNING_POWER);
    
    // Do the same for for right motor
    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motorRight.setTargetPosition(DEGREES_CONSTANT*degrees);
    motorRight.setPower(TURNING_POWER);

    // Sleep a bit to make sure the motor report as "busy"
    sleep(SLEEP_INTERVAL);
    // Loop as long as either motor reports as busy
    while(motorRight.isBusy() || motorLeft.isBusy()) {
      // Sleep until next check
      sleep(SLEEP_INTERVAL);
      // Write the telemetry to the console
      writeTelemetry();
    }
  }

  /**
   * Method to drive the robot close to an object or wall. Uses the distance 
   * sensor
   * 
   * @param distanceToWall Distiance to object in cm when the robot should stop
   */ 
  public void driveCloseToWall(int distanceToWall) {

      // Stop both motors and use the power mode without encoders
      motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motorLeft.setPower(0);
      motorRight.setPower(0);
      sleep(50);

      // Get the sensor distance reading
      double distance = distance1.getDistance(DistanceUnit.CM);
  
      // Loop until we are within 5% of the specified distance. The 5% helps
      // with the time it takes for the robot to come to a stop. We can 
      // improve on this in future versions
      while(distance>(distanceToWall*1.05)) {
        writeTelemetry();
        // Set the motor power, low power improves accuracy
        motorLeft.setPower(-0.35);
        motorRight.setPower(0.35);
        sleep(50);
        // Get the distance after waiting for 50 milliseconds
        distance = distance1.getDistance(DistanceUnit.CM);
      } 
      // Stop the motors
      motorLeft.setPower(0);
      motorRight.setPower(0);
      sleep(50);

      writeTelemetry();
  }

  /**
   * Method to send the sensor values to the telemetry console.
   */
  public void writeTelemetry() {
    // Get the distance in centimeters from the sensor and write to console
    telemetry.addData("Distance", distance1.getDistance(DistanceUnit.CM));
    // Get the red, green, blue components of the color detected by the color sensor
    // and write to the console. Remember RGB
    telemetry.addData("Color", "R:"+color1.red()+" G:"+color1.green()+" B:"+color1.blue());
    telemetry.update();
  }
}
