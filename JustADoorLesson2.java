public class MyFIRSTJavaOpMode extends LinearOpMode {

  // Local variables referencing our two motors. You can find the class definition here:
  // https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotor.html
  DcMotor motorLeft;
  DcMotor motorRight;

  // Main method that gets called once the robot is ready to run
  @Override
  public void runOpMode() {

    // Get a refrence to the left and right motor so we can 
    // easily use it in our program
    motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
    motorRight = hardwareMap.get(DcMotor.class, "motorRight");

    // Wait for the start button to get pushed, in the simulator this 
    // happens automatically.
    waitForStart();

    // Use the telemetry class to write a status message to the screen
    telemetry.addData("My Status", "Hello, I'm just getting started");
    // Need to call update() method once all data is added / updated
    telemetry.update();
    
    // Modify these movement method calls to solve the challenges
    drive(700);
    turn(200);
    // Add as many drive and turn calls as you need

    // Update the message on the screen.
    telemetry.addData("My Status", "I'm done driving");
    telemetry.update();
  }    


  /**
   * Method to drive the robot forward for specified duration
   * 
   * @param time Time in milliseconds to drive the robot forward
   */
  public void drive(int time) {
    // Start driving! Set the power on the motors
    motorLeft.setPower(-1);
    motorRight.setPower(1);

    // Wait for the robot to drive as far as we need it
    sleep(time);

    // Stop the robot by putting the power on both motors to 0
    motorLeft.setPower(0);
    motorRight.setPower(0);
  }

  /**
   * Method to turn the robot for specified duration
   * 
   * @param time Time in milliseconds to turn the robot
   */
  public void turn(int time) {
    // Start turning! Set the power on the motors
    motorLeft.setPower(1);
    motorRight.setPower(1);

    // Wait for the robot to drive as far as we need it
    sleep(time);

    // Stop the robot by putting the power on both motors to 0
    motorLeft.setPower(0);
    motorRight.setPower(0);
  }  

}  
