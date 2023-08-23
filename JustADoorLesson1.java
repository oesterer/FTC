public class MyFIRSTJavaOpMode extends LinearOpMode {

  // Local variables referencing our two motors. You can find the class definition here:
  // https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotor.html
  DcMotor motorLeft;
  DcMotor motorRight;

  // Main method that gets called once the robot is ready to run
  @Override
  public void runOpMode() {

    // Get a reference to the left and right motor so we can 
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
    
    // Start driving! Set the power on the motors
    motorLeft.setPower(-1);
    motorRight.setPower(1);

    // Wait 700 milliseconds (0.7 seconds) for the robot to drive as far
    // as we need it
    sleep(700);

    // Stop the robot by putting the power on both motors to 0
    motorLeft.setPower(0);
    motorRight.setPower(0);

    // Update the message on the screen.
    telemetry.addData("My Status", "I'm done driving");
    telemetry.update();

    // Done, first driving challenge accomplished.
  }    
}
