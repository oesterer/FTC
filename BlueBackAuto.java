package org.firstinspires.ftc.teamcode;

@Autonomous(name="BlueBackAuto", group ="Concept")
public class BlueBackAuto extends DriveRobot {

    @Override public void runOpMode() throws InterruptedException {
        init();
        waitForStart();
        auto();
    }
 

/*  Field Layout:

Parking     /
-----------/

       --3--
             |
Robot        2
             |
       --1--

*/

    @Override void auto() {
        // Is there a team-prop straight ahead? (position 2)
        if(seeBlock(900)) {
            telemetry.addData("Status", "Detected prop at #2");
            telemetry.update(); 
            // Move a bit to the side to avoid prop    
            strafe(100);
            // Drive to line
            drive(740);/* At position 2 now (the one directly in front of start)*/
            // Drop pixel by backing up
            drive(-200);
            // Navigate to parking area
            park(-540, 0, 90);
        } else {
            telemetry.addData("Status", "Aligning with pos #1");
            telemetry.update();            
            // Move sideways to align with position 1
            strafe(320);
            // Is there a team-prop straight ahead? (position 1)
            if(seeBlock(800)) {
                telemetry.addData("Status", "Detected prop at #1");
                telemetry.update();                
                // Drive to line
                drive(550);/* At the edge of position 1 now */
                // Drop pixel by backing up
                drive(-200);
                // Navigate to parking area
                park(-340, 0, 90);
            } else {
                telemetry.addData("Status", "Dropping pixel at #3");
                telemetry.update();                
                // Noting detected in pos 1 or 2 -> has to be in pos 3
                // Drive to align with pos 3
                drive(590);
                // Turn left
                turn(90);
                // Drive to the line
                drive(440); /* at pos 3 now */
                // Drop pixel by backing up
                drive(-200);
                // Navigate to parking area
                park(0, -700,0);
            }
        }
    }

    @Override void park(int driveDistance, int strafeDistance, int turnAmount) {
        telemetry.addData("Status", "Driving to parking area");
        telemetry.update(); 
        drive(driveDistance);
        strafe(strafeDistance);
        if(turnAmount>0)turn(turnAmount);
        drive(2300);
        telemetry.addData("Status", "Parked");
        telemetry.update();         
    }

}
