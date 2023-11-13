package org.firstinspires.ftc.teamcode;

public class Auto extends DriveRobot {

    @Override public void runOpMode() throws InterruptedException {
        initRobot();
        waitForStart();
        auto();
    }
 
 	// Override as -1 for mirrored positions 
 	int mirror() {
 		return 1;
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

    void auto() {
        // Is there a team-prop straight ahead? (position 2)
        if(seeBlock(900)) {
            telemetry.addData("Status", "Detected prop at #2");
            telemetry.update(); 
            // Move a bit to the side to avoid prop    
            strafe(100*mirror());
            // Drive to line
            drive(740);/* At position 2 now (the one directly in front of start)*/
            // Drop pixel by backing up
            drive(-200);
            // Navigate to parking area
            parkFrom2();
        } else {
            telemetry.addData("Status", "Aligning with pos #1");
            telemetry.update();            
            // Move sideways to align with position 1
            strafe(320*mirror());
            // Is there a team-prop straight ahead? (position 1)
            if(seeBlock(800)) {
                telemetry.addData("Status", "Detected prop at #1");
                telemetry.update();                
                // Drive to line
                drive(550);/* At the edge of position 1 now */
                // Drop pixel by backing up
                drive(-200);
                // Navigate to parking area
                parkFrom1();
            } else {
                telemetry.addData("Status", "Dropping pixel at #3");
                telemetry.update();                
                // Noting detected in pos 1 or 2 -> has to be in pos 3
                // Drive to align with pos 3
                drive(590);
                // Turn left
                turn(90*mirror());
                // Drive to the line
                drive(440); /* at pos 3 now */
                // Drop pixel by backing up
                drive(-200);
                // Navigate to parking area
                parkFrom3();
            }
        }
    }

    void parkFrom1() {
    	park(-330, 0, 90*mirror());
    }

    void parkFrom2() {
    	park(-530, 0, 90*mirror());
    }

    void parkFrom3() {
    	park(0,-680*mirror(),0);
    }

    void park(int driveDistance, int strafeDistance, int turnAmount) {
        telemetry.addData("Status", "Driving to parking area");
        telemetry.update(); 
        if(driveDistance!=0)drive(driveDistance);
        if(strafeDistance!=0)strafe(strafeDistance);
        if(turnAmount!=0)turn(turnAmount);
        driveToDistance(300);
        telemetry.addData("Status", "Parked");
        telemetry.update();         
    }    
}
