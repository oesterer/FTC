package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueFrontAuto", group ="Concept")
public class BlueFrontAuto extends DriveRobot {

    @Override public void runOpMode() throws InterruptedException {
        initRobot();
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

    void auto() {

        // Is there a team-prop straight ahead? (position 2)
        if(seeBlock(900)) {
            telemetry.addData("Status", "Detected prop at #2");
            telemetry.update(); 
            // Move a bit to the side to avoid prop    
            strafe(-100);
            // Drive to line
            drive(740);/* At position 2 now (the one directly in front of start)*/
            // Drop pixel by backing up
            drive(-200);
            // Navigate to parking area
            park(0, 0, 90);
        } else {
            telemetry.addData("Status", "Aligning with pos #1");
            telemetry.update();            
            // Move sideways to align with position 1
            strafe(-320);
            // Is there a team-prop straight ahead? (position 1)
            if(seeBlock(800)) {
                telemetry.addData("Status", "Detected prop at #1");
                telemetry.update();                
                // Drive to line
                drive(550);/* At the edge of position 1 now */
                // Drop pixel by backing up
                drive(-200);
                // Navigate to parking area
                park(0, 0, 90);
            } else {
                telemetry.addData("Status", "Dropping pixel at #3");
                telemetry.update();                
                // Noting detected in pos 1 or 2 -> has to be in pos 3
                // Drive to align with pos 3
                drive(590);
                // Turn left
                turn(-90);
                // Drive to the line
                drive(440); /* at pos 3 now */
                // Drop pixel by backing up
                drive(-200);
                // Navigate to parking area
                park(0, 0, 180);
            }
        }
    }
}