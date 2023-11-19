package org.firstinspires.ftc.teamcode;

public class Auto extends DriveRobot {

    // The 3 attributes mirror, back and name are set by the 
    // constructor of the sub class.

    // Should truns and stafe movements be mirrored?  1 for normal,
    // -1 for mirrored
    int mirror   = 1;

    // Is the starting position in the back or the front? This 
    // determines the direction and distiance to drive after placing
    // the pixel
    boolean back = true;

    // Name of the position, used to display on the screen before
    // starting the autonomous run
    String name  = "Auto";

    public void setParams() {
        mirror   = 1;
        back = true;
        name  = "Auto";        
    }

    /**
     * Main method that gets executed when the robot starts.
     */
    @Override public void runOpMode() throws InterruptedException {
        setParams();
        initRobot();
        grab();

        telemetry.addData("distance", getDistance());
        telemetry.addData("Mode", name+" mirror:"+mirror+" back:"+back);
        telemetry.addData(">", "Press Start");
        telemetry.update();  

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

    /**
     * Method that determines what the robot does during the 
     * autonomous session. This code is parameterized for the 
     * 4 possible start positions. It defaults to the moves
     * required for BLUE-BACK.
     */
    void auto() {
        // Is there a team-prop straight ahead? (position 2)
        drive(200);
        if(seeBlock(700)) {
            telemetry.addData("Status", "Detected prop at #2");
            telemetry.update(); 
            // Move a bit to the side to avoid prop    
            strafe(100*mirror);
            // Drive to line
            drive(540);/* At position 2 now (the one directly in front of start)*/
            // Drop pixel by backing up
            if(back) {
                    drive(-200);
                } else {
                    drive(-400);
                }
            // Navigate to parking area
            if(back) {
                park(-470, 0, 90*mirror);
            } else {
                park(0, 0, -90*mirror);
            }
        } else {
            telemetry.addData("Status", "Aligning with pos #1");
            telemetry.update();            
            // Move sideways to align with position 1
            strafe(300*mirror);
            // Is there a team-prop straight ahead? (position 1)
            if(seeBlock(600)) {
                telemetry.addData("Status", "Detected prop at #1");
                telemetry.update();                
                // Drive to line
                drive(350);/* At the edge of position 1 now */
                // Drop pixel by backing up
                if(back) {
                    drive(-200);
                } else {
                    drive(-400);
                }
                // Navigate to parking area
                if(back) {
                    park(-330, 0, 90*mirror);
                } else {
                    park(0, 0, -90*mirror);
                }                
            } else {
                telemetry.addData("Status", "Dropping pixel at #3");
                telemetry.update();                
                // Noting detected in pos 1 or 2 -> has to be in pos 3
                // Drive to align with pos 3
                drive(390);
                // Turn left
                turn(90*mirror);
                // Drive to the line
                drive(440); /* at pos 3 now */
                // Drop pixel by backing up
                if(back) {
                    drive(-200);
                } else {
                    drive(-400);
                }
                // Navigate to parking area
                if(back) {
                    park(0,-680*mirror,0);
                } else {
                    park(0, 0, 178);
                }
            }
        }
    }

    /**
     * Method to park the robot after droping a pixel on the spike 
     * marks.
     */
    void park(int driveDistance, int strafeDistance, int turnAmount) {
        telemetry.addData("Status", "Driving to parking area");
        telemetry.update(); 
        if(driveDistance!=0)drive(driveDistance);
        if(strafeDistance!=0)strafe(strafeDistance);
        if(turnAmount!=0)turn(turnAmount);
        
        //drive(200);
        //driveToDistance(700);
        //strafe(500*mirror);
        driveToDistance(420);
        
        telemetry.addData("Status", "Parked");
        telemetry.update();   
        wristUp();
        sleep(2000);
        release();
        sleep(2000);
        telemetry.addData("Status", "Dropped Pixel");
        telemetry.update(); 
    }    
}