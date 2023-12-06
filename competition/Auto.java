package org.firstinspires.ftc.teamcode;

public class Auto extends DriveRobot {

    boolean debug=false;
    int debugSleep=1000;

    int right = 1;
    int left = 2;
    int center =3;

    // The 4 attributes mirror, back, leftField and name are set by the 
    // constructor of the sub class.

    // Should truns and stafe movements be mirrored?  1 for normal,
    // -1 for mirrored
    int mirror   = 1;

    // Is the starting position in the back or the front? This 
    // determines the direction and distiance to drive after placing
    // the pixel
    boolean back = true;

    // Left or right field? (red is right, blue is left)
    boolean leftField = false;

    // Name of the position, used to display on the screen before
    // starting the autonomous run
    String name  = "Auto";

    // The results of the randomozation, displayed as an int
    // 0 - Unknown
    // 1 - Parking Position 1
    // 2 - Parking Position 2
    // 3 - Parking Position 3
    int randomization = 0;
    
    public void setParams() {
        mirror   = 1;
        back = true;
        name  = "Auto";    
        randomization = 0;
    }

    /**
     * Main method that gets executed when the robot starts.
     */
    @Override public void runOpMode() throws InterruptedException {
        setParams();
        initRobot();
        grab();

        telemetry.addData("distance", getDistance());
        telemetry.addData("Mode", name+" mirror:"+mirror+" back:"+back+" left:"+leftField);
        telemetry.addData("Randomization", randomization);
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
            telemetry.addData("Status", "Detected prop at #2 center");
            telemetry.update();
            if(debug)sleep(1000);  
            randomization = center;
            // Move a bit to the side to avoid prop    
            strafe(100*mirror);
            // Drive to line
            drive(590);/* At position 2 now (the one directly in front of start)*/
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
                randomization=(mirror==1)?right:left;
                telemetry.addData("Status", "Detected prop at #1 "+((mirror==1)?"right":"left"));
                telemetry.update();
                if(debug)sleep(1000);                 
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
                    park(-310, 0, 90*mirror);
                } else {
                    park(0, 0, -90*mirror);
                }                
            } else {
                randomization=(mirror==1)?left:right;
                telemetry.addData("Status", "Dropping pixel at #3 "+((mirror==1)?"left":"right"));
                telemetry.update();
                if(debug)sleep(1000);                 
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
        
        // Hug the wall
        if(leftField) {
            telemetry.addData("Status", "Left wall, strafe "+(150-getDistanceL()));
            telemetry.update();
            if(debug)sleep(debugSleep);
            strafe(150-getDistanceL());
        } else {
            telemetry.addData("Status", "Right wall, strafe "+(-150+getDistanceR()));
            telemetry.update();
            if(debug)sleep(debugSleep);            
            strafe(-150+getDistanceR());
        }        
        
        // Drive to the position from which to strafe for pixel drop
        if(back) {
            drive(1600);
        } else {
            drive(400);
        } 

        // Distance for center
        int dist = 550;
        // Difference from center for left or right
        int delta = 140; 
        if (leftField){
            if(randomization == left) {
                dist -= delta;
            } else if(randomization == right) {
                dist += delta;
            }   
        } else {
            if(randomization == right) {
                dist -= delta;
            } else if(randomization == left) {
                dist += delta;
            }                     
        }
         
        if(leftField) {
            telemetry.addData("Status", "Left field, strafe "+(dist));
            telemetry.update();
            if(debug)sleep(debugSleep);            
            strafe(dist);
        } else {
            telemetry.addData("Status", "Right field, strafe "+(-1*dist));
            telemetry.update();
            if(debug)sleep(debugSleep); 
            strafe(-1*dist);
        }
        
        extendLift();
        sleep(1000);
        stopLift();
        driveToDistance(245);
        
        telemetry.addData("Status", "Dropping Pixel");
        telemetry.update();   
        wristUp();
        sleep(1000);
        release();
        sleep(500);
        drive(-60);

        telemetry.addData("Status", "Parked");
        telemetry.update(); 
    }    
}