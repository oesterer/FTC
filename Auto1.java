package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.*;

@Autonomous(name="Auto1", group ="Concept")
public class Auto1 extends DriveRobot {

    @Override public void runOpMode() throws InterruptedException {
        init();
        waitForStart();
        auto2();
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

    @Override void auto2() {
        // Is there a team-prop straight ahead? (position 2)
        if(seeBlock(900)) {
            // Move a bit to the side to avoid prop    
            strafe(100);
            // Drive to line
            drive(740);/* At position 2 now (the one directly in front of start)*/
            // Drop pixel by backing up
            drive(-200);
            // Navigate to parking area
            park(-540, 0, 90);
        } else {
            // Move sideways to align with position 1
            strafe(320);
            // Is there a team-prop straight ahead? (position 1)
            if(seeBlock(800)) {
                // Drive to line
                drive(550);/* At the edge of position 1 now */
                // Drop pixel by backing up
                drive(-200);
                // Navigate to parking area
                park(-340, 0, 90);
            } else {
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
        drive(driveDistance);
        strafe(strafeDistance);
        if(turnAmount>0)turn(turnAmount);
        drive(2300);
    }

}
