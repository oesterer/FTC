package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueFrontAuto", group ="Concept", preselectTeleOp="DriveRobot")
public class BlueFrontAuto extends Auto {

    // Similar to BlueBack but strafe and turns are mirrored,
    // Parking strategy is different as we are in the front of 
    // the field.
    public void setParams() {
        mirror   = -1;
        back = false;
        name  = "Blue Front";
        leftField = true;        
    } 
}