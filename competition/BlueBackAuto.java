package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueBackAuto", group ="Concept", preselectTeleOp="DriveRobot")
public class BlueBackAuto extends Auto {

    
    public void setParams() {
        mirror   = 1;
        back = true;
        name  = "Blue Back";
        leftField = true;        
    }    
    
}