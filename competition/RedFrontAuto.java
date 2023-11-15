package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedFrontAuto", group ="Concept")
public class RedFrontAuto extends Auto {

    public void setParams() {
        mirror   = 1;
        back = false;
        name  = "Red Front";        
    }    
}