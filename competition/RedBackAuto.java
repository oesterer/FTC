package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedBackAuto", group ="Concept")
public class RedBackAuto extends BlueBackAuto {

    // Same as BlueBack but strafe and turns are mirrored
    public void setParams() {
        mirror   = -1;
        back = true;
        name  = "Red Back";
        leftField = false;        
    }
}