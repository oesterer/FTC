package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedBackAuto", group ="Concept")
public class RedBackAuto extends BlueBackAuto {

    // Same as BlueBack but strafe and turns are mirrored
    public void RedBackAuto() {
        this.mirror = -1;
        this.back   = true;
        this.name   = "Red Back";        
    }

}