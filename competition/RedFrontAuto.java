package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedFrontAuto", group ="Concept")
public class RedFrontAuto extends Auto {

    public void RedFrontAuto() {
        this.mirror = 1;
        this.back   = false;
        this.name   = "Red Front";        
    }

}