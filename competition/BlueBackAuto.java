package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueBackAuto", group ="Concept")
public class BlueBackAuto extends Auto {

    public void BlueBackAuto() {
        this.mirror = 1;
        this.back   = true;
        this.name   = "Blue Back";        
    }
    
}