package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueBackAuto", group ="Concept")
public class BlueBackAuto extends Auto {

    // Override as -1 for mirrored positions 
    @Override int mirror() {
        return 1;
    }
}