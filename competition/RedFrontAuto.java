package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedFrontAuto", group ="Concept")
public class RedFrontAuto extends BlueFrontAuto {

    // Override as -1 for mirrored positions 
    @Override int mirror() {
        return 1;
    }
}