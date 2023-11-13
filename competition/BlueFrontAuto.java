package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueFrontAuto", group ="Concept")
public class BlueFrontAuto extends Auto {

    // Similar to BlueBack but strafe and turns are mirrored,
    // Parking strategy is different as we are in the front of 
    // the field.

    // Override as -1 for mirrored positions 
    @Override int mirror() {
        return -1;
    }

    @Override void parkFrom1() {
        park(0, 0, -90*mirror());
    }

    @Override void parkFrom2() {
        park(0, 0, -90*mirror());
    }

    @Override void parkFrom3() {
        park(0, 0, 180);
    }
}