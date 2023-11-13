package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedBackAuto", group ="Concept")
public class RedBackAuto extends DriveRobot {

    // Same as BlueBack but strafe and turns are mirrored

    // Override as -1 for mirrored positions 
    @Override int mirror() {
        return 1;
    }
}