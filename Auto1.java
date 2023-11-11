package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.List;
    
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.*;

@Autonomous(name="Auto1", group ="Concept")
public class Auto1 extends DriveRobot {

	@Override public void runOpMode() throws InterruptedException {
		init();
		waitForStart();
		auto2();
	}

	@Override void auto2() {
        if(seeBlock(900)){
            strafe(100);
            drive(740);/* At position 2 now (the one directly in front of start)*/
            drive(-200);
            park(-540, 0, 90);
        }else{
            strafe(320);
            if(seeBlock(800)){
                drive(550);/* At the edge of position 1 now */
                drive(-200);
                park(-340, 0, 90);
            }else{
                drive(590);
                turn(90);
                drive(440); /* at pos 3 now */
                drive(-200);
                park(0, -700,0);
            }
        }
    }

    @Override void park(int driveDistance, int strafeDistance, int turnAmount) {
        drive(driveDistance);
        strafe(strafeDistance);
        if(turnAmount>0)turn(turnAmount);
        drive(2300);
    }

}