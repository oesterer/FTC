package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name="AprilTest", group ="Concept")
public class AprilTest extends LinearOpMode
{
    private VisionPortal visionPortal;               
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode()
    {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        telemetry.addData(">", "Press Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            List<AprilTagDetection> detectedTags = aprilTag.getDetections();
            for (AprilTagDetection tag : detectedTags) {
                if (tag.metadata != null)  {
                    telemetry.addData("Tag", "ID %d (%s)", tag.id, tag.metadata.name);
                    telemetry.addData("Range",  "%5.1f inches", tag.ftcPose.range);
                    telemetry.addData("Bearing","%3.0f degrees", tag.ftcPose.bearing);
                }
            }

            telemetry.update();
            sleep(10);
        }

    }
}
