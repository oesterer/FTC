package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(name="DriveRobot", group ="Concept")
public class DriveRobot extends LinearOpMode
{

/*

Robot motor layout:

Motor1 --- Motor2

  |    ^     |
  |    |     |

Motor3 --- Motor4


Drive forward:

Fwd --- Fwd

  |      |
  |      |

Fwd --- Fwd

Drive Sideways:

Fwd --- Fwd

  |      |
  |      |

Rev --- Rev

Turn:

Fwd --- Rev

  |      |
  |      |

Fwd --- Rev

To drive / turn the opposite direction, flip direction of all motors.

*/

    private DcMotor    motor1   = null;
    private DcMotor    motor2   = null;
    private DcMotor    motor3   = null;
    private DcMotor    motor4   = null;

    @Override
    public void runOpMode()
    {
        motor1  = hardwareMap.get(DcMotor.class, "motor1");
        motor2  = hardwareMap.get(DcMotor.class, "motor2");
        motor3  = hardwareMap.get(DcMotor.class, "motor3");
        motor4  = hardwareMap.get(DcMotor.class, "motor4");

        // Configure the motors so that a positive power makes the wheel 
        // turn forward
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.FORWARD);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        telemetry.addData(">", "Press Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            // The 4 output values to calculate based on inputs
            double motor1Power = 0;
            double motor2Power = 0;
            double motor3Power = 0;
            double motor4Power = 0;
            
            // Weight or scale we apply from the stick input to the 
            // motor output
            double driveScale=-0.75;
            double sideScale=0.75;
            double turnScale=0.5;

            // Get the three stick inputs
            double driveInput = gamepad1.right_stick_y;
            double sideInput  = gamepad1.right_stick_x;
            double turnInput  = gamepad1.left_stick_x;

            // Combine the three inputs, with their respective scale, and 
            // compute the corresponsing motor output
            motor1Power = driveScale*driveInput
                          +sideScale*sideInput
                          +turnScale*turnInput;
            motor2Power = driveScale*driveInput
                          +sideScale*sideInput
                          -turnScale*turnInput;
            motor3Power = driveScale*driveInput
                          -sideScale*sideInput
                          +turnScale*turnInput;
            motor4Power = driveScale*driveInput
                          -sideScale*sideInput
                          -turnScale*turnInput;

            // We now could have values greater than 1 or less than -1. Use
            // this block to find the maximum (in absolute terms) value.
            double scale=1.0;
            scale=Math.abs(motor1Power)>scale?Math.abs(motor1Power):scale;
            scale=Math.abs(motor2Power)>scale?Math.abs(motor2Power):scale;
            scale=Math.abs(motor3Power)>scale?Math.abs(motor3Power):scale;
            scale=Math.abs(motor4Power)>scale?Math.abs(motor4Power):scale;

            // Set the output, but divide the value by the max value we determined
            // above. -> no motor will be set above 1 or below -1 and the combination
            // of driving and turning will still be in effect.
            motor1.setPower(motor1Power/scale);
            motor2.setPower(motor2Power/scale);
            motor3.setPower(motor3Power/scale);
            motor4.setPower(motor4Power/scale);

            // For debugging, write all the motor values to the driver station
            // screen
            telemetry.addData("motor1", motor1Power/scale);
            telemetry.addData("motor2", motor2Power/scale);
            telemetry.addData("motor3", motor3Power/scale);
            telemetry.addData("motor4", motor4Power/scale);

            telemetry.addData("driveInput", driveInput);
            telemetry.addData("sideInput",  sideInput);
            telemetry.addData("turnInput",  turnInput);

            telemetry.update();
            sleep(10);
        }
    }
}
