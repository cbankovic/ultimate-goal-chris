package org.firstinspires.ftc.teamcode.ringvision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name = "Ring Detector - OpenCV", group="OpenCV")
public class rvFindRingsOpenCV001 extends LinearOpMode {

    // https://www.youtube.com/watch?v=-QFoOCoaW7I
    // Class Members
    RingVision ringFinder = new RingVision();

    @Override
    public void runOpMode() throws InterruptedException {

        Initialize();

        waitForStart();

        // Setup Positions
        //  StartRedOutside
        //  StartRedInside
        //  StartBlueOutside
        //  StartBlueInside

        // Steps
        // 01 : Use camera to select correct target zone
        // 02 : Drive to target zone A, B, or C
        // 03 : Return to shooting zone
        // 04 : Shoot power shots                   [3 shots]
        // 04A: Pick up stack of rings of 1 or 4    [3 rings]
        // 04B: Shoot in high target                [3 shots]
        // 04C: Pick up last ring if 4              [1 ring ]
        // 04D: Shoot in high target                [1 ring ]
        // 05 : Park on shooting line


        while(opModeIsActive()){
            telemetry.addData("Analysis", ringFinder.getAnalysis());
            telemetry.addData("Position", ringFinder.getPosition());

            switch (ringFinder.getTargetZone()){
                case A:
                    // Go to Zone A
                    telemetry.addData("Target Zone", "Target Zone: A");
                    break;
                case B:
                    // Go to Zone B
                    telemetry.addData("Target Zone", "Target Zone: B");
                    break;
                case C:
                    // Go to Zone C
                    telemetry.addData("Target Zone", "Target Zone: C");
                    break;
                default:
                    // Do nothing
                    telemetry.addData("Target Zone", "Target Zone: UNKNOWN");
                    break;
            }

            telemetry.update();

            sleep(50);

        }
    }

    /*
     * Initialize all variables
     */
    private void Initialize() {
        ringFinder.Initialize(hardwareMap);
    }

}
