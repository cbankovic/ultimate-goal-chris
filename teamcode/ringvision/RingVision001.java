package org.firstinspires.ftc.teamcode.ringvision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

class RingVision {

    // Class Members
    private OpenCvInternalCamera phoneCam;
    private RingDeterminationPipeline pipeline;

    // Initialize the camera and pipeline
    public void Initialize(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // we set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(() -> {
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
        });
    }

    public RingDeterminationPipeline.TargetZone getTargetZone(){
        switch (pipeline.position){
            case ONE:
                return RingDeterminationPipeline.TargetZone.B;
            case NONE:
                return RingDeterminationPipeline.TargetZone.A;
            case FOUR:
                return RingDeterminationPipeline.TargetZone.C;
            default:
                return RingDeterminationPipeline.TargetZone.Unknown;
        }
    }

    public int getAnalysis(){
        return pipeline.getAnalysis();
    }

    public RingDeterminationPipeline.RingPosition getPosition(){
        return pipeline.position;
    }
}

class RingDeterminationPipeline extends OpenCvPipeline {

    /*
     * Enum to determine ring position
     */
    public enum RingPosition {
        FOUR,
        ONE,
        NONE
    }

    public enum TargetZone {
        C,
        B,
        A,
        Unknown
    }
    /*
     * Colors
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    /*
     * Values to define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181, 98);

    static final int REGION_WIDTH = 35;
    static final int REGION_HEIGHT = 25;

    final int FOUR_RING_THRESHOLD = 150;
    final int ONE_RING_THRESHOLD = 141;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Other variables
     */
    Mat region1_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1;

    // Volatile since accessed by OpMod thread w/o synchronization
    public volatile RingPosition position = RingPosition.FOUR;

    /*
    This function takes the RGB frame, converts it to YCrCb
    and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input){
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }

    @Override
    public void init(Mat firstFrame){
        inputToCb(firstFrame);
        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
    }

    @Override
    public Mat processFrame(Mat input){
        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];

        Imgproc.rectangle(
                input,          // Buffer to draw on
                region1_pointA, // First point which defines rectangle
                region1_pointB, // Second point which defines rectangle
                BLUE,           // The color the rectangle is drawn in
                2);     // Thickness of the rectangle lines

        position = RingPosition.FOUR; // Record our analysis
        if (avg1 > FOUR_RING_THRESHOLD){
            position = RingPosition.FOUR;
        } else if (avg1 > ONE_RING_THRESHOLD){
            position = RingPosition.ONE;
        } else {
            position = RingPosition.NONE;
        }

        Imgproc.rectangle(
                input,          // Buffer to draw on
                region1_pointA, // First point which defines rectangle
                region1_pointB, // Second point which defines rectangle
                GREEN,          // The color the rectangle is drawn in
                -1);    //  Negative thickness means solid fill

        return input;
    }

    public int getAnalysis(){
        return avg1;
    }
}
