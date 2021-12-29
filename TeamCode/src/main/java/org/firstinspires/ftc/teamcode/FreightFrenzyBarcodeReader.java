package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCameraException;

@TeleOp
public class FreightFrenzyBarcodeReader extends LinearOpMode {
    //RingDetectionPipeline pipeline;
    WebcamName webcam;
    OpenCvWebcam camera;
    ColorPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException{
        //camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);

        camera.setMillisecondsPermissionTimeout(2500);
        pipeline = new ColorPipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(640, 480,OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error Opening Camera");
                telemetry.addData("ErrorCode: ", errorCode);
                telemetry.update();
                }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("hi","hi");
        }
    }

    class ColorPipeline extends OpenCvPipeline {

        private Point SIDE_BOTTOM_LEFT = new Point(0, 211.2);
        private Point SIDE_TOP_RIGHT = new Point(114.56,249.6);
        private Point REGION1_BOTTOM_LEFT = new Point(205.28,211.2);
        private Point REGION1_TOP_RIGHT = new Point(320,249.6);
        private Point REGION2_BOTTOM_LEFT = new Point(588.8, 211.2);
        private Point REGION2_TOP_RIGHT = new Point(640,249.6);

        final Scalar RED = new Scalar(255, 0, 0);
        final Scalar GREEN = new Scalar(0, 255, 0);

        Mat Side, Region1, Region2;
        Mat Color = new Mat();
        Mat YCrCb = new Mat();
        int avgSide, avg1, avg2;

        //convert mat into color depending on scale
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Color, 2);
        }


        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            Side = Color.submat(new Rect(SIDE_BOTTOM_LEFT,SIDE_TOP_RIGHT));
            Region1 = Color.submat(new Rect(REGION1_BOTTOM_LEFT,REGION1_TOP_RIGHT));
            Region2 = Color.submat(new Rect(REGION2_BOTTOM_LEFT,REGION2_TOP_RIGHT));

        }

        @Override
        public Mat processFrame(Mat input) {

            inputToCb(input);

            avgSide = (int) Core.mean(Side).val[0];
            avg1 = (int) Core.mean(Region1).val[0];
            avg2 = (int) Core.mean(Region2).val[0];

            //find max color value for both side + barcodes
            //test color space for biggest contrast between
            //grey + Red/blue (side)
            //blue/red + shipping element color (barcode)
            //need a range for the same color incase element is on third barcode (outside of cam view)

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    SIDE_BOTTOM_LEFT, // First point which defines the rectangle
                    SIDE_TOP_RIGHT, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    2 // Thickness of the rectangle lines
            );


            Imgproc.rectangle(
                    input,
                    REGION1_BOTTOM_LEFT,
                    REGION1_TOP_RIGHT,
                    RED,
                    2
            );


            Imgproc.rectangle(
                    input,
                    REGION2_BOTTOM_LEFT,
                    REGION2_TOP_RIGHT,
                    RED,
                    2
            );

            return input;
        }
    }

}