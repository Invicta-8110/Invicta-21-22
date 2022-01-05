package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class FreightFrenzyBarcodeReader extends LinearOpMode {
    //RingDetectionPipeline pipeline;
    WebcamName webcam;
    OpenCvWebcam camera;
    ColorPipeline pipeline;
//
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
            int number = pipeline.getBarcode();

            //telemetry.addData("Region 1: ", pipeline.getAvg1());
            //telemetry.addData("Region 2: ", pipeline.getAvg2());
            //telemetry.addData("Move To: ", number);
            telemetry.addData("Region 1 H", ColorPipeline.HLS1[0]);
            telemetry.addData("Region 1 S", ColorPipeline.HLS1[1]);
            telemetry.addData("Region 1 V", ColorPipeline.HLS1[2]);

            telemetry.addData("Region 2 H", ColorPipeline.HLS2[0]);
            telemetry.addData("Region 2 S", ColorPipeline.HLS2[1]);
            telemetry.addData("Region 2 V", ColorPipeline.HLS2[2]);

            telemetry.update();
        }
    }
}

