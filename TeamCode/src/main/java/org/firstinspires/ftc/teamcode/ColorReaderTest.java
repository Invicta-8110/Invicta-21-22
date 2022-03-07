package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class ColorReaderTest extends LinearOpMode {
   //RingDetectionPipeline pipeline;
   WebcamName webcam;
   OpenCvWebcam camera;
   ColorDetect pipeline;

   @Override
   public void runOpMode() throws InterruptedException{
      //camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      webcam = hardwareMap.get(WebcamName.class, "webcam");
      camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);

      camera.setMillisecondsPermissionTimeout(2500);
      pipeline = new ColorDetect();
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
         ColorDetect.Value number = pipeline.getBarcode();
         telemetry.addData("threshold: ", pipeline.getThreshold());
         telemetry.addData("avg1: ", pipeline.getAvg1());
         //telemetry.addData("avg2: ", pipeline.getAvg2());

         telemetry.update();
      }
   }
}

