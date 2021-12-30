// simple autonomous program that drives bot forward 2 seconds then ends.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

// below is the Annotation that registers this OpMode with the FtcRobotController app.
// @Autonomous classifies the OpMode as autonomous, name is the OpMode title and the
// optional group places the OpMode into the Exercises group.
// uncomment the @Disable annotation to remove the OpMode from the OpMode list.

@Autonomous(name="Camera", group="Freight Frenzy")



//@Disabled
public class CameraAuton extends LinearOpMode
{
    // called when init button is  pressed.

    @Override
    public void runOpMode() throws InterruptedException
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "NAME_OF_CAMERA_IN_CONFIG_FILE");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        waitForStart();

        if (opModeIsActive()) {
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    // Usually this is where you'll want to start streaming from the camera (see section 4)

                    camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                    camera.setPipeline(new ColorPipeLine());
                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    //add manual close to camera
                }
                @Override
                public void onError(int errorCode)
                {
                    /*
                     * This will be called if the camera could not be opened
                     */
                    telemetry.addData("Help: ", "hi");
                }
            });

        }

    }
}

class ColorPipeLine extends OpenCvPipeline {

    Mat screen = new Mat();

    @Override
    public Mat processFrame(Mat input)
    {
        input.submat(50,0,50,0);
        input.submat(40,30,20,30);
        input.submat(10,10,10,10);
        input.submat(1,2,2,2);
        return input;
    }
}
