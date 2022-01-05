package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
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

import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@SuppressWarnings({"JavaDoc", "unused", "SameParameterValue", "SpellCheckingInspection"})
@Autonomous(name = "FreightFrenzyAuton", group = "robot")
public class FreightFrenzyAuton extends LinearOpMode {

    OpenCvInternalCamera phoneCam;
    //FreightFrenzyBarcodeReader.RingDetectionPipeline pipeline;

    private FreightFrenzyHardware robot = new FreightFrenzyHardware();
    private Orientation angles;
    private Acceleration gravity;
    private BNO055IMU imu;

    private static final double COUNTS_PER_MOTOR_REV = 560; // Using built in encoders
    private static final double DRIVE_GEAR_REDUCTION = 1.0; // Not geared
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // Driving/control characteristics
    private static final double DRIVE_SPEED = 0.8;
    private static final double TURN_SPEED = 0.5; // Slower for more accuracy

    private static final double HEADING_THRESHOLD = 1; // Using integer gyro
    private static final double P_TURN_COEFF = 0.1; // Larger is more responsive, but also less stable
    private static final double P_DRIVE_COEFF = 0.15; // Test other values later

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // From sample OpMode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up telemetry
        //composeTelemetry();

        // Initialize hardware
        robot.init(hardwareMap);

        // Reset encoders
        //robot.resetEncoders();

        // Displays on phone
        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        // Make sure the gyro is calibrated before starting
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();

        }


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);


        // Enable encoders
        //robot.encoders();


    }
}
