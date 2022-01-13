package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "TempAuton", group = "robot")
public class TempAuton extends LinearOpMode {

    private FreightFrenzyHardware robot = new FreightFrenzyHardware();

    //OpenCV
    private int barcodeWithElement;
    WebcamName webcam;
    OpenCvWebcam camera;
    ColorPipeline pipeline;

    //PiD
    private ElapsedTime runtime = new ElapsedTime();
    // Constants to find the amount of encoder ticks per CM
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_CM = 11.5;
    // Finds the amount of encoder ticks per CM
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);


    @Override
    public void runOpMode() {

        // Initialize hardware
        robot.init(hardwareMap);

        // Reset encoders
        //robot.resetEncoders();

        // Displays on phone
        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        //camera setup stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);

        camera.setMillisecondsPermissionTimeout(2500);
        pipeline = new ColorPipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
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
            //camera --> decide barcode
            barcodeWithElement = pipeline.getBarcode();
/*
            //move to carousel, turn on + off carousel motor
            ////turn right, pid backwards
            PIDDrive(); // insert distance
            while (runtime.seconds() < 20) {
                robot.carousel.setPower(0.4);
            }
            robot.carousel.setPower(0);

            while (runtime.seconds() < 20) {
                robot.right.setPower(0.4);
                robot.left.setPower(-0.4);
            }
            robot.right.setPower(0);
            robot.left.setPower(0);

            PIDDrive(); // insert distance

            //move to hub
            ////pid forward, turn left, pid forward (not right up to hub)
            PIDDrive();
            PIDDrive();
            while (runtime.seconds() < 20) {
                robot.right.setPower(0.4);
                robot.left.setPower(-0.4);
            }
            robot.right.setPower(0);
            robot.left.setPower(0);

            PIDDrive();

            //lift arm, scoot forward
            //open claw + close claw
            //scoot back, arm back to rest (down)
            robot.arm.setPower();
            PIDDrive();
            robot.claw.setPower();
            robot.claw.setPower(0);
            PIDDrive();
            robot.arm.setPower();

            //turn right + lift arm
            //pid forward to warehouse
            while (runtime.seconds() < 20) {
                robot.right.setPower(0.4);
                robot.left.setPower(-0.4);
            }
            robot.right.setPower(0);
            robot.left.setPower(0);
            robot.arm.setPower();
            PIDDrive();

 */
        }
    }

    public void PIDDrive(double distanceCM, double tolerance) { // TODO: Adjust Tolerance

        //int []newWheelTarget = new int[2];

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            //for (int i = 0; i < 2; i++) {
            //    newWheelTarget[i] = wheels[i].getCurrentPosition() + (int)(distanceCM * COUNTS_PER_CM); // TODO: Get avg position
            //}

            //newWheelTarget[0] = wheels[0].getCurrentPosition() + (int)(distanceCM * COUNTS_PER_CM);
            //newWheelTarget[1] = wheels[1].getCurrentPosition() + (int)(distanceCM * COUNTS_PER_CM);
            telemetry.addData("inpid",  "inside first while");
            telemetry.update();
            double[] p = new double[2];

            double kp = 1;
            //TODO: do kd and ki
            //double kd = 0;
            //double ki = 0;
            double kd = 0.000075; // Constant of derivation
            double ki = 0.000006;

            //double dt = 20; //Delta time = 20 ms/cycle
            //double dtS = dt/1000;
            double[] derivative = new double[2];
            double[] integral = new double[2];

            double[] error = new double[2];
            double[] previousError = new double[2];
            double[] power = new double[2];

            //double[] area = new double[2];
            //double[] previousArea = new double[2];

            error[0] = tolerance + 1; //set to greater than tolerance to enter loop below
            previousError[0] = 0;
            previousError[1] = 0;

            while ((Math.abs(error[0]) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError

                // telemetry.addData("Path1",  "Running to %7d :%7d", newWheelTarget[0], newWheelTarget[1]);
                //telemetry.addData("Path2",  "Running at %7d :%7d", wheels[0].getCurrentPosition(), wheels[1].getCurrentPosition());
                //telemetry.addData("DistanceCM: ", (int)(distanceCM * COUNTS_PER_CM));
                //telemetry.addData("power: ", power[0]);
                //telemetry.addData("Proportion:", p[0]);
                //telemetry.addData("Derivative:", kd * ((error[0] - previousError[0]) / dtS));
                //telemetry.addData("Integral:", ki * area[0]);
                //telemetry.addData("de(t)/dt", ((error[0] - previousError[0]) / dtS));
                //telemetry.addData("error:", error[0]);
                //telemetry.addData("previous error:", previousError[0]);
                //telemetry.addData("∫e(t)dt:", area[0]);
                //telemetry.addData("previous ∫e(t)dt:", previousArea[0]);
                //telemetry.addData("dtS", dtS);
                //telemetry.update();

                //runtime.reset(); //reset timer
                //sleep((long) 5000);

                error[0] = (int)(distanceCM * COUNTS_PER_CM) - robot.right.getCurrentPosition();
                error[1] = (int)(distanceCM * COUNTS_PER_CM) - robot.left.getCurrentPosition();

                p[0] = kp * Math.abs(error[0])/ (int)(distanceCM * COUNTS_PER_CM);
                p[1] = kp * Math.abs(error[1])/ (int)(distanceCM * COUNTS_PER_CM);

                //double time = runtime.time();
                double time = 0.02; //seconds
                //telemetry.addData("Time: ",time);
                //telemetry.update();

                integral[0] += ki * (error[0] * time);
                integral[1] += ki * (error[1] * time);

                derivative[0] = kd * ((error[0] - previousError[0]) / runtime.seconds());
                derivative[1] = kd * ((error[1] - previousError[1]) / runtime.seconds());
                //may need to replace runtime.seconds() with time variable

                power[0] = p[0] + derivative[0] + integral[0];
                power[1] = p[1] + derivative[1] + integral[1];

                robot.right.setPower(power[0]);
                robot.left.setPower(power[1]);

                previousError[0] = error[0];
                previousError[1] = error[1];

                runtime.reset();
                sleep((long) 20);
            }

            // Stop all motion;
            robot.right.setPower(0);
            // Resets encoders
            robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.left.setPower(0);
            robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


}
