package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="QualifierAuton",group="Zach Tests")
public class QualifierAuton extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Constants to find the amount of encoder ticks per CM
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_CM = 11.5;

    // Finds the amount of encoder ticks per CM
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    DcMotor[] wheels = new DcMotor[2];
    FreightFrenzyHardware robot = new FreightFrenzyHardware();

    //OpenCV
    private int barcodeWithElement;
    WebcamName webcam;
    OpenCvWebcam camera;
    ColorPipeline pipeline;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        Servo claw = robot.claw;

        wheels[0] = robot.right;
        wheels[1] = robot.left;

        wheels[0].setDirection(DcMotor.Direction.FORWARD);
        wheels[1].setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor wheel : wheels) {
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d", wheels[0].getCurrentPosition(), wheels[1].getCurrentPosition());

        telemetry.update();

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

        //int clawPosition = 10;
        claw.setPosition(0.85); //when pressed init, claw closes around preload

        waitForStart();

        while (opModeIsActive()) {

            //claw.setPosition(clawPosition);

            barcodeWithElement = pipeline.getBarcode();

            //PIDDrive(48.5, 1);
            //PIDDrive(30,3);
            sleep(1000);
            PIDTurn(-40, 5);
            sleep(1000);
            PIDDrive(-87.3, 1);

            /*
            while (runtime.seconds() < 4) {
                robot.carousel.setPower(0.6);
            }
            robot.carousel.setPower(0);

            PIDDrive(141.9625, 1);
            PIDTurn(40, 1);
            PIDDrive(8.90875, 1);

            //drop off freight at correct level
            placePreload(barcodeWithElement);

            PIDDrive(-8.90875, 1);
            PIDTurn(-40, 1);
            PIDDrive(87.3125, 1);

             */

            //TODO: Work on the turning
            //PIDTurn(80, 5);
            //sleep(100);
        }

    }

    public void placePreload(int barcode) {
        //drop freight at correct hub level using barcode
        if (barcode == 1) {
            robot.arm.setPower(0.1);
            //swing servo in
            robot.arm.setPower(0);

            //drop preload
            robot.claw.setPosition(0);
            sleep(1000);
            robot.claw.setPosition(0.5);
            sleep(1000);

            robot.arm.setPower(0.1);
            //swing servo back out
            robot.arm.setPower(0);
        }
        
        if (barcode == 2) {
            robot.claw.setPosition(0);
            sleep(1000);
            robot.claw.setPosition(0.5);
            sleep(1000);
        }

        if (barcode == 3) {
            robot.arm.setPower(0.2);
            //mechnical stop???
            robot.claw.setPosition(0);
            sleep(1000);
            robot.claw.setPosition(0.5);
            sleep(1000);
        }
    }

    public void PIDDrive(double distanceCM, double tolerance) { // TODO: Adjust Tolerance

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            telemetry.addData("inpid", "inside first while");
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
                telemetry.addData("Path2", "Running at %7d :%7d", wheels[0].getCurrentPosition(), wheels[1].getCurrentPosition());

                telemetry.addData("DistanceCM: ", (int) (distanceCM * COUNTS_PER_CM));

                telemetry.addData("power: ", power[0]);

                telemetry.addData("Proportion:", p[0]);
                //telemetry.addData("Derivative:", kd * ((error[0] - previousError[0]) / dtS));
                //telemetry.addData("Integral:", ki * area[0]);

                //telemetry.addData("de(t)/dt", ((error[0] - previousError[0]) / dtS));

                telemetry.addData("error:", error[0]);
                telemetry.addData("previous error:", previousError[0]);


                //telemetry.addData("∫e(t)dt:", area[0]);
                //telemetry.addData("previous ∫e(t)dt:", previousArea[0]);

                //telemetry.addData("dtS", dtS);

                telemetry.update();

                //runtime.reset(); //reset timer
                //sleep((long) 5000);

                error[0] = (int) (distanceCM * COUNTS_PER_CM) - wheels[0].getCurrentPosition();
                error[1] = (int) (distanceCM * COUNTS_PER_CM) - wheels[1].getCurrentPosition();

                p[0] = kp * Math.abs(error[0]) / (int) (distanceCM * COUNTS_PER_CM);
                p[1] = kp * Math.abs(error[1]) / (int) (distanceCM * COUNTS_PER_CM);

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

                wheels[0].setPower(power[0]);
                wheels[1].setPower(power[1]);

                previousError[0] = error[0];
                previousError[1] = error[1];

                runtime.reset();
                sleep((long) 20);
            }

            // Stop all motion;
            for (int i = 0; i < 2; i++) {
                wheels[i].setPower(0);
                // Resets encoders
                wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                wheels[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }


    //TODO: Figure out what this is
    public void PIDTurn(double distanceCM, double tolerance) {

        int[] newWheelTarget = new int[2];

        // Ensure that the opmode is still active
        if (opModeIsActive()) { //if statewment, change

            // Determine new target position, and pass to motor controller
            for (int i = 0; i < 2; i++) {
                newWheelTarget[i] = wheels[i].getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM); // TODO: Get avg position
            }

            double kp = 1;
            double kd = 0.000075; // Constant of derivation
            double ki = 0.000006;

            double dt = 20; //Delta time = 20 ms/cycle
            double dtS = dt / 1000;

            double[] error = new double[2];

            double avgError = 0;
            double avgPreviousError = 0;

            double avgArea = 0;
            double previousArea = 0;

            double Proportion = 0;

            double avgPower = 0;

            error[0] = tolerance + 1;

            while ((Math.abs(error[0]) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError

                telemetry.addData("Path1", "Running to %7d :%7d", newWheelTarget[0], newWheelTarget[1]);
                telemetry.addData("Path2", "Running at %7d :%7d", wheels[0].getCurrentPosition(), -wheels[1].getCurrentPosition());

                telemetry.addData("Power: ", avgPower);

                telemetry.addData("Proportion:", Proportion);

                telemetry.addData("Derivative:", kd * ((avgError - avgPreviousError) / runtime.seconds()));
                telemetry.addData("Integral:", ki * avgArea);

                telemetry.addData("error:", avgError);
                telemetry.addData("previous error:", avgPreviousError);

                telemetry.addData("de(t)/dt", ((avgError - avgPreviousError) / runtime.seconds()));

                telemetry.addData("∫e(t)dt:", avgArea);
                telemetry.addData("previous ∫e(t)dt:", previousArea);

                telemetry.addData("dtS", dtS);

                telemetry.update();

                error[0] = (int) (distanceCM * COUNTS_PER_CM) - wheels[0].getCurrentPosition();
                error[1] = (int) (-distanceCM * COUNTS_PER_CM) - wheels[1].getCurrentPosition();


                avgPreviousError = error[0];

                Proportion = Math.abs(error[0]) / (int) (distanceCM * COUNTS_PER_CM);

                previousArea = avgArea;
                avgArea = error[0] * dtS + previousArea;

                avgPower = kp * Proportion + kd * ((error[0] - avgPreviousError) / runtime.seconds()) + (ki * avgArea);

                wheels[0].setPower(avgPower);
                wheels[1].setPower(-avgPower);

                runtime.reset();
                sleep((long) dt);
            }

            // Stop all motion;
            for (int i = 0; i < 2; i++) {
                wheels[i].setPower(0);

                // Resets encoders
                wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                wheels[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }
}
