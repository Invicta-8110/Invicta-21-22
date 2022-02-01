package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Temp Auton",group="Freight Frenzy")
public class TempAuton extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Constants to find the amount of encoder ticks per CM
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_CM = 11.5;

    // Finds the amount of encoder ticks per CM
    static final double COUNTS_PER_CM =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

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

        wheels[0].setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor wheel : wheels){
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                wheels[0].getCurrentPosition(),wheels[1].getCurrentPosition());

        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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

        int clawPosition = 10;

        waitForStart();

        while (opModeIsActive()) {

            //claw.setPosition(clawPosition);

            barcodeWithElement = pipeline.getBarcode();

            PIDDrive(67, 5);

            while (runtime.seconds() < 20) {
                robot.carousel.setPower(0.6);
            }
            robot.carousel.setPower(0);

            PIDDrive(-420,5);
            break;

            /*
            //move to hub
            ////pid forward, turn left, pid forward (not right up to hub)
            PIDDrive(-125, 5); // insert distance

            PIDTurn(-40, 5);

            PIDDrive(-30.48, 10);
            //PIDDrive(0, 0);
            liftArm(barcodeWithElement);
            PIDDrive(-5,5);
            clawPosition = 0;
            claw.setPosition(clawPosition);

            PIDDrive(15, 10);
            robot.arm.setPower(0);

            // PIDDrive(25, 5);
            PIDTurn(40, 5);
            robot.arm.setPower(0.2);
            PIDDrive(-125, 5);

             */


            //TODO: Work on the turning
            //PIDTurn(80, 5);
            //sleep(100);
        }

    }

    public void PIDDrive(double distanceCM, double tolerance) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            telemetry.addData("inpid",  "inside first while");
            telemetry.update();
            double[] p = new double[2];

            double kp = 1;
            double kd = 0.000075; // Constant of derivation
            double ki = 0.000006;

            double[] derivative = new double[2];
            double[] integral = new double[2];

            double[] error = new double[2];
            double[] previousError = new double[2];
            double[] power = new double[2];

            error[0] = tolerance + 1; //set to greater than tolerance to enter loop below
            previousError[0] = 0;
            previousError[1] = 0;

            while ((Math.abs(error[0]) > tolerance) && opModeIsActive()) {

                error[0] = (int)(distanceCM * COUNTS_PER_CM) - wheels[0].getCurrentPosition();
                error[1] = (int)(distanceCM * COUNTS_PER_CM) - wheels[1].getCurrentPosition();

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

                wheels[0].setPower(power[0]);
                wheels[1].setPower(power[1]);

                previousError[0] = error[0];
                previousError[1] = error[1];

                runtime.reset();
                sleep((long) 20);
            }

            // Stop all motion;
            for (int i = 0; i < 2; i++){
                wheels[i].setPower(0);
                // Resets encoders
                wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                wheels[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }


    /*public void PIDArm(int level, double tolerance) {

        double error = 0;
        double previousError;
        double targetPosition;
        double speedError;
        double referenceSpeed;

        if (opModeIsActive()) {
            telemetry.addData("Arm: ", "entered");

            if (level == 2) {

                targetPosition = 500; //tune value
                referenceSpeed = 0.1;

                //while (Math.abs(error)) {
                   // speedError = Math.abs(referenceSpeed);

                  //  runtime.reset();
                //}
            }
            else if (level == 3) {
                targetPosition = 1000; //tune value
                //while (Math.abs(error)) {


                  //  runtime.reset();
                //}
            }
        }
    }



    public void liftArm(int level) {

        if (level == 2) {
            while (robot.arm.getCurrentPosition() < 76 && opModeIsActive()) {
                robot.arm.setPower(0.2);
                //robot.arm.setPower(0.1); //equilibrium power
            }
            robot.arm.setPower(0.05);
        }

        else if (level == 3) {
            robot.arm.setPower(0.15);
            robot.arm.setPower(0.1);
            robot.arm.setPower(0.05);

            //equilibrium power
        }
        else {
            robot.arm.setPower(0.2);
        }

    }

    //TODO: Figure out what this is
    public void PIDTurn(double distanceCM, double tolerance) {

        int []newWheelTarget = new int[2];

        // Ensure that the opmode is still active
        if (opModeIsActive()) { //if statewment, change

            // Determine new target position, and pass to motor controller
            for (int i = 0; i < 2; i++) {
                newWheelTarget[i] = wheels[i].getCurrentPosition() + (int)(distanceCM * COUNTS_PER_CM); // TODO: Get avg position
            }

            double kp = 1;
            double kd = 0.000075; // Constant of derivation
            double ki = 0.000006;

            double dt = 20; //Delta time = 20 ms/cycle
            double dtS = dt/1000;

            double[] error = new double[2];

            double avgError = 0;
            double avgPreviousError = 0;

            double avgArea = 0;
            double previousArea = 0;

            double Proportion = 0;

            double avgPower = 0;

            error[0] = tolerance + 1;

            while ((Math.abs(error[0]) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError

                telemetry.addData("Path1",  "Running to %7d :%7d", newWheelTarget[0], newWheelTarget[1]);
                telemetry.addData("Path2",  "Running at %7d :%7d", wheels[0].getCurrentPosition(), -wheels[1].getCurrentPosition());

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

                error[0] = (int)(distanceCM * COUNTS_PER_CM) - wheels[0].getCurrentPosition();
                error[1] = (int)(-distanceCM * COUNTS_PER_CM) - wheels[1].getCurrentPosition();


                avgPreviousError = error[0];

                Proportion = Math.abs(error[0])/(int)(distanceCM * COUNTS_PER_CM);

                previousArea = avgArea;
                avgArea = error[0] * dtS + previousArea;

                avgPower = kp * Proportion + kd * ((error[0] - avgPreviousError) / runtime.seconds()) + (ki * avgArea);

                wheels[0].setPower(avgPower);
                wheels[1].setPower(-avgPower);

                runtime.reset();
                sleep((long) dt);
            }

            // Stop all motion;
            for (int i = 0; i < 2; i++){
                wheels[i].setPower(0);

                // Resets encoders
                wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                wheels[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }



    /*WE PHYSICALLY CANNOT DO THIS
    public void PIDStrafe(double distanceCM, double tolerance) { // TODO: Adjust Tolerance

        int []newWheelTarget = new int[4];

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            for (int i = 0; i < 4; i++) {
                newWheelTarget[i] = wheels[0].getCurrentPosition() + (int)(distanceCM * COUNTS_PER_CM); // TODO: Get avg position
            }

            double kp = 1;
            double kd = 0.000075; // Constant of derivation
            double ki = 0.000006;

            double dt = 20; //Delta time = 20 ms/cycle
            double dtS = dt/1000;

            double[] error = new double[4];

            double avgError = 0;
            double avgPreviousError = 0;

            double avgArea = 0;
            double previousArea = 0;

            double Proportion = 0;

            double avgPower = 0;

            error[0] = tolerance + 1;

            while ((Math.abs(error[0]) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError

                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newWheelTarget[0], newWheelTarget[1], newWheelTarget[2], newWheelTarget[3]);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", wheels[0].getCurrentPosition(), -wheels[1].getCurrentPosition(), -wheels[2].getCurrentPosition(), wheels[3].getCurrentPosition());

                telemetry.addData("Power: ", avgPower);

                telemetry.addData("Proportion:", Proportion);

                telemetry.addData("Derivative:", kd * ((avgError - avgPreviousError) / dtS));
                telemetry.addData("Integral:", ki * avgArea);

                telemetry.addData("error:", avgError);
                telemetry.addData("previous error:", avgPreviousError);

                telemetry.addData("de(t)/dt", ((avgError - avgPreviousError) / dtS));

                telemetry.addData("∫e(t)dt:", avgArea);
                telemetry.addData("previous ∫e(t)dt:", previousArea);

                telemetry.addData("dtS", dtS);

                telemetry.update();


                error[0] = (int)(distanceCM * COUNTS_PER_CM) - wheels[0].getCurrentPosition();
                error[1] = (int)(-distanceCM * COUNTS_PER_CM) - wheels[1].getCurrentPosition();
                error[2] = (int)(-distanceCM * COUNTS_PER_CM) - wheels[2].getCurrentPosition();
                error[3] = (int)(distanceCM * COUNTS_PER_CM) - wheels[3].getCurrentPosition();

                avgPreviousError = error[0];

                Proportion = Math.abs(error[0])/(int)(distanceCM * COUNTS_PER_CM);

                previousArea = avgArea;
                avgArea = error[0] * dtS + previousArea;

                avgPower = kp * Proportion + kd * ((error[0] - avgPreviousError) / dtS) + (ki * avgArea);

                wheels[0].setPower(avgPower);
                wheels[1].setPower(-avgPower);
                wheels[2].setPower(avgPower);
                wheels[3].setPower(-avgPower);

                sleep((long) dt);
            }

            // Stop all motion;
            for (int i = 0; i < 4; i++){
                wheels[i].setPower(0);

                // Resets encoders
                wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                wheels[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }*/

    /*CAROUSEL SPINNER NONSENSE
    public void encoderCarouselSpinner(double speed, double spinnerMM, double timeOutS){
        int newCarouselSpinnerTarget;

        if (opModeIsActive()) { //remove if statement
            newCarouselSpinnerTarget = TIseBot.carouselSpinner.getCurrentPosition() + (int)(spinnerMM * COUNTS_PER_CM);

            TIseBot.carouselSpinner.setTargetPosition(newCarouselSpinnerTarget);

            TIseBot.carouselSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            TIseBot.carouselSpinner.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeOutS) && (TIseBot.carouselSpinner.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", newCarouselSpinnerTarget);
                telemetry.addData("Path2",  "Running at %7d", TIseBot.carouselSpinner.getCurrentPosition());
                telemetry.update();
            }
            TIseBot.carouselSpinner.setPower(0); // Resets encoders
            TIseBot.carouselSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TIseBot.carouselSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }*/
}
