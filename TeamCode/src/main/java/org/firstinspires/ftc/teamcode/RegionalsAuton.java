package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="RegionalAuton",group="Freight Frenzy")
public class RegionalsAuton extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime clawTime = new ElapsedTime();

    // Constants to find the amount of encoder ticks per CM
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_CM = 11.5;

    Orientation lastAngles = new Orientation();
    public double globalAngle;

    // Finds the amount of encoder ticks per CM
    //static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);
    static final double COUNTS_PER_CM = 1.22*(537.6/(11.5* Math.PI));

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

        claw.setPosition(0); //close claw

        waitForStart();

        barcodeWithElement = pipeline.getBarcode();
        placePreload(barcodeWithElement);

//        PIDDrive(20,1);
//        sleep(1000);

   //     IMU_Turn_PID_Total(90,1);
   //     sleep(1000);

        //PIDDrive(50,1);

//        while (opModeIsActive()) {
//
//            barcodeWithElement = pipeline.getBarcode();
//
//            PIDController pidDrive = new PIDController(1, 2, 3);
//            pidDrive.setTolerance();
//            pidDrive.setSetpoint();
//            while (!pidDrive.onTarget()) {
//                double output = pidDrive.calculate(robot.left.getCurrentPosition());
//                robot.left.setPower(pidDrive.calculate(robot.left.getCurrentPosition()));
//            }
//            /*
//            PIDDrive(18.5, 1);
//            sleep(500);
//            PIDTurn(-37.5, 5);
//            sleep(1000);
//            PIDDrive(-78.0, 5);
//            sleep(500);
//
//            runtime.reset();
//            while (runtime.seconds() < 3.25) {
//                robot.carousel.setPower(0.6);
//            }
//            robot.carousel.setPower(0);
//
//            PIDDrive(77.0, 5);
//            PIDTurn(23.5, 5);
//
//            placePreload(barcodeWithElement);
//
//            PIDTurn(-25.5,5);
//            PIDDrive(200.5,5);
//            break;
//            */
//        }

    }

    public double calculateTotalAngle(){
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = angles.firstAngle - robot.straight.firstAngle;

        return -globalAngle;
    }

    public void PIDDrive(double distanceCM, double tolerance) { // TODO: Adjust Tolerance

        wheels[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheels[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheels[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheels[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            telemetry.addData("inpid", "inside first while");
            telemetry.update();
            double[] p = new double[2];

            double kp = 1;
            //TODO: do kd and ki
            double kd = 0.000045;
            double ki = 0.00001;
            //double kd = 0.000075; // Constant of derivation
            //double ki = 0.000006;

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
//            previousError[1] = 0;

            while ((Math.abs(error[0]) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError

                error[0] = (int) (distanceCM * COUNTS_PER_CM) - wheels[0].getCurrentPosition();
                //error[1] = (int) (distanceCM * COUNTS_PER_CM) - wheels[1].getCurrentPosition();

                p[0] = kp * Math.abs(error[0]) / (int) (distanceCM * COUNTS_PER_CM);
                //p[1] = kp * error[1] / (int) (distanceCM * COUNTS_PER_CM);

                //double time = runtime.time();
//                double time = 0.02; //seconds
                //telemetry.addData("Time: ",time);
                //telemetry.update();

                integral[0] += ki * (error[0] * runtime.seconds());
                //integral[1] += ki * (error[1] * time);

                derivative[0] = kd * ((error[0] - previousError[0]) / runtime.seconds());
                //derivative[1] = kd * ((error[1] - previousError[1]) / runtime.seconds());
                //may need to replace runtime.seconds() with time variable

                power[0] = p[0] + derivative[0] + integral[0];
                //power[1] = p[1] + derivative[1] + integral[1];
                wheels[0].setPower(power[0]);
                wheels[1].setPower(power[0]);


                previousError[0] = error[0];
                //previousError[1] = error[1];

                telemetry.addData("error:", error[0]);
                telemetry.addData("previous error:", previousError[0]);

                telemetry.addData("Target Angle: ", distanceCM);

                telemetry.addData("Power", power[0]);

                telemetry.addData("de(t)/dt: ", ((error[0] - previousError[0])/runtime.seconds()));

                telemetry.addData("Proportion: ", p[0]);

                telemetry.addData("Error: ", error);

                telemetry.addData("Derivative: ", derivative[0]);
                telemetry.addData("Integral: ", integral[0]);


                telemetry.addData("Previous Error: ", previousError[0]);


                //telemetry.addData("∫e(t)dt:", area[0]);
                //telemetry.addData("previous ∫e(t)dt:", previousArea[0]);

                //telemetry.addData("dtS", dtS);

                telemetry.update();

                runtime.reset();
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

    public void IMU_Turn_PID_Total(double degree, double tolerance) {

        double totalDistance = degree - calculateTotalAngle();

        double kp = 1;
        double kd = 0.0009;
        double ki = 0.00075;

        double power = 0;

        double P = 0;

        double previousError = 0;
        double error = tolerance + 1;

        double area = 0;
        double previousArea = 0;

        double dt = 20;
        double dts = dt/1000;

        while (opModeIsActive() && (Math.abs(error) > tolerance)) {

            telemetry.addData("Target Angle: ", degree);
            telemetry.addData("Current Angle: ", calculateTotalAngle());

            telemetry.addData("Total Distance", totalDistance);

            telemetry.addData("Power", power);

            telemetry.addData("de(t)/dt: ", (error - previousError)/dts);

            telemetry.addData("Proportion: ", P);

            telemetry.addData("Error: ", error);

            telemetry.addData("Derivative: ", kd * ((error - previousError)/dts));
            telemetry.addData("Integral: ", ki * area);


            telemetry.addData("Previous Error: ", previousError);

            telemetry.addData("Area: ", area);
            telemetry.addData("Previous Area: ", previousArea);

            telemetry.addData("dtS", dts);

            telemetry.update();

            previousError = error;
            error = degree - calculateTotalAngle();

            previousArea = area;
            area = (error * dts) + previousArea;

            P = Math.abs(error)/totalDistance;

            power = kp * P + kd * ((error - previousError)/dts) + ki * area;

            wheels[0].setPower(-power);
            wheels[1].setPower(power);

            sleep((long) dt);
        }

        for (int i = 0; i < 4; i++){
            wheels[i].setPower(0);
        }
    }

//    public void PIDTurn(double distanceCM, double tolerance) {
//        wheels[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        wheels[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        wheels[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        wheels[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        int[] newWheelTarget = new int[2];
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) { //if statewment, change
//
//            // Determine new target position, and pass to motor controller
//            for (int i = 0; i < 2; i++) {
//                newWheelTarget[i] = wheels[i].getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM); // TODO: Get avg position
//            }
//
//            double kp = 1;
//            double kd = 0.000075; // Constant of derivation
//            double ki = 0;
//
//            double dt = 20; //Delta time = 20 ms/cycle
//            double dtS = dt / 1000;
//
//            double[] error = new double[2];
//
//            double avgError = 0;
//            double avgPreviousError = 0;
//
//            double avgArea = 0;
//            double previousArea = 0;
//
//            double Proportion = 0;
//
//            double avgPower = 0;
//
//            error[0] = tolerance + 1;
//
//            while ((Math.abs(error[0]) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError
//
//                telemetry.addData("Path1", "Running to %7d :%7d", newWheelTarget[0], newWheelTarget[1]);
//                telemetry.addData("Path2", "Running at %7d :%7d", wheels[0].getCurrentPosition(), -wheels[1].getCurrentPosition());
//
//                telemetry.addData("Power: ", avgPower);
//
//                telemetry.addData("Proportion:", Proportion);
//
//                telemetry.addData("Derivative:", kd * ((avgError - avgPreviousError) / runtime.seconds()));
//                telemetry.addData("Integral:", ki * avgArea);
//
//                telemetry.addData("error:", avgError);
//                telemetry.addData("previous error:", avgPreviousError);
//
//                telemetry.addData("de(t)/dt", ((avgError - avgPreviousError) / runtime.seconds()));
//
//                telemetry.addData("∫e(t)dt:", avgArea);
//                telemetry.addData("previous ∫e(t)dt:", previousArea);
//
//                telemetry.addData("dtS", dtS);
//
//                telemetry.update();
//
//                error[0] = (int) (distanceCM * COUNTS_PER_CM) - wheels[0].getCurrentPosition();
//                error[1] = (int) (-distanceCM * COUNTS_PER_CM) - wheels[1].getCurrentPosition();
//
//
//                avgPreviousError = error[0];
//
//                Proportion = Math.abs(error[0]) / (int) (distanceCM * COUNTS_PER_CM);
//
//                previousArea = avgArea;
//                avgArea = error[0] * dtS + previousArea;
//
//                avgPower = kp * Proportion + kd * ((error[0] - avgPreviousError) / runtime.seconds()) + (ki * avgArea);
//
//                wheels[0].setPower(avgPower);
//                wheels[1].setPower(-avgPower);
//
//                runtime.reset();
//                sleep((long) dt);
//            }
//
//            // Stop all motion;
//            for (int i = 0; i < 2; i++) {
//                wheels[i].setPower(0);
//
//                // Resets encoders
//                wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                wheels[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//        }
//    }
     public void placePreload(int barcode) {
         //drop freight at correct hub level using barcode
         if (barcodeWithElement == 3) {
             telemetry.addLine("level 3");
             telemetry.update();

             robot.claw.setPosition(0.38);
             sleep(1000);

             robot.arm.setPower(0.2);
             sleep(1000);

         } else if (barcodeWithElement == 2) {
             telemetry.addLine("level 2");
             telemetry.update();

             robot.claw.setPosition(0.45);
             sleep(1000);

             telemetry.addLine("second level, Claw done");
             telemetry.update();

         } else if (barcodeWithElement == 1) {
             telemetry.addLine("level 1");
             telemetry.update();

             robot.clawAngle.setPosition(0.65);
             sleep(1000);

             robot.arm.setPower(0.2);
             sleep(1000);

             robot.arm.setPower(0);
             sleep(1000);

         }
     }
}
