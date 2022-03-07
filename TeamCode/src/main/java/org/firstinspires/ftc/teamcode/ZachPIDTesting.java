package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Zach Tests PID",group="tests")
public class ZachPIDTesting extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();

    // Constants to find the amount of encoder ticks per CM
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_CM = 9.6;

    // Finds the amount of encoder ticks per CM
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    DcMotor[] wheels = new DcMotor[2];

    @Override
    public void runOpMode() {

        wheels[0] = hardwareMap.dcMotor.get("left");
        wheels[1] = hardwareMap.dcMotor.get("right");

        wheels[0].setDirection(DcMotor.Direction.FORWARD);
        wheels[1].setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor wheel : wheels){
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("Path0",  "Starting at %7d :%7d", wheels[0].getCurrentPosition(),wheels[1].getCurrentPosition());

        telemetry.update();

        waitForStart();

        PIDDrive(120, 1);
        sleep(100);

        //TODO: Work on the turning
        //PIDTurn(100, 1);
        //sleep(100);

        //PIDStrafe(120, 1);
        //sleep(100);

    }

    public void PIDDrive(double distanceCM, double tolerance) { // TODO: Adjust Tolerance

        int []newWheelTarget = new int[2];

        // Ensure that the opmode is still active
        while (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            for (int i = 0; i < 2; i++) {
                newWheelTarget[i] = wheels[i].getCurrentPosition() + (int)(distanceCM * COUNTS_PER_CM); // TODO: Get avg position
            }

            double[] P = new double[2];

            double kp = 1;
            //TODO: do kd and ki
            //double kd = 0.000075; // Constant of derivation
            //double ki = 0.000006;

            double dt = 20; //Delta time = 20 ms/cycle
            double dtS = dt/1000;

            double[] error = new double[2];
            double[] previousError = new double[2];
            double[] power = new double[2];

            double[] area = new double[2];
            double[] previousArea = new double[2];


            error[0] = tolerance + 1;

            while ((Math.abs(error[0]) > tolerance)) { // TODO: replace error[0] with avgError

                telemetry.addData("Path1",  "Running to %7d :%7d", newWheelTarget[0], newWheelTarget[1]);
                telemetry.addData("Path2",  "Running at %7d :%7d", wheels[0].getCurrentPosition(), wheels[1].getCurrentPosition());

                telemetry.addData("DistanceCM: ", (int)(distanceCM * COUNTS_PER_CM));

                telemetry.addData("power: ", power[0]);

                telemetry.addData("Proportion:", P[0]);
                //telemetry.addData("Derivative:", kd * ((error[0] - previousError[0]) / dtS));
                //telemetry.addData("Integral:", ki * area[0]);

                telemetry.addData("de(t)/dt", ((error[0] - previousError[0]) / dtS));

                telemetry.addData("error:", error[0]);
                telemetry.addData("previous error:", previousError[0]);

                //telemetry.addData("∫e(t)dt:", area[0]);
                //telemetry.addData("previous ∫e(t)dt:", previousArea[0]);

                telemetry.addData("dtS", dtS);

                telemetry.update();

                previousError[0] = error[0];
                previousError[1] = error[1];

                error[0] = (int)(distanceCM * COUNTS_PER_CM) - wheels[0].getCurrentPosition();
                error[1] = (int)(distanceCM * COUNTS_PER_CM) - wheels[1].getCurrentPosition();

                P[0] = Math.abs(error[0])/(int)(distanceCM * COUNTS_PER_CM);
                P[1] = Math.abs(error[1])/(int)(distanceCM * COUNTS_PER_CM);

                previousArea[0] = area[0];
                previousArea[1] = area[1];

                area[0] = error[0] * dtS + previousArea[0];
                area[1] = error[1] * dtS + previousArea[1];


                power[0] = kp * P[0] * ((error[0] - previousError[0]) / dtS) + (area[0]);
                power[1] = kp * P[1] * ((error[1] - previousError[1]) / dtS) + (area[1]);


                wheels[0].setPower(power[0]);
                wheels[1].setPower(power[1]);


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
    }

    //TODO: Figure out what this is
    /*PROJECT FOR LATER
    public void PIDTurn(double distanceCM, double tolerance) {

        int []newWheelTarget = new int[4];

        // Ensure that the opmode is still active
        if (opModeIsActive()) { //if statewment, change

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
                wheels[2].setPower(-avgPower);
                wheels[3].setPower(avgPower);

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
