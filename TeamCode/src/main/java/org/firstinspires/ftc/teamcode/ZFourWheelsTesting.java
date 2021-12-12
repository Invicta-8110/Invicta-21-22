package org.firstinspires.ftc.teamcode;
//hi

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Four Motors",group="FreightFrenzy")

public class ZFourWheelsTesting extends LinearOpMode {

    private ElapsedTime     runtime = new ElapsedTime();

    //DcMotor[] wheels = new DcMotor[4];
    FourWheel robot = new FourWheel();

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM   = 9.6 ;     // For figuring circumference
    static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                    (WHEEL_DIAMETER_CM * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.2;

    @Override
    public void runOpMode() {
        /*
        wheels[0] = hardwareMap.dcMotor.get("frontRight");
        wheels[1] = hardwareMap.dcMotor.get("backRight");
        wheels[2] = hardwareMap.dcMotor.get("frontLeft");
        wheels[3] = hardwareMap.dcMotor.get("backLeft");
        */

        //initialize robot
        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        /*
        wheels[2].setDirection(DcMotor.Direction.REVERSE);
        wheels[3].setDirection(DcMotor.Direction.REVERSE);
         */
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        for (DcMotor wheel : wheels){
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        for (DcMotor wheel : wheels) {
            telemetry.addData("Path0","Starting at %7d",wheel.getCurrentPosition());
            telemetry.update();
        }
        */

        //message for successful encoder reset
         telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                            robot.frontLeft.getCurrentPosition(),
                            robot.backLeft.getCurrentPosition(),
                            robot.frontRight.getCurrentPosition(),
                            robot.backRight.getCurrentPosition());
         telemetry.update();
         waitForStart(); //wait for press play (start)

        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  48.0,  48.0,10.0);  // S1: Forward 48 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   30.48, -12.0*2.54, 10.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -48.0, -48, 10.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        sleep(1000);

        telemetry.addData("Path","Complete");
        telemetry.update();

    }
    public void encoderDrive(double speed,
                             //double frontLeft, double backLeft,
                             // double frontRight, double backRight
                             double leftCM, double rightCM,
                             double timeOutS) {
        //left and right CM are distance from target location?????????

        //double[] targets = new double[4];
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            /*
            targets[0] = wheels[0].getCurrentPosition() + (int) (frontRight * COUNTS_PER_CM);
            targets[1] = wheels[1].getCurrentPosition() + (int) (backRight * COUNTS_PER_CM);
            targets[2] = wheels[2].getCurrentPosition() + (int) (frontLeft * COUNTS_PER_CM);
            targets[3] = wheels[3].getCurrentPosition() + (int) (backLeft * COUNTS_PER_CM);
             */
            newLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(leftCM * COUNTS_PER_CM);
            newRightTarget = robot.frontRight.getCurrentPosition() + (int)(rightCM * COUNTS_PER_CM);
            robot.frontLeft.setTargetPosition(newLeftTarget);
            robot.backLeft.setTargetPosition(robot.backLeft.getCurrentPosition() + (int)(leftCM * COUNTS_PER_CM));
            robot.frontRight.setTargetPosition(newRightTarget);
            robot.backRight.setTargetPosition(robot.backRight.getCurrentPosition() + (int) (rightCM * COUNTS_PER_CM));

            //turn on RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //reset timeout time + start motion
            runtime.reset();
            robot.frontLeft.setPower(Math.abs(speed));
            robot.backLeft.setPower(Math.abs(speed));
            robot.frontRight.setPower(Math.abs(speed));
            robot.backRight.setPower(Math.abs(speed));

            /*
            runtime.reset();
            for (int i = 0; i < 4; i++) {
                wheels[i].setTargetPosition((int) targets[i]);
                wheels[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wheels[i].setPower(Math.abs(speed));
            }

             */

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    //(wheels[0].isBusy() && wheels[1].isBusy() && wheels[2].isBusy() && wheels[3].isBusy())
                    (runtime.seconds() < timeOutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()))
            {

                // Display it for the driver.
                /*
                telemetry.addData("Path1", "Running to %7d :%7d :%7d, :%7d", targets[0], targets[1], targets[2], targets[3])
                for (DcMotor wheel : wheels) {
                    telemetry.addData("Path2", "Starting at %7d", wheel.getCurrentPosition());
                    telemetry.update();
                }
                 */
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                                            robot.frontLeft.getCurrentPosition(),
                                            robot.backLeft.getCurrentPosition(),
                                            robot.frontRight.getCurrentPosition(),
                                            robot.backRight.getCurrentPosition());
                telemetry.update();
            }

            /*
            for (DcMotor wheel : wheels) { //stop all motion
                wheel.setPower(0);
                wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            */

            //stop all motion
            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);

            //turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
