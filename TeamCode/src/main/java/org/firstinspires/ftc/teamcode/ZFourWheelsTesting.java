package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Four Motors Chassis",group="Zach")
public class ZFourWheelsTesting extends LinearOpMode {
    DcMotor[] wheels = new DcMotor[4];
    FourWheel robot = new FourWheel();

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM   = 9.6 ;     // For figuring circumference
    static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        wheels[0] = hardwareMap.dcMotor.get("frontRight");
        wheels[1] = hardwareMap.dcMotor.get("backRight");
        wheels[2] = hardwareMap.dcMotor.get("frontLeft");
        wheels[3] = hardwareMap.dcMotor.get("backLeft");

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        wheels[2].setDirection(DcMotor.Direction.REVERSE);
        wheels[3].setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor wheel : wheels){
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        for (DcMotor wheel : wheels) {
            telemetry.addData("Path0","Starting at %7d",wheel.getCurrentPosition());
            telemetry.update();
        }

        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout



        waitForStart();




    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = wheels[2].getCurrentPosition() + (int) (leftInches * COUNTS_PER_CM);
            newRightTarget = wheels[0].getCurrentPosition() + (int) (rightInches * COUNTS_PER_CM);
            wheels[0].setTargetPosition(newRightTarget);
            wheels[1].setTargetPosition(newRightTarget);
            wheels[2].setTargetPosition(newLeftTarget);
            wheels[3].setTargetPosition(newLeftTarget);

            runtime.reset();
            for (DcMotor wheel : wheels) {
                wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wheel.setPower((Math.abs(speed)));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (wheels[2].isBusy() && wheels[0].isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                for (DcMotor wheel : wheels) {
                    telemetry.addData("Path2", "Starting at %7d", wheel.getCurrentPosition());
                    telemetry.update();
                }
            }

            for (DcMotor wheel : wheels) { //stop all motion
                wheel.setPower(0);
                wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            //  sleep(250);   // optional pause after each move
        }
    }
}
