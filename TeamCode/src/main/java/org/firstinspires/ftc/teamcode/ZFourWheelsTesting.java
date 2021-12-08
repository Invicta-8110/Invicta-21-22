package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Four Motors Chassis",group="Zach")
public class ZFourWheelsTesting extends LinearOpMode {
    DcMotor[] wheels = new DcMotor[4];

    @Override
    public void runOpMode() {

        static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
        static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
        static final double     WHEEL_DIAMETER_CM   = 9.6 ;     // For figuring circumference
        static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_CM * 3.1415);
        static final double     DRIVE_SPEED             = 0.6;
        static final double     TURN_SPEED              = 0.5;

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


        waitForStart();

    }
}
