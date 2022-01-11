package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "TempAuton", group = "robot")
public class TempAuton extends LinearOpMode {

    private FreightFrenzyHardware robot = new FreightFrenzyHardware();

    @Override
    public void runOpMode() {

        // Initialize hardware
        robot.init(hardwareMap);

        // Reset encoders
        //robot.resetEncoders();

        // Displays on phone
        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //camera --> decide barcode

            //move to carousel, turn on + off carousel motor
            ////turn right, pid backwards

            //move to hub
            ////pid forward, turn left, pid forward (not right up to hub)

            //lift arm, scoot forward
            //open claw + close claw
            //scoot back, arm back to rest (down)

            //turn right, pid forward to warehouse

        }
    }


}
