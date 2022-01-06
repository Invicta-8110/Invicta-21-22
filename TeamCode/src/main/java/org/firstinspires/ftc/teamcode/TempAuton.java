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

        // Set up telemetry
        //composeTelemetry();

        // Initialize hardware
        robot.init(hardwareMap);

        // Reset encoders
        //robot.resetEncoders();

        // Displays on phone
        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();
    }


}
