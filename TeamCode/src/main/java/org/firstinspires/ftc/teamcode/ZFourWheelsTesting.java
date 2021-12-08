package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Four Motors Chassis",group="Zach")
public class ZFourWheelsTesting extends LinearOpMode {
    DcMotor[] wheels = new DcMotor[4];

    @Override
    public void runOpMode() {
        wheels[0] = hardwareMap.dcMotor.get("frontRight");
        wheels[1] = hardwareMap.dcMotor.get("backRight");
        wheels[2] = hardwareMap.dcMotor.get("frontLeft");
        wheels[3] = hardwareMap.dcMotor.get("backLeft");

    }
}
