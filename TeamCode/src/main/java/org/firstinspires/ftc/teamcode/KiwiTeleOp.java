package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Kiwi Tele-Op", group = "Freight Frenzy")
public class KiwiTeleOp extends LinearOpMode {

    KiwiHardware robot = new KiwiHardware();

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor top = hardwareMap.dcMotor.get("top");
        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");

        top.setDirection(DcMotor.Direction.FORWARD);
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while(opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;//cl
            double turn = -gamepad1.right_stick_x;//cl
            double strafe = gamepad1.left_stick_x;

            double speed = scaleInput(drive);

            // negated turn + added strafe

            double topPower = turn + strafe;
            double leftPower = turn - drive + strafe;
            double rightPower = turn + drive - strafe;

            if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
                top.setPower(0);
                left.setPower(0);
                right.setPower(0);
            } else {
                top.setPower(topPower);
                left.setPower(leftPower);
                right.setPower(rightPower);
            }

            telemetry.addData("Top Power", "top (%.2f)", top.getPower());
            telemetry.addData("Right Power", "right (%.2f)", right.getPower());
            telemetry.addData("Left Power", "left (%.2f)", left.getPower());
            telemetry.update();

        }

    }

    private double scaleInput(double dval){

        double[] scaleArray = {0.0, 0.001, 0.005, 0.01, 0.05, 0.1,
                0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, .7};

        //get the corresponding index for the scaleInput array
        int index = (int) (dval * 16.0);

        //index should be positive
        index = Math.abs(index);

        if (index > 16) {
            index = 16;
        }

        //get value from the array
        double dScale;
        if(dval < 0){
            dScale = -scaleArray[index];
        } else{
            dScale = scaleArray[index];
        }

        //return scaled value
        return dScale;
    }
}
