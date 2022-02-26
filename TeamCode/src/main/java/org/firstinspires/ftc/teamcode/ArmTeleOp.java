package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Arm Testing", group = "tests")
public class ArmTeleOp extends LinearOpMode{

    FreightFrenzyHardware robot = new FreightFrenzyHardware();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger != 0) { //right open
                robot.claw.setPosition(0.35);
            }
            if (gamepad1.left_trigger != 0) { //left close
                //clawPosition = 0;
                robot.claw.setPosition(0.5);
            }
            if(gamepad1.right_stick_button) {
                robot.claw.setPosition(.5);
            }

            if (gamepad1.a) {
                robot.arm.setPower(0);
            }
            if(gamepad1.b) {
                robot.arm.setPower(0.2);
            }


        }




    }
}
