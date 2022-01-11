package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Arm Testing", group = "Freight Frenzy")
public class ArmTeleOp extends LinearOpMode{

    FreightFrenzyHardware robot = new FreightFrenzyHardware();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            /*if (gamepad1.b) {
                robot.claw.setPower(0.1 + robot.claw.getPower());
            }
            else if (gamepad1.a) {
                robot.claw.setPower(robot.claw.getPower() - 0.1);
            }*/


            if (gamepad1.y) {
                robot.arm.setPower(0.4);
                //robot.arm.setZeroPowerBehavior(robot.arm.getZeroPowerBehavior());
            }
            else if (gamepad1.x) {
                robot.arm.setPower(-0.4);
            }
            else {
                robot.arm.setPower(0);
            }
            telemetry.addData("Arm Power: ", robot.arm.getPower());
            telemetry.update();
            telemetry.addData("ZeroPowerBehavior: ", robot.arm.getZeroPowerBehavior());
            telemetry.update();

            sleep(100);
        }




    }
}
