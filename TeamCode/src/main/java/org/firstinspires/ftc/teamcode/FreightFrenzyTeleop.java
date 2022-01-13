package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Actual Tele-Op", group = "Freight Frenzy")
public class FreightFrenzyTeleop extends LinearOpMode {

    FreightFrenzyHardware robot = new FreightFrenzyHardware();
    private static final int POSITIVE_LIMIT = 1000;
    private static final int NEGATIVE_LIMIT = -1000;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        DcMotor left = robot.left;
        DcMotor right = robot.right;
        DcMotor arm = robot.arm;
        DcMotor carousel = robot.carousel;
        CRServo claw = robot.claw;
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //arm.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //left joystick for driving

            double drive = gamepad1.left_stick_y;//cl
            double turn = gamepad1.left_stick_x;//cl
            //double lift = -gamepad1.right_stick_y;
            //double grab = gamepad1.

            double speed = scaleInput(drive);

            double leftPower = scaleInput(drive + turn);
            double rightPower = scaleInput(drive - turn);
            //double armPower = scaleArm(lift);


            if (gamepad1.left_stick_button) { //emergency stop if joystick drifts
                left.setPower(0);
                right.setPower(0);
            } else {
                left.setPower(leftPower);
                right.setPower(rightPower);
            }

            telemetry.addData("Right Power", "right (%.2f)", right.getPower());
            telemetry.addData("Left Power", "left (%.2f)", left.getPower());
            telemetry.update();

            /*
            if (gamepad1.right_stick_button || gamepad1.right_stick_button ) {
                arm.setPower(0);
            } else if (gamepad1.right_stick > 0.1) {
                if (arm.getCurrentPosition() > POSITIVE_LIMIT) {
                    arm.setPower(0);
                    if (lift < 0)
                        arm.setPower(armPower);
                }
                else if (arm.getCurrentPosition() < NEGATIVE_LIMIT) {
                    arm.setPower(0);
                    if (lift > 0)
                        arm.setPower(armPower);
                }
                else {
                    arm.setPower(armPower);
                }
            }
            else if (gamepad1.right_stick_y < 0.1) {
                    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }

            if (gamepad1.y) {
                claw.open smth
            }
            if (gamepad1.x) {
                claw.close smth
            }
            if (gamepad1.dpad_right) {
                carousel.setPower(0.15);
                sleep(1000);
                carousel.setPower(0);
            }

             */

            //NEW STUFF
            if (gamepad1.a) {
                while (arm.getCurrentPosition() > 10) {
                    arm.setPower(0.1); //pid lifting
                }
                arm.setPower(0); //equilibrium power
            }

            if (gamepad1.x) {
                while (arm.getCurrentPosition() < 500) {
                    arm.setPower(0.1);
                }
                arm.setPower(0); //set it to equilibrium power
            }

            if (gamepad1.y) {
                while (arm.getCurrentPosition() < 1000) {
                    arm.setPower(0.1);
                }
                arm.setPower(0); //set to equil power
            }

            if (gamepad1.right_bumper) {
                claw.setPower(0.3);
            }

            if (gamepad1.left_bumper) {
                claw.setPower(0);
            }

            if (gamepad1.right_stick_button) {
                if (carousel.getPower() > 0) {
                    carousel.setPower(0);
                }
                else if (carousel.getPower() == 0) {
                    carousel.setPower(0.15);
                }
            }

            /*
            if (gamepad1.b){
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //arm.setPower(0);
            }
             */

            /*
            if (gamepad1.right_stick_y == 0)
            {
                // we are in hold so use RTP
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.2);

                while (gamepad1.right_stick_y == 0) {// Did we JUST let go of the stick?  if so, save location.
                    arm.setTargetPosition(arm.getCurrentPosition());
                }
            }

             */


            //telemetry.addData("Arm position: ", arm.getCurrentPosition());
            //telemetry.update();


      /*
      IMPORTANT TELEMETRY FOR DEBUGGING
       */
            telemetry.addData("Right Power", "right (%.2f)", right.getPower());
            telemetry.addData("Left Power", "left (%.2f)", left.getPower());
            telemetry.update();
        }
    }

    /*
    This method scales the joystick input so for low joystick values, the
    scaled value is less than linear. This is to make it easier to drive the
    robot more precisely at slower speeds
     */
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

    //scales the input from the joystick controlling the lifting/lowerin
    //of the arm, slower adjusted speeds than driving controls
    private double scaleArm(double joystick) {

        double[] scaleArray = {0.0, 0.001, 0.005, 0.01, 0.05, 0.1,
                0.15, 0.2, 0.25, 0.3, 0.35};

        //get the corresponding index for the scaleInput array
        int index = (int) (joystick * 10.0);

        //index should be positive
        index = Math.abs(index);

        if (index > 10) {
            index = 10;
        }

        //get value from the array
        double dScale;
        if(joystick < 0){
            dScale = -scaleArray[index];
        } else{
            dScale = scaleArray[index];
        }

        //return scaled value
        return dScale;

    }


}
