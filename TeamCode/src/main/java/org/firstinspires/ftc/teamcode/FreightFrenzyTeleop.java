package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Actual Tele-Op", group = "Freight Frenzy")
public class FreightFrenzyTeleop extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();

    FreightFrenzyHardware robot = new FreightFrenzyHardware();
    private static final int POSITIVE_LIMIT = 1000;
    private static final int NEGATIVE_LIMIT = -1000;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        DcMotor left = robot.left;
        DcMotor right = robot.right;
        DcMotor arm = robot.arm;
        Servo claw = robot.claw;
        Servo clawAngle = robot.clawAngle;
        DcMotor carousel = robot.carousel;
        DcMotor extender = robot.extender;

        // arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //arm.setDirection(DcMotor.Direction.REVERSE);

        //left.setDirection(DcMotor.Direction.FORWARD);
        //right.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //claw.setPosition(0.35);

        waitForStart();

        //int clawPosition = 0;

        while (opModeIsActive()) {

            //telemetry.addData("servo position", claw.getPosition());
            //telemetry.update();

            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //left joystick for driving

            double drive = gamepad1.left_stick_y;//cl
            double turn = -gamepad1.left_stick_x;//cl
            double lift = -gamepad1.right_stick_y;

            double speed = scaleInput(drive);

            double leftPower = scaleInput(drive + turn);
            double rightPower = scaleInput(drive - turn);

//            if (gamepad1.left_stick_button || gamepad1.right_stick_button) { //emergency stop if joystick drifts
//                left.setPower(0);
//                right.setPower(0);
//            } else {
//                left.setPower(leftPower);
//                right.setPower(rightPower);
//            }

            if (gamepad1.right_trigger != 0) { //right open
                //clawPosition = 10;
                //claw.setPosition(0.35);
                claw.setPosition(0.35);
                //claw.setPosition(1);

            }
            if (gamepad1.left_trigger != 0) { //left close
                //clawPosition = 0;
                //claw.setPosition(0.85);
                claw.setPosition(0);
            }

            if (gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_up) {
                carousel.setPower(1);
                sleep(1000);
                carousel.setPower(0);
                //carousel.setPower(-0.8);
            }
            /*else if (gamepad1.dpad_left || gamepad1.dpad_up) {
                carousel.setPower(-0.8);
                sleep(1000);
                carousel.setPower(0);
            }*/

            if (gamepad1.a) {
                arm.setPower(0);
            }

            if(gamepad1.b) {
                //liftArm(3);
                arm.setPower(0.1);
            }

            if(gamepad1.right_stick_y > 0) {
                extender.setPower(0.4);
            }
            else if(gamepad1.right_stick_y < 0) {
                extender.setPower(-0.4);
            }
            else if(gamepad1.right_stick_button) {
                extender.setPower(0);
            }

            if (gamepad1.y) {   //closes
                clawAngle.setPosition(0.65); //first level
                //extender.setPower(-0.2);
            }
            if (gamepad1.x) {   //extends
                clawAngle.setPosition(0.45); //second level
                //clawAngle.setPosition(0.38); //third level
                //extender.setPower(0.2);
                //extender.setTargetPosition(944);

                /*
                while(extender.getCurrentPosition() != extender.getTargetPosition()) {
                    extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extender.setPower(0.2);
                }
                extender.setPower(0);
                */

            }

            //telemetry.addData("Arm position: ", arm.getCurrentPosition());
            //telemetry.update();


      /*
      IMPORTANT TELEMETRY FOR DEBUGGING
       */
            telemetry.addData("Right Power", "right (%.2f)", right.getPower());
            telemetry.addData("Left Power", "left (%.2f)", left.getPower());
            //telemetry.addData("claw direction: ", clawAngle.getDirection());
            telemetry.addData("clawAngle: ", robot.clawAngle.getPosition());
            //telemetry.addData("clawPosition", claw.getPosition());
            telemetry.addData("extenderPosition", extender.getCurrentPosition());
            telemetry.update();
        }
    }

    /*
    This method scales the joystick input so for low joystick values, the
    scaled value is less than linear. This is to make it easier to drive the
    robot more precisely at slower speeds
     */
    private double scaleInput(double dval){

        //
        //double[] scaleArray = {0.0, 0.001, 0.005, 0.01, 0.05, 0.1,
                //0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, .7};

        double[] scaleArray = {0.0, 0.001, 0.005, 0.01, 0.05, 0.1,
                0.15, 0.2, 0.25, 0.4, 0.45, 0.5, 0.65, 0.7, 0.75, 0.8, .9};

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

    public void liftArm(int level) {

        if (level == 3) {
            while (robot.arm.getCurrentPosition() < 76 && opModeIsActive()) {
                robot.arm.setPower(0.2);
                //robot.arm.setPower(0.1); //equilibrium power
            }
            robot.arm.setPower(0.05);
        }

        else if (level == 2) {
            robot.arm.setPower(0.15);
            robot.arm.setPower(0.1);
            robot.arm.setPower(0.05);

            //equilibrium power
        }

    }

}