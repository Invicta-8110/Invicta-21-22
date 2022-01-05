package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.*;

class FreightFrenzyHardware {

    DcMotor right;
    DcMotor left;
    DcMotor pivot;
    DcMotor arm;
    CRServo claw;

    public void init(HardwareMap hardwareMap) {

        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        pivot = hardwareMap.get(DcMotor.class, "pivot");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(CRServo.class, "claw");


        right.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.FORWARD);
        pivot.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        claw.setDirection(CRServo.Direction.FORWARD);

        //right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right.setPower(0);
        left.setPower(0);
        arm.setPower(0);
        pivot.setPower(0);

        claw.setPower(0);

    }
//h
}
