package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

class FreightFrenzyHardware {

    DcMotor right;
    DcMotor left;
    DcMotor arm;
    DcMotor carousel;
    Servo claw;
    //Servo levelOne;//
    Servo clawAngle;
    DcMotor extender;

    public BNO055IMU imu;

    public Orientation straight = null;

    public void init(HardwareMap hardwareMap) {

        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        arm = hardwareMap.get(DcMotor.class, "arm");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        claw = hardwareMap.get(Servo.class, "claw");
        //levelOne = hardwareMap.get(Servo.class,"levelOne");
        extender = hardwareMap.get(DcMotor.class, "extender");
        clawAngle = hardwareMap.get(Servo.class, "clawAngle");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelUnit = BNO055IMU.AccelUnit.MILLI_EARTH_GRAVITY;
            /*
            We never use the accelerometer functions of the imu, so we set the
            accel unit to milli-e/arth-gravities as a joke
            */

        imu.initialize(parameters);

        straight = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        right.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        carousel.setDirection(DcMotor.Direction.REVERSE);
        extender.setDirection(DcMotor.Direction.FORWARD);
        //claw.setDirection(CRServo.Direction.FORWARD);
        clawAngle.setDirection(Servo.Direction.REVERSE);



        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        right.setPower(0);
        left.setPower(0);
        arm.setPower(0);
        carousel.setPower(0);
        extender.setPower(0);

        //claw.setPosition(0);

    }
//h
}
