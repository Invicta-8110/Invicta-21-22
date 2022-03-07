package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RegionalsPIDTesting extends LinearOpMode {

   @Override
   public void runOpMode() {
      FreightFrenzyHardware robot = new FreightFrenzyHardware();
      robot.init(hardwareMap);
      DcMotor left = robot.left;
      DcMotor right = robot.right;

      PIDController pid = new PIDController(0.0005,0,0);
      pid.reset();

      waitForStart();

      while (opModeIsActive()) {

         while(opModeIsActive() && !pid.atSetPoint()) {
            double velocity = pid.calculate(right.getCurrentPosition());
            right.setPower(velocity);
            left.setPower(-velocity);
            telemetry.addData("Steady-state error:", pid.getPositionError());
            telemetry.addData("Speed: ",velocity);
            telemetry.update();

            drive(1200, 5);

            right.setTargetPosition(1200);
            left.setTargetPosition(1200);

         }
         right.setPower(0);
         left.setPower(0);

      }
   }

   public void drive(double distanceCM, double tolerance) {




   }


}
