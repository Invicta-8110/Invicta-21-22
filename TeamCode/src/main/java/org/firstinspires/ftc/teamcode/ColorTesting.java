package org.firstinspires.ftc.teamcode;
//program to test rev color sensor v3

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.graphics.Color;


@TeleOp(name = "ColorTesting", group = "FreightFrenzy")
public class ColorTesting extends LinearOpMode{

    ColorSensor color;
    float hsvValues[] = {0F, 0F, 0F};

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "Color");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            Color.RGBToHSV((int)(color.red() * 255),
                    (int)(color.green()*255),
                    (int)(color.blue()*255),
                    hsvValues);
            //telemetry.addData("Red", Color.red(color.argb()));
            //telemetry.addData("Green", color.green()*255);
            //telemetry.addData("Blue", color.blue()*255);
            telemetry.addData("Alpha:",color.alpha());
            telemetry.addData("Hue: ", color.argb());
            telemetry.addData("hue2:", hsvValues[0]);
            telemetry.addData("saturation: ",hsvValues[1]);
            telemetry.addData("value",hsvValues[2]);
            telemetry.addData("d", Color.argb(color.alpha(),color.red(),color.green(),color.blue()));
            telemetry.addData("f",color.argb() >> 16 & 0xFF);

            telemetry.update();
        }
    }
}
