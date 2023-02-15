package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class NewTele extends LinearOpMode {
    RobotNW Robot = new RobotNW();
    ElapsedTime sensor_sleep = new ElapsedTime();
    Servo servo;
    ColorSensor colorSensor;    // Hardware Device Object
    elevatorNewThreadMiddle new_thread = new elevatorNewThreadMiddle();
    double[] basePos = {0, 0, 0, 0};
    //double elevatorEncoder;
    int led_flag = 1;
    boolean last_sen = false;
    vec2 JoyDir = new vec2(0), translational = new vec2(0);

    public void runOpMode() {
        Robot.init(hardwareMap, telemetry, this);
        telemetry.addData("", "Init complited");
        telemetry.update();
        basePos = Robot.WB.getEncoders();
        new_thread.HM = hardwareMap;
        new_thread.init();
        new_thread.start();

        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our ColorSensor object.
        //colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);

        //
        waitForStart();
        sensor_sleep.reset();

        while (opModeIsActive()) {
            double koef = 1.;
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double y_el = -gamepad2.right_stick_y;
            //double y_ch = -gamepad2.left_stick_y;
            double rotate = gamepad1.right_trigger - gamepad1.left_trigger;
            JoyDir.set(y, -x);
            translational.set(JoyDir.turn(Robot.IMU.getPositiveAngle() + 2 * PI));
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            if (gamepad1.a) koef = 2.5;
            else if (gamepad1.b) koef = 1.;
            else koef = 10. / 7.;

            Robot.WB.applySpeed(translational.X, translational.Y, rotate, koef);
            //new_thread.EL.changeover.setPosition(y_ch);
            new_thread.EL.Elevator.setTargetPosition(0);
            if (gamepad2.y) {
                new_thread.EL.Elevator.setPower(0.1);
            } else {
                new_thread.EL.Apply_Speed_for_Elevator(y_el);

                if (gamepad2.right_trigger > 0.1) {
                    if (Robot.servo.getPosition() < 0.9) {
                        telemetry.addData("!", "!");
                        led_flag = 2;
                    }
                    Robot.servo.setPosition(1.);
                } else if (gamepad2.left_trigger > 0.1) {
                    Robot.servo.setPosition(0.8);
                    sensor_sleep.reset();
                } else if (gamepad2.a) {
                    if (Robot.servo.getPosition() < 0.9) {
                        telemetry.addData("!", "!");
                        led_flag = 2;
                    }
                    Robot.servo.setPosition(1);
                    while (new_thread.EL.Elevator.getCurrentPosition() <= 500) {
                        new_thread.EL.Elevator.setPower(1);
                    }
                    new_thread.EL.Elevator.setPower(0.1);
                } else if (gamepad2.y) {
                    if (Robot.servo.getPosition() < 0.9) {
                        telemetry.addData("!", "!");
                        led_flag = 2;
                    }
                    Robot.servo.setPosition(1);
                    while (new_thread.EL.Elevator.getCurrentPosition() >= 0) {
                        new_thread.EL.Elevator.setPower(-1);
                    }
                    new_thread.EL.Elevator.setPower(0);
                } else if (gamepad2.right_trigger > 0.1) {
                    Robot.servo.setPosition(0.8 - gamepad2.right_trigger * 0.8);
                } else if (Robot.servo.getPosition() < 0.9 && Robot.sensor.getVoltage() > 1 && sensor_sleep.milliseconds() > 500) {
                    Robot.servo.setPosition(1.);
                    led_flag = 2;
                }
                last_sen = (Robot.sensor.getVoltage() > 1);

                if (abs(y_el) > 0.05) new_thread.EL.Apply_Speed_for_Elevator(y_el);
                else new_thread.EL.Apply_Speed_for_Elevator(0.1);

                //if (abs(y_ch) > 0.05) new_thread.EL.changeover.setPosition(y_ch);

                if (led_flag == 1) {
                    Robot.LED.setPower(1.);
                } else if (led_flag == 2) {
                    Robot.timer.reset();
                    led_flag = 3;
                } else if (led_flag == 3) {
                    if (Robot.timer.milliseconds() < 100) Robot.LED.setPower(0.5);
                    else if (Robot.timer.milliseconds() < 350) Robot.LED.setPower(0);
                    led_flag = 1;
                }
                /*if (gamepad2.dpad_down) {
                    //y_ch = 0.1;
                    new_thread.EL.changeover.setPosition(0.16);
                } else if (gamepad2.dpad_up) {
                    //y_ch = 0.85;
                    new_thread.EL.changeover.setPosition(0.955);
                }*/
                double change_position = new_thread.EL.changeover.getPosition();
                if (gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_down || gamepad2.dpad_up) {
                    /*if (change_position < 0.5) {
                        new_thread.EL.changeover.setPosition(0.955);
                    } else {
                        new_thread.EL.changeover.setPosition(0.16);
                    }*/
                    new_thread.change_per(0);
                }
                telemetry.addData("Lift Enc", new_thread.getPos());
                telemetry.addData("Clear", colorSensor.alpha());
                telemetry.addData("Red  ", colorSensor.red());
                telemetry.addData("Green", colorSensor.green());
                telemetry.addData("Blue ", colorSensor.blue());
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();
            }

            //telemetry.addData("x", Robot.WB.getPos());
            //telemetry.addData("y", Robot.WB.getPos());
            //telemetry.update();
        }
        new_thread.interrupt();
    }
}
