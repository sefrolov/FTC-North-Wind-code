package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "NewTele")
public class MainTele2 extends LinearOpMode {
    RobotNW Robot = new RobotNW();
    ElapsedTime sensor_sleep = new ElapsedTime(), change_sleep = new ElapsedTime();
    Servo servo;
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
        waitForStart();
        new_thread.start();
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
            if (gamepad1.a) koef = 2.5;
            else if (gamepad1.b) koef = 1.;
            else koef = 10. / 7.;

            Robot.WB.applySpeed(translational.X, translational.Y, rotate, koef);
            if(gamepad1.right_stick_y < -0.7 && gamepad1.x){
                Robot.IMU.init(hardwareMap);
            }
            if (gamepad2.y && change_sleep.milliseconds() > 500) {
                new_thread.change_per(0);
                change_sleep.reset();
            }
            if (gamepad2.b || gamepad2.right_bumper) {
                if (Robot.servo.getPosition() < 0.9) {
                    telemetry.addData("!", "!");
                    led_flag = 2;
                }
                Robot.servo.setPosition(1.);
            } else if (gamepad2.x || gamepad2.left_bumper) {
                Robot.servo.setPosition(0.8);
                sensor_sleep.reset();
            } else if (Robot.servo.getPosition() < 0.9 && Robot.sensor.getVoltage() > 1 && sensor_sleep.milliseconds() > 500) {
                Robot.servo.setPosition(1.);
                led_flag = 2;
            }
            last_sen = (Robot.sensor.getVoltage() > 1);

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

            if(gamepad2.dpad_right) {
                new_thread.setTargetAng(0);
            } else if (gamepad2.dpad_down) {
                new_thread.setTargetAng(1000);
            } else if (gamepad2.dpad_left) {
                new_thread.setTargetAng(1550);
            } else if (gamepad2.dpad_up) {
            new_thread.setTargetAng(2030);
            }
            if (abs(y_el) > 0.1){
                new_thread.addTargetAng((int)(y_el * 50));
            }
            telemetry.addData("Positive Angle", Robot.IMU.getPositiveAngle());
            telemetry.addData("Angle", Robot.IMU.getAngle());
            telemetry.addData("lb", Robot.WB.MotorLB.getCurrentPosition() - basePos[0]);
            telemetry.addData("lf", Robot.WB.MotorLF.getCurrentPosition() - basePos[1]);
            telemetry.addData("rf", Robot.WB.MotorRF.getCurrentPosition() - basePos[2]);
            telemetry.addData("rb", Robot.WB.MotorRB.getCurrentPosition() - basePos[3]);
            telemetry.addData("elevator", Robot.EL.getPos());
            //telemetry.addData("elevator1", y_el);
            /*telemetry.addData("Pos X", Robot.getPos().X);
            telemetry.addData("Pos Y", Robot.getPos().Y);*/
            telemetry.addData("led_flag", led_flag);
            //telemetry.addData("intake", Robot.sensor.getVoltage());
            telemetry.addData("servo", Robot.servo.getPosition());
            telemetry.addData("ch", new_thread.EL.changeover.getPosition());
            telemetry.addData("last_sen", last_sen);
            telemetry.addData("timer", Robot.timer.milliseconds());
            telemetry.update();
        }
        new_thread.interrupt();

            //telemetry.addData("x", Robot.WB.getPos());
            //telemetry.addData("y", Robot.WB.getPos());
            //telemetry.update();
    }
}
