package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotNW {
    DriveTrainMecanum WB = new DriveTrainMecanum();
    Elevator_GoBilda EL = new Elevator_GoBilda();
    elevatorNewThreadMiddle_Blue new_thread = new elevatorNewThreadMiddle_Blue();
    Servo servo;
    imu_sensor IMU = new imu_sensor();
    //TensorFlow camera = null;
    ElapsedTime timer = new ElapsedTime();
    DcMotor LED;
    AnalogInput sensor;
    ColorSensor color;

    DistanceSensor distanceSensor;

    public void init(HardwareMap HM, Telemetry tele, LinearOpMode lop){
        WB.init(HM);
        EL.initElevator(HM);
        IMU.init(HM);
        servo = HM.get(Servo.class, "servo");
        color = HM.get(ColorSensor.class, "color_sensor");
        distanceSensor = HM.get(DistanceSensor.class, "distance_sensor");
        LED = HM.get(DcMotor.class, "led");
        sensor = HM.get(AnalogInput.class, "intake_sensor");
        //camera = new TensorFlow(HM,  lop,  tele, "model_unquant.tflite", "labels.txt");

    }

    public void waiter(long time){
        timer.reset();
        while(timer.milliseconds() < time);
    }

    public vec2 getPos() {
        return WB.getPos(IMU.getAngle());
    }
}
