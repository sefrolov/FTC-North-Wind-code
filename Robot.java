package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.DriveTrainRomb;
import org.firstinspires.ftc.teamcode.modules.DriveTrainTriangle;
import org.firstinspires.ftc.teamcode.utils.Driver;

public class Robot {
    public DriveTrainRomb driveTrainRomb;
    public DriveTrainTriangle driveTrainTriangle;
    private Gamepad gamepad1, gamepad2;
    BNO055IMU imu;
    public Driver driver;
    public Telemetry tele;
    private LinearOpMode li;



    public void initRomb(LinearOpMode li){
        this.li = li;
        this.tele = li.telemetry;
        HardwareMap hwd= li.hardwareMap;

        initHWDRomb(hwd);
        initIMU(hwd);
        tele.addData("Init complete!", null);
        tele.update();
    }

    public void initTriangle(LinearOpMode li){
        this.li = li;
        this.tele = li.telemetry;
        HardwareMap hwd= li.hardwareMap;

        initHWDTriangle(hwd);
        initIMU(hwd);
        tele.addData("Init complete!", null);
        tele.update();
    }

    public void initHWDRomb(HardwareMap hwd){
        driveTrainRomb = new DriveTrainRomb();

        driveTrainRomb.init(hwd);
    }

    public void initHWDTriangle(HardwareMap hwd){
        driveTrainTriangle = new DriveTrainTriangle();

        driveTrainTriangle.init(hwd);
    }

    public void initIMU(HardwareMap hwd){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hwd.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while(!imu.isGyroCalibrated()){}

    }

    public void addDriver(){
        driver = new Driver(this);
    }

    public void attachGamepads(Gamepad g1, Gamepad g2){
         gamepad1 = g1;
         gamepad2 = g2;
    }

    public double heading (){
        return -imu.getAngularOrientation().firstAngle;
    }

    public void teleOpLoopRomb(){
        while(li.opModeIsActive()){
            double x = gamepad1.right_stick_x;
            double y = gamepad1.right_stick_y;
            double len = Math.sqrt(x * x + y * y);
            double t = gamepad1.left_trigger - gamepad1.right_trigger;

            if(Math.abs(x) >= Math.abs(y) && x != 0){
                y /= Math.abs(x);
                x /= Math.abs(x);

            } else if (Math.abs(y) >= Math.abs(x) && y != 0) {
                x /= Math.abs(y);
                y /= Math.abs(y);
            }
            x *= len;
            y *= len;

            driveTrainRomb.follow(x, y, t);

            if(gamepad1.a){
                driveTrainRomb.follow(0,1,0);
                driveTrainRomb.delay(1000);
                driveTrainRomb.stop();
            }
            if(gamepad1.x) {
                tele.addData("Angle", heading());
                tele.update();
            }
        }
        driveTrainRomb.stop();
    }

    public void teleOpLoopTriangle(){
        while(li.opModeIsActive()){
            double x = gamepad1.right_stick_x;
            double y = gamepad1.right_stick_y;
            double len = Math.sqrt(x * x + y * y);
            double t = gamepad1.left_trigger - gamepad1.right_trigger;

            if(Math.abs(x) >= Math.abs(y) && x != 0){
                y /= Math.abs(x);
                x /= Math.abs(x);

            } else if (Math.abs(y) >= Math.abs(x) && y != 0) {
                x /= Math.abs(y);
                y /= Math.abs(y);
            }
            x *= len;
            y *= len;

            driveTrainTriangle.follow(x, y, t);

            if(gamepad1.a){
                driveTrainTriangle.setPowerToLift(-1);
                driveTrainTriangle.delay(1000);

                driveTrainTriangle.stop();
            }
            if(gamepad1.x) {
                tele.addData("Angle", heading());
                tele.update();
            }

            if(gamepad1.b) {
            }
        }
        driveTrainRomb.stop();
    }

    public class inner extends Thread{
        inner launch = new inner();
        public void run() {
            while (!isInterrupted()) {
                driveTrainTriangle.setPowerToLift(0.5);
                driveTrainTriangle.delay(1000);
                driveTrainTriangle.setPowerToLift(0);
            }
        }
    }
}
