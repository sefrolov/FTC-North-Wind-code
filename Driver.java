package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

public class Driver {
    private Robot R;
    public static Pcontroller FIELD_P = new Pcontroller(0.0005);
    public static double HEADING_TRESHOLD = 7;
    public static double FIELD_TRESHOLD = 10;
    public static double TIME_LIMIT = 5000;
    private static double pkoef = 0.0655, ikoef = 0.005, dkoef = 0.45, p, i = 0, d = 1;

    public Driver(Robot R){
        this.R = R;
    }

    public void moveToForRomb (double x, double y){
        ElapsedTime t = new ElapsedTime();
        double t0 = t.milliseconds();

        double[] posit = R.driveTrainRomb.getPosition();

        double errX = posit[0] - x;
        double errY = posit[1] - y;

        R.tele.addData("ErrorX", errX);
        R.tele.addData("ErrorY", errY);
        R.tele.update();

        R.driveTrainRomb.delay(2000);
        while((Math.abs(errX) > FIELD_TRESHOLD || Math.abs(errY) > FIELD_TRESHOLD))
        {
            posit = R.driveTrainRomb.getPosition();
            errX = posit[0] - x;
            errY = posit[1] - y;

            R.tele.addData("Position", posit);
            R.tele.addData("ErrorX", errX);
            R.tele.addData("ErrorY", errY);
            R.tele.update();

            double pX = FIELD_P.output(errX);
            double pY = FIELD_P.output(errY);

            R.tele.addData("pX", pX);
            R.tele.addData("pY", pY);
            R.tele.update();
            R.driveTrainRomb.delay(2000);

            //R.driveTrain.follow(-pX, -pY, 0);
            double sin = Math.sin(Math.toRadians(45));
            double cos = Math.cos(Math.toRadians(45));

            double x1 = pY * sin + pX * cos;
            double y1 = pY * cos + pX * sin;

            x1 = x1 / Math.abs(x1);
            y1 = y1 / Math.abs(y1);
            R.driveTrainRomb.setPower(x1, y1, y1, x1, 0.3);
        }

        R.driveTrainRomb.stop();
    }

    public void turn (double ang){
        ElapsedTime t = new ElapsedTime();
        double t0 = t.milliseconds();

        double begin_angle = R.heading();
        double errA = begin_angle - ang;

        R.tele.addData("Error", errA);
        R.tele.update();

        R.driveTrainRomb.delay(2000);
        while(Math.abs(errA) > FIELD_TRESHOLD && t.milliseconds() - t0 <= 5000)
        {
            begin_angle = R.heading();
            errA = begin_angle - ang;

            double pA = FIELD_P.output(errA);

            R.driveTrainRomb.follow(0, 0, -pA);

            R.tele.addData("Error", errA);
            R.tele.addData("pA", pA);
            R.tele.update();
        }
        R.driveTrainRomb.stop();
    }

    public void bang_bang(double angle){
        double begin_ang = R.heading();

        while(begin_ang != angle){
            if(begin_ang < angle){
                R.driveTrainRomb.follow(0, 0, angle - begin_ang);
                begin_ang = R.heading();
                R.tele.addData("Angle", begin_ang);
                R.tele.update();
            }else{
                R.driveTrainRomb.follow(0, 0, begin_ang + angle);
                begin_ang = R.heading();
            }
        }
        R.driveTrainRomb.stop();
    }

    public void PDcontroller(double ang) {
        ang = ang - 2;
        double posit = R.driveTrainTriangle.getPositionOfLift();
        double errPrev;
        double errA = posit - (ang * 2 + ang * 53 / 600);

        R.tele.addData("ErrorA", errA);
        R.tele.addData("Position", posit);
        R.tele.update();

        R.driveTrainTriangle.delay(2000);

        while (Math.abs(errA) > 2 || d != 0){
            posit = -R.driveTrainTriangle.getPositionOfLift();
            errPrev = errA;
            errA = posit - (ang * 2 + ang * 53 / 600);

            R.tele.addData("ErrorA", errA);
            R.tele.addData("Position", posit / 2 );
            R.tele.update();

            p = errA * pkoef;
            d = (errA - errPrev) * dkoef;


            R.driveTrainTriangle.setPowerToLift(p + d);
        }
        double d1 = d;
        R.driveTrainTriangle.setPowerToLift(2 * (p + d1));
        R.driveTrainTriangle.delay(10000);

    }

    public void PIDcontroller(double ang) {
        ang = ang - 4;
        double posit = R.driveTrainTriangle.getPositionOfLift();
        double errPrev;
        double errA = posit - (ang * 2 + ang * 53 / 600);

        R.tele.addData("ErrorA", errA);
        R.tele.addData("Position", posit);
        R.tele.update();

        R.driveTrainTriangle.delay(2000);

        while (Math.abs(errA) > 2 || d != 0){
            posit = -R.driveTrainTriangle.getPositionOfLift();
            errPrev = errA;
            errA = posit - (ang * 2 + ang * 53 / 600);

            R.tele.addData("ErrorA", errA);
            R.tele.addData("Position", posit / 2 );
            R.tele.update();

            p = errA * pkoef;
            i = (Math.abs(errA) + Math.abs(errPrev)) * ikoef;
            d = (errA - errPrev) * dkoef;


            R.driveTrainTriangle.setPowerToLift(p + i + d);
        }
        double d1 = d;
        R.driveTrainTriangle.setPowerToLift(2 * (p + i + d1));
        R.driveTrainTriangle.delay(10000);
    }
}
