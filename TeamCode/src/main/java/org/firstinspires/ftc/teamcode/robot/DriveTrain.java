package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


/***
 * Поля:
 * - Моторы
 *
 * - Датчики
 *
 * Методы:
 * - Общий метод для подачи мощности на все моторы
 *
 * - Работы с датчиками
 */
public class DriveTrain {
    Robot R;
    DcMotor bl, fl, fr, br;
    HardwareMap hwd;

    public DriveTrain (HardwareMap hwd, Robot R) {
        this.R = R;
        this.hwd = hwd;
        bl = hwd.get(DcMotor.class, "bl");
        fl = hwd.get(DcMotor.class, "fl");
        fr = hwd.get(DcMotor.class, "fr");
        br = hwd.get(DcMotor.class, "br");

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        // init IMU
    }


    public void GoAngle(int goal) {
        while (Math.abs(goal - R.getAngle()) > 5) {
            double e = goal - R.getAngle();
            setPower(-e, -e, e, e, 0.2);
            R.tele.addData("angle", R.getAngle());
            R.tele.update();
        }
    }



    public void setPower(double bl, double fl, double fr, double br) {
        this.bl.setPower(bl);
        this.fl.setPower(fl);
        this.fr.setPower(fr);
        this.br.setPower(br);
    }

    public void setPower(double bl, double fl, double fr, double br, double k) {
        this.bl.setPower(bl * k);
        this.fl.setPower(fl * k);
        this.fr.setPower(fr * k);
        this.br.setPower(br * k);
    }

    // t < 0 - left t > 0 - right

    public void followDirection (double x, double y, double t, boolean isSlow, boolean isSlower) {
        double sin = Math.sin(Math.toRadians(45));
        double cos = sin;

        double tX = 0;
        double tY = 0;
        double k = 1;

        double len = x * x + y * y;

        tY = (y / sin - x / cos) / 2;
        tX = (y / sin + x / cos) / 2;

        if (isSlow) {
            k = 0.6;
        }
        if (isSlower) {
            k = 0.3;
        }

        if (Math.abs(tX) >= Math.abs(tY) && tX != 0) {
            tY /= Math.abs(tX);
            tX /= Math.abs(tX);
        } else if (Math.abs(tX) <= Math.abs(tY) && tY != 0) {
            tX /= Math.abs(tY);
            tY /= Math.abs(tY);
        }
        tX *= len * k;
        tY *= len * k;

        if (len < 0.002) {
            setPower(t, t, -t, -t);
        } else {
            setPower(
                    tX + t,
                    tY + t,
                    tX - t,
                    tY - t
            );
        }


    }

}
