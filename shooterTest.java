package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;


/**
 * Created by robot on 9/17/2018.
 */
@TeleOp(name = "9356 Shooter? Test")

public class shooterTest extends OpMode
{

    DcMotor shooter;
    Servo test;
    double shooterVar = 0;

    //double rightclawpower;
    @Override
    public void init()  {
        shooter = hardwareMap.dcMotor.get("shooter");
        test = hardwareMap.servo.get("test");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        test.setPosition(0);


    }

    @Override
    public void loop() {

        shooter.setPower(gamepad1.left_stick_y);
        if (gamepad1.a) {
            shooterVar = 1;
        }
        if (gamepad1.b) {
            shooterVar = 0;
        }
        if (gamepad1.x) {
            shooterVar = -1;
        }
        shooter.setPower(shooterVar);

        if (gamepad1.dpad_right) {
            test.setPosition(0.5);
            try {
                sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            test.setPosition(0);
        }
        if (gamepad1.dpad_left) {
            test.setPosition(0);
        }
        telemetry.addData("power", shooter.getPower());
    }
    public void checkFlywheelRPM() throws InterruptedException {
        double initialPosition = shooter.getCurrentPosition();
        sleep(250);
        double afterPosition = shooter.getCurrentPosition();

        double totalRevs = (afterPosition-initialPosition)/7;
        double RPM = totalRevs * 4 *60;
        telemetry.addData("Shooter RPM: ", RPM);
        telemetry.update();
    }

}