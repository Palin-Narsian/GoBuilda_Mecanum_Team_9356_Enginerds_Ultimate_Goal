package org.firstinspires.ftc.teamcode.Mecanum.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mecanum.Teleop.HardwarePushbot;

import static java.lang.Thread.sleep;


/**
 * Created by robot on 9/17/2018.
 */
@TeleOp(name = "Mecanum Ultimate Goal")

public class Teleop extends OpMode
{

    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    boolean WobblePositionOpen;

    @Override
    public void init()  {
        robot.init(hardwareMap);
        //shooter = hardwareMap.dcMotor.get("shooter");
    }

    @Override
    public void loop() {
        double r = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.lf.setPower(v1*0.3);
        robot.rf.setPower(-v2*0.3);
        robot.lb.setPower(v3*0.3);
        robot.rb.setPower(v4*0.3);
        //wobble stuff
        telemetry.addData("Wobble Position", robot.WobbleArm.getCurrentPosition());


        if (gamepad1.dpad_down) {
            WobblePositionOpen = true;
        }
        if (gamepad1.dpad_up) {
            WobblePositionOpen = false;
        }
        if (WobblePositionOpen==true) {
            WobbleMove(0.5, robot.wobbleExtend, 100);
        }
        if (WobblePositionOpen==false) {
            WobbleMove(0.5, robot.wobbleClose, 100);
        }
        if (gamepad1.dpad_right) {
            robot.wobbleServo.setPosition(1);

        }
        if (gamepad1.dpad_left) {
            robot.wobbleServo.setPosition(0);

        }


       /* double shooterVar = 0;
        if (gamepad1.a) {
            shooterVar = 1;
        }
        if (gamepad1.b) {
            shooterVar = 0;
        }
        if (gamepad1.x) {
            shooterVar = -1;
        }
        shooter.setPower(shooterVar);*/
    }
    public void WobbleMove(double speed, double target, double timeoutS) {
        int Wobbletarget = (int) target;

        robot.WobbleArm.setTargetPosition(Wobbletarget);
        robot.WobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.WobbleArm.setPower(Math.abs(speed));
        while (
                (runtime.seconds() < timeoutS) &&
                        (robot.WobbleArm.isBusy())) {
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            robot.lf.setPower(v1);
            robot.rf.setPower(-v2);
            robot.lb.setPower(v3);
            robot.rb.setPower(v4);
        }
        robot.WobbleArm.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.WobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}