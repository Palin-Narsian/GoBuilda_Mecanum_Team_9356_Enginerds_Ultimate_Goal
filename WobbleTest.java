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
@TeleOp(name = "Wobble Goal Test")

public class WobbleTest extends OpMode
{

    DcMotor WobbleArm;
    Servo wobbleServo;
    boolean WobblePositionOpen;
    double wobbleClose = 0;
    double wobbleExtend = 800;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init()  {
        WobbleArm = hardwareMap.get(DcMotor.class, "WobbleArm");
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        WobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wobbleServo.setPosition(0);
    }

    @Override
    public void loop() {
        telemetry.addData("Wobble Position", WobbleArm.getCurrentPosition());

       if (gamepad1.dpad_up) {
           WobblePositionOpen = true;
       }
       if (gamepad1.dpad_down) {
           WobblePositionOpen = false;
       }
       if (WobblePositionOpen==true) {
           WobbleMove(0.3, wobbleExtend, 100);
       }
       if (WobblePositionOpen==false) {
           WobbleMove(0.3, wobbleClose, 100);
       }
       if (gamepad1.dpad_right) {
           wobbleServo.setPosition(1);
       }
        if (gamepad1.dpad_left) {
            wobbleServo.setPosition(0);
        }
        WobbleArm.setPower(gamepad1.right_stick_y);

    }
    public void WobbleMove(double speed, double target, double timeoutS) {
        int Wobbletarget = (int) target;

        WobbleArm.setTargetPosition(Wobbletarget);
        WobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        WobbleArm.setPower(Math.abs(speed));
        while (
                (runtime.seconds() < timeoutS) &&
                        (WobbleArm.isBusy())) {

        }
        WobbleArm.setPower(0);

        // Turn off RUN_TO_POSITION
        WobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}