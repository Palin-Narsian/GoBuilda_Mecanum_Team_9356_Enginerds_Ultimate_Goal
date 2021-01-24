package org.firstinspires.ftc.teamcode.Mecanum.Teleop.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Mecanum.Teleop.HardwarePushbot;

@Autonomous(name="Mecanum Calibrate", group="chad")
public class Calibrate extends LinearOpMode {
    //

    HardwarePushbotAutonomous robot = new HardwarePushbotAutonomous();   // Use a Pushbot's hardware

    public void runOpMode() {
        //
        robot.init(hardwareMap);

        //
        waitForEnginerdsToGoOff();
        //
        moveToPosition(20, .2);//Don't change this line, unless you want to calibrate with different speeds
        //
    }
    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    public void moveToPosition(double inches, double speed) {
        //
        if (inches < 5) {
            int move = (int) (Math.round(inches * robot.conversion));
            //
            robot.lf.setTargetPosition(robot.lf.getCurrentPosition() + move);
            robot.rf.setTargetPosition(robot.rf.getCurrentPosition() + move);
            robot.lb.setTargetPosition(robot.lb.getCurrentPosition() + move);
            robot.rb.setTargetPosition(robot.rb.getCurrentPosition() + move);
            //
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            robot.lf.setPower(speed);
            robot.rf.setPower(speed);
            robot.lb.setPower(speed);
            robot.rb.setPower(speed);
            //
            while (robot.lf.isBusy() && robot.rf.isBusy() && robot.lb.isBusy() && robot.rb.isBusy()) {
            }
            robot.lf.setPower(0);
            robot.rf.setPower(0);
            robot.lb.setPower(0);
            robot.rb.setPower(0);
        } else {
            int move1 = (int) (Math.round((inches - 5) * robot.conversion));
            int movefl2 = robot.lf.getCurrentPosition() + (int) (Math.round(inches * robot.conversion));
            int movefr2 = robot.rf.getCurrentPosition() + (int) (Math.round(inches * robot.conversion));
            int movebl2 = robot.lb.getCurrentPosition() + (int) (Math.round(inches * robot.conversion));
            int movebr2 = robot.rb.getCurrentPosition() + (int) (Math.round(inches * robot.conversion));
            //
            robot.lf.setTargetPosition(robot.lf.getCurrentPosition() + move1);
            robot.rf.setTargetPosition(robot.rf.getCurrentPosition() + move1);
            robot.lb.setTargetPosition(robot.lb.getCurrentPosition() + move1);
            robot.rb.setTargetPosition(robot.rb.getCurrentPosition() + move1);
            //
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            robot.lf.setPower(speed);
            robot.rf.setPower(speed);
            robot.lb.setPower(speed);
            robot.rb.setPower(speed);
            //
            while (robot.lf.isBusy() && robot.rf.isBusy() && robot.lb.isBusy() && robot.rb.isBusy()) {
            }
            //
            robot.lf.setTargetPosition(movefl2);
            robot.rf.setTargetPosition(movefr2);
            robot.lb.setTargetPosition(movebl2);
            robot.rb.setTargetPosition(movebr2);
            //
            robot.lf.setPower(.1);
            robot.rf.setPower(.1);
            robot.lb.setPower(.1);
            robot.rb.setPower(.1);
            //
            while (robot.lf.isBusy() && robot.rf.isBusy() && robot.lb.isBusy() && robot.rb.isBusy()) {
            }
            robot.lf.setPower(0);
            robot.rf.setPower(0);
            robot.lb.setPower(0);
            robot.rb.setPower(0);
        }
        return;
    }
    /*
    A tradition within the Thunder Pengwins code, we always start programs with waitForStartify,
    our way of adding personality to our programs.
     */
    public void waitForEnginerdsToGoOff() {
        waitForStart();
    }
}