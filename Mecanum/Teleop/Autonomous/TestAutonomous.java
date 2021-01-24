package org.firstinspires.ftc.teamcode.Mecanum.Teleop.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Mecanum.Teleop.HardwarePushbot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

@Autonomous(name="Simple Autonomouse", group="chad")
public class TestAutonomous extends LinearOpMode {
    //

    HardwarePushbotAutonomous robot = new HardwarePushbotAutonomous();   // Use a Pushbot's hardware

    public void runOpMode() {
        //
        robot.init(hardwareMap);
        robot.initGyro();

        //
     waitForEnginerdsToGoOff();
        //
//        strafeToPosition(5,0.8); //whatever makes it go the right direction remember to change
//        sleep(500);
      moveToPosition(20, .8);
      sleep(500);
      robot.WobbleMove(0.6, robot.wobbleExtend, 200);
      robot.wobbleServo.setPosition(0); // or whatever is open
        //Input code to move backwards
         robot.WobbleMove(0.6, robot.wobbleClose, 200);
        //
    }
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
    public void turnWithGyro(double degrees, double speedDirection){
        //<editor-fold desc="Initialize">
        robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -robot.angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0){//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 20){
                first = (degrees - 20) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        }else{
            //<editor-fold desc="turn left">
            if (degrees > 20){
                first = devertify(-(degrees - 20) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robot.gravity = robot.imu.getGravity();
                yaw = -robot.angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robot.gravity = robot.imu.getGravity();
                yaw = -robot.angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robot.gravity = robot.imu.getGravity();
                yaw = -robot.angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
        }else {
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robot.gravity = robot.imu.getGravity();
                yaw = -robot.angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
        }
        //</editor-fold>
        //
        robot.lf.setPower(0);
        robot.rf.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);
    }
    //
    /*
    This is our function for arcing, a special type of movement that allows for turning while moving.
    Use the angle and length to determine where the robot will end up.
     */
    //
    /*
    A tradition within the Thunder Pengwins code, we always start programs with waitForStartify,
    our way of adding personality to our programs.
     */
    public void waitForEnginerdsToGoOff(){
        waitForStart();
    }
    //
    /*
    These functions are used in the turnWithGyro function to ensure angle
    inputs are interpreted properly.
     */
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }
    public void strafeToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches * robot.cpi * robot.meccyBias));
        //
        robot.lb.setTargetPosition(robot.lb.getCurrentPosition() - move);
        robot.lf.setTargetPosition(robot.lf.getCurrentPosition() + move);
        robot.rb.setTargetPosition(robot.rb.getCurrentPosition() + move);
        robot.rf.setTargetPosition(robot.rf.getCurrentPosition() - move);
        //
        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        robot.lf.setPower(speed);
        robot.lb.setPower(speed);
        robot.rf.setPower(speed);
        robot.rb.setPower(speed);
        //
        while (robot.lf.isBusy() && robot.rf.isBusy() && robot.lb.isBusy() && robot.rb.isBusy()){}
        robot.rf.setPower(0);
        robot.lf.setPower(0);
        robot.rb.setPower(0);
        robot.lb.setPower(0);
        return;
    }
    //
    /*
    This function is called at the beginning of the program to activate
    the robot.imu Integrated Gyro.
     */

    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and begin turning.
     */
    public void turnWithEncoder(double input){
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        robot.lf.setPower(input);
        robot.rf.setPower(-input);
        robot.lb.setPower(input);
        robot.rb.setPower(-input);
    }
}
    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */

