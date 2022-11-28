package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "ATRobotManual")
public class ATRobotManual extends LinearOpMode {


    private DcMotor turntable;
    private DcMotor arm;
    private DcMotor elbow;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        turntable = hardwareMap.get(DcMotor.class, "turntable");
        arm = hardwareMap.get(DcMotor.class, "arm");
        elbow = hardwareMap.get(DcMotor.class, "elbow");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turntable.setTargetPosition(turntable.getCurrentPosition());
            turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) turntable).setVelocity(5000);

            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setTargetPosition(arm.getCurrentPosition());
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) arm).setVelocity(5000);

            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setTargetPosition(elbow.getCurrentPosition());
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) elbow).setVelocity(5000);

            while (opModeIsActive()) {
                if (gamepad2.left_trigger > 0) {
                    turntable.setTargetPosition((int) (turntable.getCurrentPosition() + gamepad2.left_trigger * -10));
                    telemetry.addData("Turntable Left Trigger:", turntable.getCurrentPosition());
                }
                if (gamepad2.right_trigger > 0) {
                    turntable.setTargetPosition((int) (turntable.getCurrentPosition() + gamepad2.right_trigger * 10));
                    telemetry.addData("Turntable Right Trigger:", turntable.getCurrentPosition());
                }
                if (gamepad2.left_stick_y != 0) {
                    arm.setTargetPosition((int) (arm.getCurrentPosition() + gamepad2.left_stick_y * 10));
                    telemetry.addData("Arm Left Stick:", arm.getCurrentPosition());
                }
                if (gamepad2.right_stick_y != 0) {
                    elbow.setTargetPosition((int) (elbow.getCurrentPosition() + gamepad2.right_stick_y * 10));
                    telemetry.addData("Arm Right Stick:", elbow.getCurrentPosition());
                }
                telemetry.update();
            }
        }
    }
}


