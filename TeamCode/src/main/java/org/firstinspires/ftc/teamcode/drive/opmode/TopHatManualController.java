package org.firstinspires.ftc.teamcode.drive.opmode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TopHatManualController (Blocks to Java)")
public class TopHatManualController extends LinearOpMode {

    private DcMotor arm1;
    private DcMotor arm2;
    private DcMotor turntable;
    private Servo clawrotate;
    private Servo claw;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double ElbowPosition;
        int ElbowSpeed;
        double WaistPosition;
        int WaistSpeed;
        double ArmPosition;
        int ArmSpeed;
        double ClawSpeed;
        double ClawPosition;
        double WristSpeed;
        double WristPosition;

        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        turntable = hardwareMap.get(DcMotor.class, "turntable");
        clawrotate = hardwareMap.get(Servo.class, "clawrotate");
        claw = hardwareMap.get(Servo.class, "claw");

        // Set servo to mid position
        ElbowPosition = 0;
        ElbowSpeed = 50;
        WaistPosition = 0;
        WaistSpeed = 10;
        ArmPosition = 0;
        ArmSpeed = 50;
        ClawSpeed = 0.01;
        ClawPosition = 0.1;
        WristSpeed = 0.01;
        WristPosition = 0.1;
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            // Use gamepad X and B to open close servo
            if (gamepad1.dpad_up) {
                WristPosition += WristSpeed;
                telemetry.addData("DpadUp", "Pressed");
            }
            if (gamepad1.dpad_down) {
                WristPosition += -WristSpeed;
                telemetry.addData("DpadDown", "Pressed");
            }
            if (gamepad1.dpad_right) {
                ClawPosition += ClawSpeed;
                telemetry.addData("DpadRight", "Pressed");
            }
            if (gamepad1.dpad_left) {
                ClawPosition += -ClawSpeed;
                telemetry.addData("DpadLeft", "Pressed");
            }
            if (gamepad1.left_trigger > 0 && !gamepad1.left_bumper) {
                ArmPosition += -ArmSpeed;
                telemetry.addData("Left Trigger Arm Down", "Pressed");
            }
            if (gamepad1.right_trigger > 0 && !gamepad1.right_bumper) {
                ArmPosition += ArmSpeed;
                telemetry.addData("Right Trigger Arm Up", "Pressed");
            }
            if (gamepad1.left_trigger == 0 && gamepad1.left_bumper) {
                ElbowPosition += -ElbowSpeed;
                telemetry.addData("Left Bumper Elbow Down", "Pressed");
            }
            if (gamepad1.right_trigger == 0 && gamepad1.right_bumper) {
                ElbowPosition += ElbowSpeed;
                telemetry.addData("Right Bumper Elbow Up", "Pressed");
            }
            if (gamepad1.left_trigger > 0 && gamepad1.left_bumper) {
                WaistPosition += WaistSpeed;
                telemetry.addData("Waist Left", "Pressed");
            }
            if (gamepad1.right_trigger > 0 && gamepad1.right_bumper) {
                WaistPosition += -WaistSpeed;
                telemetry.addData("Waist Right", "Pressed");
            }
            // Keep Servo position in valid range
            ClawPosition = Math.min(Math.max(ClawPosition, 0), 1);
            WristPosition = Math.min(Math.max(WristPosition, 0), 1);
            clawrotate.setPosition(WristPosition);
            claw.setPosition(ClawPosition);
            arm1.setTargetPosition((int) ArmPosition);
            arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) arm1).setVelocity(5000);
            arm2.setTargetPosition((int) ElbowPosition);
            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) arm2).setVelocity(5000);
            turntable.setTargetPosition((int) WaistPosition);
            turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) turntable).setVelocity(5000);
            telemetry.addData("wrist position", clawrotate.getPosition());
            telemetry.addData("claw position", claw.getPosition());
            telemetry.addData("arm position", arm1.getCurrentPosition());
            telemetry.addData("elbow position", ElbowPosition);
            telemetry.addData("waist position", turntable.getCurrentPosition());
            telemetry.update();
            sleep(20);
        }
    }
}