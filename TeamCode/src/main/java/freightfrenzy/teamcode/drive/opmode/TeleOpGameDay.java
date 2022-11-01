package freightfrenzy.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.TouchSensor;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "TeleOpGameDay FrightFrenzy")
@Disabled
public class TeleOpGameDay extends LinearOpMode {

    private DcMotor turntable;
    private DcMotor arm1AsDcMotor;
    private TouchSensor armuptouch;
    private TouchSensor armdowntouch;
    private DcMotor carosuelmoterAsDcMotor;
    private DcMotor intakeAsDcMotor;
    private Servo clawrotateAsServo;
    private DcMotor frontLeft;
    private DcMotor rearLeft;
    private DcMotor frontRight;
    private DcMotor rearRight;
    private TouchSensor turntabletouch;
    private BNO055IMU imu;

    int TurnTableCurrentPosition;
    int ArmCurrentPosition;
    boolean AutoArm;
    boolean AutoTurnTable;
    float Yaw_Angle;
    double WheelPower;
    int ArmVelocity;
    int TurnTableVelocity2;
    boolean IsIntakeBoxSet;
    String IntakeBoxPosition;
    String RobotPosition;

    /**
     * Describe this function...
     */
    private void MoveArmManual() {
        TurnTableCurrentPosition = turntable.getCurrentPosition();
        ArmCurrentPosition = arm1AsDcMotor.getCurrentPosition();
        if (gamepad2.left_trigger > 0 && gamepad2.left_bumper != true && !armuptouch.isPressed()) {
            AutoArm = false;
            arm1AsDcMotor.setTargetPosition((int) (arm1AsDcMotor.getTargetPosition() - targetposition(gamepad2.left_trigger)));
            arm1AsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) arm1AsDcMotor).setVelocity(1000);
        } else if (gamepad2.right_trigger > 0 && gamepad2.right_bumper != true && ArmCurrentPosition < -450 && !armdowntouch.isPressed()) {
            telemetry.addData("Yes less than -450", ArmCurrentPosition);
            ArmCurrentPosition = arm1AsDcMotor.getCurrentPosition();
            if (TurnTableCurrentPosition < 600 && TurnTableCurrentPosition > 100 || TurnTableCurrentPosition < 8850 && TurnTableCurrentPosition > 8350) {
                AutoArm = false;
                arm1AsDcMotor.setTargetPosition((int) (arm1AsDcMotor.getTargetPosition() + targetposition(gamepad2.right_trigger)));
                arm1AsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) arm1AsDcMotor).setVelocity(1000);
            }
        } else if (AutoArm == false) {
            arm1AsDcMotor.setTargetPosition(arm1AsDcMotor.getCurrentPosition());
            ((DcMotorEx) arm1AsDcMotor).setVelocity(((DcMotorEx) arm1AsDcMotor).getVelocity());
        }
    }

    /**
     * Describe this function...
     */
    private void Move_Carousel() {
        if (gamepad1.left_bumper == true && gamepad1.left_stick_x != 0) {
            // Move Carousel when Left Stick X is Pressed
            carosuelmoterAsDcMotor.setPower(1 * gamepad2.left_stick_x);
        } else {
            carosuelmoterAsDcMotor.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void MoveTurnTablePosition(String turntableposition) {
        if (arm1AsDcMotor.getCurrentPosition() > -1000) {
            SetArmPosition(-1000);
        }
        if (turntableposition.equals("C") && !RobotPosition.equals("C") && arm1AsDcMotor.getCurrentPosition() <= -1000) {
            SetTurnTablePosition(4350);
            RobotPosition = "C";
        } else if (turntableposition.equals("L") && !RobotPosition.equals("L") && arm1AsDcMotor.getCurrentPosition() <= -1000) {
            SetTurnTablePosition(8500);
            RobotPosition = "L";
        } else if (turntableposition.equals("R") && !RobotPosition.equals("R") && arm1AsDcMotor.getCurrentPosition() <= -1000) {
            SetTurnTablePosition(500);
            RobotPosition = "R";
        }
    }

    /**
     * Describe this function...
     */
    private void Move_Intake_Manually() {
        if (gamepad2.right_stick_x != 0 && gamepad2.right_bumper == false) {
            if (gamepad2.right_stick_x > 0) {
                // Outtake
                intakeAsDcMotor.setPower(0.25);
            } else if (gamepad2.right_stick_x < 0) {
                if (clawrotateAsServo.getPosition() != 0.649) {
                    clawrotateAsServo.setPosition(0.649);
                }
                clawrotateAsServo.setPosition(0.649);
                intakeAsDcMotor.setPower(-0.75);
            }
        } else {
            intakeAsDcMotor.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void AutoSpinCarousel() {
        if (gamepad1.y) {
            Red_alliance_carousel();
        }
        if (gamepad1.a) {
            Blue_alliance_carousel();
        }
        carosuelmoterAsDcMotor.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void Red_alliance_carousel() {
        // Red alliance Carousel
        carosuelmoterAsDcMotor.setPower(-0.05);
        for (int count = 0; count < 7; count++) {
            carosuelmoterAsDcMotor.setPower(carosuelmoterAsDcMotor.getPower() - 0.05);
            sleep(300);
        }
    }

    /**
     * Describe this function...
     */
    private void Blue_alliance_carousel() {
        // Blue alliance Carousel
        carosuelmoterAsDcMotor.setPower(0.05);
        for (int count2 = 0; count2 < 7; count2++) {
            carosuelmoterAsDcMotor.setPower(carosuelmoterAsDcMotor.getPower() + 0.05);
            sleep(300);
        }
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        boolean AutoIntake;

        turntable = hardwareMap.get(DcMotor.class, "turntable");
        arm1AsDcMotor = hardwareMap.get(DcMotor.class, "arm1AsDcMotor");
        armuptouch = hardwareMap.get(TouchSensor.class, "armuptouch");
        armdowntouch = hardwareMap.get(TouchSensor.class, "armdowntouch");
        carosuelmoterAsDcMotor = hardwareMap.get(DcMotor.class, "carosuelmoterAsDcMotor");
        intakeAsDcMotor = hardwareMap.get(DcMotor.class, "intakeAsDcMotor");
        clawrotateAsServo = hardwareMap.get(Servo.class, "clawrotateAsServo");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        turntabletouch = hardwareMap.get(TouchSensor.class, "turntabletouch");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Put initialization blocks here.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carosuelmoterAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmVelocity = 5000;
        TurnTableVelocity2 = 5000;
        WheelPower = 0.5;
        AutoArm = false;
        AutoIntake = false;
        AutoTurnTable = false;
        IsIntakeBoxSet = false;
        InitializeIMU();
        RobotPosition = "R";
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                DrivePlatform();
                AligntoShippingHubs();
                AutoSpinCarousel();
                Move_Intake_Manually();
                MoveToIntakePosition();
                CheckNSetTurboMode();
                MoveRobotDirection();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Move_ArmShippingHub_Auto(String Position2) {
        if (Position2.equals("A")) {
            // Lower Shelf of Shipping Hub
            SetArmPosition(-996);
        } else if (Position2.equals("B")) {
            // Middle Shelf of Shipping Hub
            SetArmPosition(-1987);
        } else if (Position2.equals("Y")) {
            // Top Shelf of Shipping Hub
            SetArmPosition(-3167);
        } else if (Position2.equals("X")) {
            // Warehouse Clearance & Shared Shipping Hub
            SetArmPosition(-1300);
        }
    }

    /**
     * Describe this function...
     */
    private void MoveRobotDirection() {
        if (gamepad2.dpad_up && gamepad2.left_bumper != true) {
            MoveTurnTablePosition("C");
        } else if (gamepad2.dpad_left && gamepad2.left_bumper != true) {
            MoveTurnTablePosition("L");
        } else if (gamepad2.dpad_right && gamepad2.left_bumper != true) {
            MoveTurnTablePosition("R");
        }
    }

    /**
     * Describe this function...
     */
    private double Wheel_Power(float power) {
        return WheelPower * power;
    }

    /**
     * Describe this function...
     */
    private double Arm_Velocity(double power) {
        return ArmVelocity * power;
    }

    /**
     * Describe this function...
     */
    private void MoveTurnTableManual() {
        ArmCurrentPosition = arm1AsDcMotor.getCurrentPosition();
        TurnTableCurrentPosition = turntable.getCurrentPosition();
        if (gamepad2.left_trigger > 0 && gamepad2.left_bumper == true) {
            AutoTurnTable = false;
            turntable.setTargetPosition((int) (turntable.getTargetPosition() + targetposition(gamepad2.left_trigger)));
            turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) turntable).setVelocity(1000);
        } else if (gamepad2.right_trigger > 0 && gamepad2.right_bumper == true) {
            AutoTurnTable = false;
            turntable.setTargetPosition((int) (turntable.getTargetPosition() - targetposition(gamepad2.right_trigger)));
            turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) turntable).setVelocity(1000);
        } else if (AutoTurnTable == false) {
            turntable.setTargetPosition(turntable.getCurrentPosition());
            ((DcMotorEx) turntable).setVelocity(((DcMotorEx) turntable).getVelocity());
        }
    }

    /**
     * Describe this function...
     */
    private float targetposition(float power) {
        return 10000 * power;
    }

    /**
     * Describe this function...
     */
    private double TurnTableVelocity(double power) {
        return TurnTableVelocity2 * power;
    }

    /**
     * Describe this function...
     */
    private void DrivePlatform() {
        if (gamepad1.left_trigger > 0) {
            // Move Backward when LeftTrigger is Pressed
            frontLeft.setPower(Wheel_Power(-gamepad1.left_trigger));
            frontRight.setPower(Wheel_Power(-gamepad1.left_trigger));
            rearLeft.setPower(Wheel_Power(-gamepad1.left_trigger));
            rearRight.setPower(Wheel_Power(-gamepad1.left_trigger));
        } else if (gamepad1.left_stick_x != 0 && gamepad1.left_bumper == false) {
            // Turn Right when Left Sick is moved to the Right
            // Turn Left when Left Sick is moved to the Left
            frontLeft.setPower(Wheel_Power(gamepad1.left_stick_x));
            frontRight.setPower(Wheel_Power(-gamepad1.left_stick_x));
            rearLeft.setPower(Wheel_Power(gamepad1.left_stick_x));
            rearRight.setPower(Wheel_Power(-gamepad1.left_stick_x));
        } else if (gamepad1.right_trigger > 0) {
            // Move Forward when RightTrigger is Pressed
            frontLeft.setPower(Wheel_Power(gamepad1.right_trigger));
            frontRight.setPower(Wheel_Power(gamepad1.right_trigger));
            rearLeft.setPower(Wheel_Power(gamepad1.right_trigger));
            rearRight.setPower(Wheel_Power(gamepad1.right_trigger));
        } else if (gamepad1.right_stick_x != 0) {
            // Move Left when Right Sick is moved to the Left
            // Move Right when Right Sick is moved to the Right
            frontLeft.setPower(Wheel_Power(gamepad1.right_stick_x));
            frontRight.setPower(Wheel_Power(-gamepad1.right_stick_x));
            rearLeft.setPower(Wheel_Power(-gamepad1.right_stick_x));
            rearRight.setPower(Wheel_Power(gamepad1.right_stick_x));
        } else if (gamepad1.left_bumper == true && gamepad1.left_stick_y != 0) {
            // Move on Left Diagonal Up & Down when Left Sick Y
            frontLeft.setPower(0);
            frontRight.setPower(Wheel_Power(gamepad1.left_stick_y));
            rearLeft.setPower(Wheel_Power(gamepad1.left_stick_y));
            rearRight.setPower(0);
        } else if (gamepad1.right_bumper == true && gamepad1.right_stick_y != 0) {
            // Move on Right Diagonal Up & Down when Right Sick Y
            frontLeft.setPower(Wheel_Power(gamepad1.right_stick_y));
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(Wheel_Power(gamepad1.right_stick_y));
        } else {
            Zero_Power_Wheels();
        }
    }

    /**
     * Describe this function...
     */
    private void TuneIntakeBox() {
        if (gamepad2.right_stick_x > 0 && gamepad2.right_bumper == true) {
            // Tune Intake Box
            clawrotateAsServo.setPosition(gamepad2.right_stick_x);
        }
    }

    /**
     * Describe this function...
     */
    private void Zero_Power_Wheels() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void SetIntakeBoxPosition() {
        if (IsIntakeBoxSet != true) {
            if (IntakeBoxPosition.equals("T")) {
                clawrotateAsServo.setPosition(0.38);
                IsIntakeBoxSet = true;
                // Top shelf drop
            } else if (IntakeBoxPosition.equals("M")) {
                clawrotateAsServo.setPosition(0.649);
                IsIntakeBoxSet = true;
                // Middle shelf drop
            } else if (IntakeBoxPosition.equals("L")) {
                clawrotateAsServo.setPosition(0.89);
                IsIntakeBoxSet = true;
                // Lower Shelf Drop
            } else if (IntakeBoxPosition.equals("B")) {
                // Drop at Shared Hub
                clawrotateAsServo.setPosition(0.38);
                IsIntakeBoxSet = true;
            } else if (IntakeBoxPosition.equals("I")) {
                clawrotateAsServo.setPosition(0.649);
                IsIntakeBoxSet = true;
                // Intake position
            }
        }
    }

    /**
     * Describe this function...
     */
    private void SetArmPosition(int Position2) {
        AutoArm = true;
        arm1AsDcMotor.setTargetPosition(Position2);
        arm1AsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) arm1AsDcMotor).setVelocity(ArmVelocity);
    }

    /**
     * Describe this function...
     */
    private void ResetTurnTable() {
        if (!turntabletouch.isPressed()) {
            turntable.setTargetPosition(-5000);
            turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) turntable).setVelocity(3000);
            while (!turntabletouch.isPressed()) {
                telemetry.addData("Reset Turn Table Position", turntable.getCurrentPosition());
                telemetry.update();
            }
            turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SetTurnTablePosition(100);
        }
    }

    /**
     * Describe this function...
     */
    private void CheckNSetTurboMode() {
        if (gamepad1.b) {
            WheelPower = 1;
        } else if (gamepad1.x) {
            WheelPower = 0.5;
        }
    }

    /**
     * Describe this function...
     */
    private void SetTurnTablePosition(int Position2) {
        AutoTurnTable = true;
        turntable.setTargetPosition(Position2);
        turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) turntable).setVelocity(TurnTableVelocity2);
    }

    /**
     * Describe this function...
     */
    private void ResetEncoders() {
        if (gamepad1.dpad_up == true) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm1AsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /**
     * Describe this function...
     */
    private void MoveIntakeBoxManually() {
        if (gamepad2.left_bumper == true && gamepad2.dpad_up == true) {
            clawrotateAsServo.setPosition(0.38);
            IntakeBoxPosition = "T";
            // Top shelf drop
        } else if (gamepad2.left_bumper == true && gamepad2.dpad_right == true) {
            clawrotateAsServo.setPosition(0.649);
            IntakeBoxPosition = "M";
            // Middle shelf drop
        } else if (gamepad2.left_bumper == true && gamepad2.dpad_left == true) {
            clawrotateAsServo.setPosition(0.89);
            IntakeBoxPosition = "L";
            // Lower Shelf Drop
        } else if (gamepad2.left_bumper == true && gamepad2.dpad_down == true) {
            // Drop at Shared Hub
            IntakeBoxPosition = "B";
            clawrotateAsServo.setPosition(0.38);
        }
    }

    /**
     * Describe this function...
     */
    private void ResetArmNTurnTableAuto() {
        clawrotateAsServo.setPosition(0.27);
        ResetArmUp();
        ResetTurnTable();
        ResetArmDown();
    }

    /**
     * Describe this function...
     */
    private void AligntoShippingHubs() {
        if (gamepad2.a) {
            // Lower Shelf of Shipping Hub
            Move_ArmShippingHub_Auto("A");
        } else if (gamepad2.b) {
            // Middle Shelf of Shipping Hub
            Move_ArmShippingHub_Auto("B");
        } else if (gamepad2.y) {
            // Top Shelf of Shipping Hub
            Move_ArmShippingHub_Auto("Y");
        } else if (gamepad2.x) {
            // Warehouse Clearance
            Move_ArmShippingHub_Auto("X");
        }
    }

    /**
     * Describe this function...
     */
    private void InitializeIMU() {
        BNO055IMU.Parameters imuParameters;

        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
        // Prompt user to press start buton.
        telemetry.addData("IMU Example", "Press start to continue...");
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void MoveToIntakePosition() {
        if (gamepad2.dpad_down && gamepad2.left_bumper != true && RobotPosition.equals("C")) {
            SetArmPosition(-50);
            clawrotateAsServo.setPosition(0.649);
            IntakeBoxPosition = "I";
        }
    }

    /**
     * Describe this function...
     */
    private void Display_Key_Measures() {
        telemetry.addData("Front Left Position", frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Position", frontRight.getCurrentPosition());
        telemetry.addData("Rear Left Position", rearLeft.getCurrentPosition());
        telemetry.addData("Rear Right Position", rearRight.getCurrentPosition());
        telemetry.addData("Turn Table", turntable.getCurrentPosition());
        telemetry.addData("Arm", arm1AsDcMotor.getCurrentPosition());
        Set_Yaw_Angle();
        telemetry.addData("Yaw Angle", Yaw_Angle);
        telemetry.addData("Intake Box Position", clawrotateAsServo.getPosition());
    }

    /**
     * Describe this function...
     */
    private void ResetArmUp() {
        if (!armuptouch.isPressed()) {
            arm1AsDcMotor.setTargetPosition(-5000);
            arm1AsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) arm1AsDcMotor).setVelocity(3000);
            while (!armuptouch.isPressed()) {
                telemetry.addData("Reset Arm Position", arm1AsDcMotor.getCurrentPosition());
                telemetry.update();
            }
            arm1AsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SetArmPosition(80);
        }
    }

    /**
     * Describe this function...
     */
    private void ResetArmDown() {
        if (!armdowntouch.isPressed()) {
            arm1AsDcMotor.setTargetPosition(5000);
            arm1AsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) arm1AsDcMotor).setVelocity(3000);
            while (!armdowntouch.isPressed()) {
                telemetry.addData("Reset Arm Position", arm1AsDcMotor.getCurrentPosition());
                telemetry.update();
            }
            arm1AsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SetArmPosition(0);
        }
    }

    /**
     * Describe this function...
     */
    private void Set_Yaw_Angle() {
        Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /**
     * Describe this function...
     */
    private void Reset_ArmNTurnTableManually() {
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0) {
            ResetArmNTurnTableAuto();
        }
    }
}