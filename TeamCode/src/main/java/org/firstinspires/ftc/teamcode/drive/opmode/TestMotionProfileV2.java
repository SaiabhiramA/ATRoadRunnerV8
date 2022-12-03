package org.firstinspires.ftc.teamcode.drive.opmode;

//import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.hardware.motors.MotorEx;
//import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Autonomous(name = "AMotionProfileV2")

public class TestMotionProfileV2 extends LinearOpMode {

    double output;

    boolean x = true;
    int testCounter = 0;

    @Override
    public void runOpMode() throws InterruptedException {
       /* TrapezoidProfile.Constraints m_constraints =
                new TrapezoidProfile.Constraints(2000, 2000);
        ProfiledPIDController m_controller =
                new ProfiledPIDController(1.3, 0.0, 0.7, m_constraints);
        MotorEx m_motor = new MotorEx(hardwareMap, "turntable", Motor.GoBILDA.RPM_1620);
        m_motor.setInverted(true);
        m_motor.resetEncoder();
        m_controller.setGoal(new TrapezoidProfile.State(500,1000));
        m_motor.setRunMode(Motor.RunMode.PositionControl);
        waitForStart();
        if (isStopRequested()) return;
        while (!m_controller.atGoal()) {
            output = m_controller.calculate(
                    m_motor.getCurrentPosition()  // the measured value
            );
            m_motor.setVelocity(output);
            testCounter = testCounter + 1;
            telemetry.addData("TT position", m_motor.getCurrentPosition());
            telemetry.addData("X value", x);
            telemetry.addData("testCounter", testCounter);
            telemetry.update();
        }
        m_motor.stopMotor();*/
    }
}



