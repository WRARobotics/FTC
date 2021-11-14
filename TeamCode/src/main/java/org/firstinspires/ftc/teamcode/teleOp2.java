package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

@TeleOp(name="TeleOp2")
public class teleOp2 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        // Initialize and add Motors
        ArrayList<Motor> motors=new ArrayList<>();
        String[] motorNames=new String[]{"lf","rf"};
        double[] motorCoefficients=new double[]{0.05,0.05,0.05,0.05};
        for (int i=0;i<2;i++) {
            Motor newMotor=new Motor(hardwareMap,motorNames[i]);
            motors.add(newMotor);
        }

        // Setup all motors
        for (int i=0;i<2;i++) {
            Motor motor=motors.get(i);
            motor.setPositionCoefficient(motorCoefficients[i]);
            double kP = motor.getPositionCoefficient();
            motor.setInverted(true);
            motor.resetEncoder();
            motor.setRunMode(Motor.RunMode.PositionControl);
        }


        GamepadEx pad1=new GamepadEx(gamepad1);
        MecanumDrive mDrive=new MecanumDrive(motors.get(0),motors.get(1),motors.get(2),motors.get(3));

        waitForStart();
        while (opModeIsActive()) {
            mDrive.driveRobotCentric(pad1.getLeftX(),pad1.getLeftY(),pad1.getRightX());
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            for (int i=0;i<2;i++){
                packet.put(motorNames[i]+"Enc",motors.get(i).getCurrentPosition());
            }
            dashboard.sendTelemetryPacket(packet);
        }
    }
}