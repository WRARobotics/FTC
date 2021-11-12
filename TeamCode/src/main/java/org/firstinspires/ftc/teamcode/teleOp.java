package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp")
public class teleOp extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        Motor lf=new Motor(hardwareMap,"lf");
        Motor lb=new Motor(hardwareMap,"lb");
        Motor rf=new Motor(hardwareMap,"rf");
        Motor rb=new Motor(hardwareMap,"rb");
//        MotorEx lf=new MotorEx(hardwareMap,"lf");
//        MotorEx lb=new MotorEx(hardwareMap,"lb");
//        MotorEx rf=new MotorEx(hardwareMap,"rf");
//        MotorEx rb=new MotorEx(hardwareMap,"rb");
        lf.setInverted(true);
        lb.setInverted(true);
        rf.setInverted(true);
        rb.setInverted(true);
        lf.resetEncoder();
        lb.resetEncoder();
        rf.resetEncoder();
        rb.resetEncoder();

        GamepadEx pad1=new GamepadEx(gamepad1);
        MecanumDrive mDrive=new MecanumDrive(lf,rf,lb,rb);

        waitForStart();
        while (opModeIsActive()) {
            mDrive.driveRobotCentric(pad1.getLeftX(),pad1.getLeftY(),pad1.getRightX());
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("lfenc",lf.getCurrentPosition());
            packet.put("lbenc",lb.getCurrentPosition());
            packet.put("rfenc",rf.getCurrentPosition());
            packet.put("rbenc",rb.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}