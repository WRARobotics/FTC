package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

@Autonomous(name="auto")
public class auto extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        // Initialize and add Motors
        ArrayList<Motor> motors=new ArrayList<>();
        String[] motorNames=new String[]{"lf","rf","lb","rb"};
        for (int i=0;i<4;i++) {
            Motor newMotor=new Motor(hardwareMap,motorNames[i]);
            motors.add(newMotor);
        }
        MecanumDrive drive = new Mecan
        waitForStart();
        while (opModeIsActive()) {
        }
    }
}