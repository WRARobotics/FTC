package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.hitechnic.HiTechnicNxtIrSeekerSensor.MAX_ANGLE;
import static com.qualcomm.hardware.hitechnic.HiTechnicNxtIrSeekerSensor.MIN_ANGLE;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "servoTest")
public class servoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SimpleServo servo = new SimpleServo(
                hardwareMap, "servo1", MIN_ANGLE, MAX_ANGLE,
                AngleUnit.DEGREES
        );
        servo.setRange(-90, 90);
        double degreeRange = servo.getAngleRange();
        servo.turnToAngle(0);
        servo.rotateByAngle(2000);
    }
}