package org.firstinspires.ftc.teamcode.opmodes.localizers;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;


@Autonomous
public class RedRightAutoStartLocalizer extends LinearOpMode {
    @Override
    public void runOpMode() {
        Drivetrain.localize(new Pose(55.59, 8, 90));
        RobotOpMode.setAlliance(Alliance.RED);
    }
}
