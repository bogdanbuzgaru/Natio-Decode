package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math.ShooterCalculations;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp
public class Testing extends OpMode {
    private Pose2D pose = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
    private Turret turret;
    private Shooter shooter;
    private ShooterCalculations shooterCalculations;

    public void init(){
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        pose = turret.getPose();
        shooterCalculations = new ShooterCalculations(30, 45,
                                                    0, 1,
                                                    0.12, 15
                                                        );
    }
    public void loop(){
        pose = turret.getPose();

    }
}
