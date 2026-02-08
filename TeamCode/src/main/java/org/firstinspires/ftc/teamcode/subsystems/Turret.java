package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Turret {
    private final Pose redGoal = new Pose(138, 138);
    private final Pose blueGoal = redGoal.mirror();
    private Servo turretServo1, turretServo2;
    private GoBildaPinpointDriver pinpoint;
    public Turret(HardwareMap hardwareMap){
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }
    public void init(double x, double y, double heading){
        pinpoint.recalibrateIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading));
    }
    public void update(){
        pinpoint.update();
    }
}
