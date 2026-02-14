package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot {
    GoBildaPinpointDriver odo;
    private void something(){
        odo.resetPosAndIMU();
        odo.getPosition();
//        odo.setOffsets(15.3, 14, DistanceUnit.MM);
//        odo.setPosition(19, 3, 0, DistaceUnit.INCH, AngleUnit.DEGREES);
    }
}
