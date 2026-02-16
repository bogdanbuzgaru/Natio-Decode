package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    private List<Double> results = new ArrayList<>();
    private GoBildaPinpointDriver pinpoint;

    public void init(){
        File file = AppUtil.getInstance().getSettingsFile("FinalPos.txt");
        String[] vals = ReadWriteFile.readFile(file).split("\n");
        for (String val : vals) {
            results.add(Double.parseDouble(val));
        }
        System.out.println(results.toString());
    }
    public void loop(){

    }
}
