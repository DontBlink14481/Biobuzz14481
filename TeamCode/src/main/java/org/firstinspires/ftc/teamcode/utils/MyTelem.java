package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MyTelem {
    public static MultipleTelemetry telemetry;
    public static void init(Telemetry tm){
        telemetry = new MultipleTelemetry(tm, FtcDashboard.getInstance().getTelemetry());
    }
    public static void addData(String key, Object value){
        telemetry.addData(key, value);
    }
    public static void addLine(String key){ //just made some small changes to a bunch of things
        telemetry.addLine(key);
    }
    public static void addLine(){
        addLine("");
    }
    public static void update(){
        telemetry.update();
    }

}
