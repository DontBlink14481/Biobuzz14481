package org.firstinspires.ftc.teamcode.utils.zones;

import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

public class ParkZone {

    public static PolygonZone zone;

    public static PolygonZone get() {
        if (zone == null) {
            zone = new PolygonZone(
                    new Point(0, 96),
                    new Point(11, 96),
                    new Point(11, 120),
                    new Point(0, 120)
            );
        }
        return zone;
    }
}
