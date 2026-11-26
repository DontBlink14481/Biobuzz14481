package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

// Pedro's Pose can't be edited on the dashboard, so autos keep their points in these
public class DashPose {
    public double x, y, headingDeg;

    public DashPose(double x, double y, double headingDeg) {
        this.x = x;
        this.y = y;
        this.headingDeg = headingDeg;
    }

    public DashPose(double x, double y) {
        this(x, y, 0);
    }

    public Pose pose() {
        return pose(headingDeg);
    }

    public Pose pose(double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }
}
