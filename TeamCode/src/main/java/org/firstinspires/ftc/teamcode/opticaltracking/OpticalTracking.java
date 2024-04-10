package org.firstinspires.ftc.teamcode.opticaltracking;

import com.qualcomm.robotcore.hardware.HardwareDevice;

public interface OpticalTracking extends HardwareDevice {
    // Structure for returning the motion:
    static public class Motion {
        public int x;
        public int y;
        Motion(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }

    public Motion getMotion();

    public int getQuality();

}
