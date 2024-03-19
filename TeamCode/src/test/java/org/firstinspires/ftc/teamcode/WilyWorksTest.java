package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.wilyworks.TestWorks;
import org.junit.Test;

public class WilyWorksTest {

    @Test
    public void beWily() {
        TestWorks works = new TestWorks();
        works.runOpMode("org.firstinspires.ftc.teamcode.explorations.DistanceTest");
    }
}
