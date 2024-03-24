package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.wilyworks.TestWorks;
import org.junit.Test;

import java.util.function.Function;

public class WilyWorksTest {

    @Test
    public void beWily() {
        TestWorks works = new TestWorks();
        works.runOpMode("org.firstinspires.ftc.teamcode.explorations.DistanceTest");
    }

    private void print(String s) {
        System.out.printf("%s: ", s);
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            System.out.printf("\\u%04X", (int) c);
        }
        System.out.printf("\n");
    }


    @Test
    public void characterEncodings() {
        print("✅"); // \u2705
        print("❌"); // \u274C
        print("☒"); // \u2612
        print("☐"); // \u2610
        print("↔️"); // \u2194\uFE0F
        print("❗"); // \u2757
    }
}
