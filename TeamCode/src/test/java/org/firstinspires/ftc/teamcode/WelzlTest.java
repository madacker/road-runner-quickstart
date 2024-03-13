package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import java.util.Arrays;

public class WelzlTest {
    @Test
    public void testBoundingCircle() {
        Point[] points = {
                new Point(0.002742, 0.020184),
                new Point(-0.001640, 0.012331),
                new Point(0.008572, 0.007275),
                new Point(-0.009674, -0.039790),
        };

        System.out.println("\n\n\n\n");
        for (int i = 0; i < 10; i++) {
            Welzl.Circle circle = Welzl.welzl(Arrays.asList(points));
            System.out.println(String.format("*** My result: (%f, %f), radius %f", circle.x, circle.y, circle.r));
        }
        System.out.println("\n\n\n\n");
    }
}