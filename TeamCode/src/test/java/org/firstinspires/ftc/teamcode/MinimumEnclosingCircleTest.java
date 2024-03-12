package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import java.util.Arrays;

public class MinimumEnclosingCircleTest {
    @Test
    public void testBoundingCircle() {
        Point[] points = { new Point(0, 0), new Point(3, 5) };
        MinimumEnclosingCircle.Circle circle = MinimumEnclosingCircle.welzl(Arrays.asList(points));
        System.out.println(String.format("\n\n\n\n*** My result: %s", circle.toString()));
    }
}