package org.firstinspires.ftc.teamcode;

import org.junit.Assert;
import org.junit.Test;

public class PointTest {

    public PointTest() {
        Point point = new Point(2, 3);
        Assert.assertEquals(point.x, point.y, 1.0);
        point.dot(new Point(3, 5));
    }

    @Test
    public void add() {
    }
}
