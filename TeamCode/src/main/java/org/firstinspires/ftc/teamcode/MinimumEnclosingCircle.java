package org.firstinspires.ftc.teamcode;


import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

// Welzl's implementation courtesy of https://www.geeksforgeeks.org/minimum-enclosing-circle-using-welzls-algorithm/
public class MinimumEnclosingCircle {

    // Handy class to describe a circle:
    static class Circle {
        Point center;
        double radius;

        public Circle(Point center, double radius) {
            this.center = center; this.radius = radius;
        }

        public String toString() {
            return String.format("Center: (%.2f, %.2f), Radius: %.2f", center.x, center.y, radius);
        }
    }

    // Function to return the euclidean distance between two points
    private static double dist(Point a, Point b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }

    // Function to check whether a point lies inside or on the boundaries of the circle
    private static boolean isInside(Circle c, Point p) {
        return dist(c.center, p) <= c.radius;
    }

    // Helper method to get a circle defined by 3 points
    private static Point getCircleCenter(double bx, double by, double cx, double cy) {
        double B = bx * bx + by * by;
        double C = cx * cx + cy * cy;
        double D = bx * cy - by * cx;
        return new Point((cy * B - by * C) / (2 * D), (bx * C - cx * B) / (2 * D));
    }

    // Function to return a unique circle that intersects three points
    private static Circle circleFrom(Point A, Point B, Point C) {
        Point I = getCircleCenter(B.x - A.x, B.y - A.y, C.x - A.x, C.y - A.y);
        I.x += A.x;
        I.y += A.y;
        return new Circle(I, dist(I, A));
    }

    // Function to return the smallest circle that intersects 2 points
    private static Circle circleFrom(Point A, Point B) {
        Point C = new Point((A.x + B.x) / 2.0, (A.y + B.y) / 2.0);
        return new Circle(C, dist(A, B) / 2.0);
    }

    // Function to check whether a circle encloses the given points
    private static boolean isValidCircle(Circle c, List<Point> P) {
        // Iterating through all the points to check whether the points lie inside the circle or not
        for (Point p : P) {
            if (!isInside(c, p)) {
                return false;
            }
        }
        return true;
    }

    // Function to return the minimum enclosing circle for N <= 3
    private static Circle minCircleTrivial(List<Point> P) {
        assert P.size() <= 3;
        if (P.isEmpty()) {
            return new Circle(new Point(0, 0), 0);
        } else if (P.size() == 1) {
            return new Circle(P.get(0), 0);
        } else if (P.size() == 2) {
            return circleFrom(P.get(0), P.get(1));
        }

        // To check if MEC can be determined by 2 points only
        for (int i = 0; i < 3; i++) {
            for (int j = i + 1; j < 3; j++) {
                Circle c = circleFrom(P.get(i), P.get(j));
                if (isValidCircle(c, P)) {
                    return c;
                }
            }
        }
        return circleFrom(P.get(0), P.get(1), P.get(2));
    }

    // Returns the Minimal Enclosing Circle using Welzl's algorithm
    // Takes a set of input points P and a set R
    // points on the circle boundary.
    // n represents the number of points in P that are not yet processed.
    private static Circle welzlHelper(List<Point> P, List<Point> R, int n) {
        // Base case when all points processed or |R| = 3
        if (n == 0 || R.size() == 3) {
            return minCircleTrivial(R);
        }

        // Pick a random point randomly
        int idx = (int) (Math.random() * n);
        Point p = P.get(idx);

        // Put the picked point at the end of P since it's more efficient than
        // deleting from the middle of the list
        Collections.swap(P, idx, n - 1);

        // Get the MEC circle d from the set of points P - {p}
        Circle d = welzlHelper(P, R, n - 1);

        // If d contains p, return d
        if (isInside(d, p)) {
            return d;
        }

        // Otherwise, must be on the boundary of the MEC
        R.add(p);

        // Return the MEC for P - {p} and R U {p}
        return welzlHelper(P, R, n - 1);
    }

    // Function to find the minimum enclosing circle for N integer points in a 2-D plane
    public static Circle welzl(List<Point> P) {
        List<Point> PCopy = new ArrayList<>(P);
        Collections.shuffle(PCopy);
        return welzlHelper(PCopy, new ArrayList<>(), PCopy.size());
    }
}
