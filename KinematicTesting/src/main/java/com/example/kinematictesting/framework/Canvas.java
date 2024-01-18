package com.example.kinematictesting.framework;

import com.acmerobotics.dashboard.canvas.CanvasOp;
import com.acmerobotics.dashboard.canvas.Circle;
import com.acmerobotics.dashboard.canvas.Fill;
import com.acmerobotics.dashboard.canvas.Polygon;
import com.acmerobotics.dashboard.canvas.Polyline;
import com.acmerobotics.dashboard.canvas.Spline;
import com.acmerobotics.dashboard.canvas.Stroke;
import com.acmerobotics.dashboard.canvas.StrokeWidth;

import java.awt.Image;
import java.util.ArrayList;
import java.util.List;

public class Canvas {
    private List<CanvasOp> ops;

    public Canvas() {
        ops = new ArrayList<>();
    }

    public Canvas setScale(double scaleX, double scaleY) {
        // @@@
        return this;
    }

    public Canvas setRotation(double radians) {
        // @@@
        return this;
    }

    public Canvas setTranslation(double x, double y) {
        // @@@
        return this;
    }

    public Canvas strokeText(String text, double x, double y, String font, double theta,
                             boolean usePageFrame) {
        // @@@
        return this;
    }

    public Canvas strokeText(String text, double x, double y, String font, double theta) {
        // @@@
        return this;
    }

    public Canvas fillText(String text, double x, double y, String font, double theta,
                           boolean usePageFrame) {
        // @@@
        return this;
    }

    public Canvas fillText(String text, double x, double y, String font, double theta) {
        // @@@
        fillText(text, x, y, font, theta, true);
        return this;
    }

    public Canvas strokeCircle(double x, double y, double radius) {
        // @@@
        return this;
    }

    public Canvas fillCircle(double x, double y, double radius) {
        // @@@
        return this;
    }

    public Canvas strokePolygon(double[] xPoints, double[] yPoints) {
        // @@@
        return this;
    }

    public Canvas fillPolygon(double[] xPoints, double[] yPoints) {
        // @@@
        return this;
    }

    public Canvas strokePolyline(double[] xPoints, double[] yPoints) {
        // @@@
        return this;
    }

    public Canvas strokeLine(double x1, double y1, double x2, double y2) {
        // @@@
        strokePolyline(new double[] {x1, x2}, new double[] {y1, y2});
        return this;
    }

    public Canvas fillRect(double x, double y, double width, double height) {
        // @@@
        fillPolygon(new double[] {x, x + width, x + width, x},
                new double[] {y, y, y + height, y + height});
        return this;
    }

    public Canvas strokeRect(double x, double y, double width, double height) {
        // @@@
        strokePolygon(new double[] {x, x + width, x + width, x},
                new double[] {y, y, y + height, y + height});
        return this;
    }

    @Deprecated
    public Canvas strokeSpline(double ax, double bx, double cx, double dx, double ex, double fx,
                               double ay, double by, double cy, double dy, double ey, double fy) {
        // @@@
        return this;
    }

    public Canvas setFill(String color) {
        // @@@
        return this;
    }

    public Canvas setStroke(String color) {
        // @@@
        return this;
    }

    public Canvas setStrokeWidth(int width) {
        // @@@
        return this;
    }

    /**
     * Draws an image served at the given path. All files stored in the assets images/ folder will
     * be served under path /images/.
     */
    public Canvas drawImage(String path, double x, double y, double width, double height) {
        // @@@
        drawImage(path, x, y, width, height, 0, 0, 0, true);
        return this;
    }

    public Canvas drawImage(String path, double x, double y, double width, double height,
                            double theta, double pivotX, double pivotY, boolean usePageFrame) {
        // @@@
        return this;
    }

    public Canvas drawGrid(double x, double y, double width, double height, int numTicksX,
                           int numTicksY) {
        // @@@
        drawGrid(x, y, width, height, numTicksX, numTicksY, 0, 0, 0, true);
        return this;
    }

    public Canvas drawGrid(double x, double y, double width, double height, int numTicksX,
                           int numTicksY, double theta, double pivotX, double pivotY,
                           boolean usePageFrame) {
        // @@@
        return this;
    }

    /**
     * Set the global alpha for subsequent operations.
     */
    public Canvas setAlpha(double alpha) {
        // @@@
        return this;
    }

    public List<CanvasOp> getOperations() {
        return ops;
    }

    public void clear() {
        // @@@
        this.ops.clear();
    }
}
