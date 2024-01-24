package com.example.kinematictesting.framework;

import com.acmerobotics.dashboard.canvas.CanvasOp;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.util.ArrayList;

class Circle extends CanvasOp {
    public double x;
    public double y;
    public double radius;
    public boolean stroke;

    public Circle(double x, double y, double radius, boolean stroke) {
        super(Type.CIRCLE);

        this.x = x;
        this.y = y;
        this.radius = radius;
        this.stroke = stroke;
    }
}

class Fill extends CanvasOp {
    public String color;

    public Fill(String color) {
        super(Type.FILL);

        this.color = color;
    }
}

class Polygon extends CanvasOp {
    public double[] xPoints;
    public double[] yPoints;
    public boolean stroke;

    public Polygon(double[] xPoints, double[] yPoints, boolean stroke) {
        super(Type.POLYGON);

        this.xPoints = xPoints;
        this.yPoints = yPoints;
        this.stroke = stroke;
    }
}

class Polyline extends CanvasOp {
    public double[] xPoints;
    public double[] yPoints;

    public Polyline(double[] xPoints, double[] yPoints) {
        super(Type.POLYLINE);

        this.xPoints = xPoints;
        this.yPoints = yPoints;
    }
}

class Spline extends CanvasOp {
    public double ax, bx, cx, dx, ex, fx;
    public double ay, by, cy, dy, ey, fy;

    public Spline(double ax, double bx, double cx, double dx, double ex, double fx,
                  double ay, double by, double cy, double dy, double ey, double fy) {
        super(Type.SPLINE);

        this.ax = ax;
        this.bx = bx;
        this.cx = cx;
        this.dx = dx;
        this.ex = ex;
        this.fx = fx;

        this.ay = ay;
        this.by = by;
        this.cy = cy;
        this.dy = dy;
        this.ey = ey;
        this.fy = fy;
    }
}

class Stroke extends CanvasOp {
    public String color;

    public Stroke(String color) {
        super(Type.STROKE);

        this.color = color;
    }
}

class StrokeWidth extends CanvasOp {
    public int width;

    public StrokeWidth(int width) {
        super(Type.STROKE_WIDTH);

        this.width = width;
    }
}

public class Canvas {
    private ArrayList<CanvasOp> ops;

    public Canvas() {
        ops = new ArrayList<>();
    }

    public Canvas strokeCircle(double x, double y, double radius) {
        ops.add(new Circle(x, y, radius, true));
        return this;
    }

    public Canvas fillCircle(double x, double y, double radius) {
        ops.add(new Circle(x, y, radius, false));
        return this;
    }

    public Canvas strokePolygon(double[] xPoints, double[] yPoints) {
        ops.add(new Polygon(xPoints, yPoints, true));
        return this;
    }

    public Canvas fillPolygon(double[] xPoints, double[] yPoints) {
        ops.add(new Polygon(xPoints, yPoints, false));
        return this;
    }

    public Canvas strokePolyline(double[] xPoints, double[] yPoints) {
        ops.add(new Polyline(xPoints, yPoints));
        return this;
    }

    public Canvas strokeLine(double x1, double y1, double x2, double y2) {
        strokePolyline(new double[] { x1, x2 }, new double[] { y1, y2 });
        return this;
    }

    public Canvas fillRect(double x, double y, double width, double height) {
        fillPolygon(new double[] { x, x + width, x + width, x },
                new double[] { y, y, y + height, y + height });
        return this;
    }

    public Canvas strokeRect(double x, double y, double width, double height) {
        strokePolygon(new double[] { x, x + width, x + width, x },
                new double[] { y, y, y + height, y + height });
        return this;
    }

    @Deprecated
    public Canvas strokeSpline(double ax, double bx, double cx, double dx, double ex, double fx,
                               double ay, double by, double cy, double dy, double ey, double fy) {
        ops.add(new Spline(ax, bx, cx, dx, ex, fx, ay, by, cy, dy, ey, fy));
        return this;
    }

    public Canvas setFill(String color) {
        ops.add(new Fill(color));
        return this;
    }

    public Canvas setStroke(String color) {
        ops.add(new Stroke(color));
        return this;
    }

    public Canvas setStrokeWidth(int width) {
        ops.add(new StrokeWidth(width));
        return this;
    }

    public ArrayList<CanvasOp> getOperations() {
        return ops;
    }

    public void clear() {
        this.ops.clear();
    }

    private static int round(double x) { return (int) Math.round(x); }

    private int[] coords(double[] coords, boolean isPolygon) {
        int[] result = new int[coords.length + 1];
        for (int i = 0; i < coords.length; i++) {
            result[i] = round(coords[i]);
        }
        if (isPolygon)
            result[coords.length] = round(coords[0]);
        return result;
    }

    public void render(Graphics2D g) {
        // https://github.dev/acmerobotics/ftc-dashboard/blob/26920d66b1abe1e03d5d10d7ec3701467ea56a0c/FtcDashboard/dash/src/components/views/FieldView/Field.js
        Color strokeColor = Color.BLACK;
        Color fillColor = Color.WHITE;
        for (CanvasOp op : getOperations()) {
            if (op instanceof Circle) {
                Circle circle = (Circle) op;
                if (circle.stroke) {
                    g.setColor(strokeColor);
                    g.draw(new Ellipse2D.Double(circle.radius, circle.radius, circle.x, circle.y));
                } else {
                    g.setColor(fillColor);
                    g.fill(new Ellipse2D.Double(circle.radius, circle.radius, circle.x, circle.y));
                }
            } else if (op instanceof Polygon) {
                Polygon polygon = (Polygon) op;
                if (polygon.stroke) {
                    g.setColor(strokeColor);
                    g.drawPolygon(coords(polygon.xPoints, true),
                            coords(polygon.yPoints, true),
                            polygon.xPoints.length);
                } else {
                    g.setColor(fillColor);
                    g.drawPolyline(coords(polygon.xPoints, true),
                            coords(polygon.yPoints, true),
                            polygon.xPoints.length);
                }
            } else if (op instanceof Polyline) {
                Polyline polyline = (Polyline) op;
                g.setColor(strokeColor);
                g.drawPolyline(coords(polyline.xPoints, false),
                               coords(polyline.yPoints, false),
                               polyline.xPoints.length);
            } else if (op instanceof Spline) {
                Spline spline = (Spline) op;
                final int SPLINE_SAMPLES = 250;
                int[] xPoints = new int[SPLINE_SAMPLES + 1];
                int[] yPoints = new int[SPLINE_SAMPLES + 1];
                g.setColor(strokeColor);
                for (int i = 0; i <= SPLINE_SAMPLES; i++) {
                    double t = (double) i / SPLINE_SAMPLES;
                    xPoints[i] = round((spline.ax * t + spline.bx) * (t * t * t * t) +
                            spline.cx * (t * t * t) +
                            spline.dx * (t * t) +
                            spline.ex * t +
                            spline.fx);
                    yPoints[i] = round((spline.ay * t + spline.by) * (t * t * t * t) +
                            spline.cy * (t * t * t) +
                            spline.dy * (t * t) +
                            spline.ey * t +
                            spline.fy);
                }
                g.drawPolyline(xPoints, yPoints, xPoints.length);
            } else if (op instanceof Stroke) {
                // c.setStroke("#3F51B5");
                Stroke stroke = (Stroke) op;
                strokeColor = Color.decode(stroke.color);
            } else if (op instanceof Fill) {
                Fill fill = (Fill) op;
                fillColor = Color.decode(fill.color);
            } else if (op instanceof StrokeWidth) {
                StrokeWidth strokeWidth = (StrokeWidth) op;
                g.setStroke(new BasicStroke(strokeWidth.width));
            } else {
                throw new IllegalArgumentException("Unexpected field overlay op");
            }
        }
        clear();
    }
}
