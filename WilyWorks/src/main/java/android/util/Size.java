package android.util;

public class Size {
    int width;
    int height;

    public Size(int width, int height) {
        this.width = width;
        this.height = height;
    }
    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public boolean equals(Object obj) {
        Size other = (Size) obj;
        return other.width == width && other.height == height;
    }

    public String toString() {
        return String.format("Size(%d, %d)", width, height);
    }

    public static Size parseSize(String string) throws NumberFormatException {
        throw new NumberFormatException();
    }

    public int hashCode() {
        throw new RuntimeException("Unimplemented");
    }
}
