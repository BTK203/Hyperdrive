package BTK203.util;

import java.io.IOException;
import java.nio.file.Files;

import BTK203.Constants;

import java.awt.Color;

/**
 * Represents a Path that can be rendered on the screen.
 */
public class Path implements IRenderable {
    private static final int MAX_ALLOWED_NULL_POINTS = 5;
    private static float currentHue = 0;

    private Point2D[] points;
    private Color color;
    private boolean
        valid,
        visible;

    private String name;

    /**
     * Creates a new path from the given file path.
     * @param file The path to the file to read from.
     * @param color The color of the path.
     */
    public Path(String file, Color color) {
        this.name = file.substring(file.lastIndexOf('\\') + 1); //name is JUST the file name, not the absolute file path.
        this.color = color;
        this.valid = false;
        this.visible = true;
        try {
            String fileContents = Files.readString(java.nio.file.Path.of(file));
            String[] pointStrings = fileContents.split("\n");
            points = new Point2D[pointStrings.length];
            for(int i=0; i<pointStrings.length; i++) {
                points[i] = Point2D.fromString(pointStrings[i]);
            }

            valid = true;
        } catch (IOException ex) {
            ex.printStackTrace();
        } catch (NumberFormatException ex) {
            System.out.println("Invalid File!");
        }
    }

    /**
     * Creates a new Path.
     * @param points An array of points describing the path.
     * @param color The color of the path.
     */
    public Path(Point2D[] points, Color color, String name) {
        this.points = points;
        this.color = color;
        this.name = name;
        this.valid = points.length > 0;
        this.visible = true;
    }

    public Point2D[] getPoints() {
        return points;
    }

    public boolean isValid() {
        return valid;
    }

    public void setVisible(boolean visible) {
        this.visible = visible;
    }

    public boolean isVisible() {
        return visible;
    }

    public Color getColor() {
        return color;
    }

    public String getName() {
        return name;
    }

    /**
     * Sets the name of a Path. This should really only be used if avoiding duplicate names.
     * @param name The new name of the Path.
     */
    public void setName(String name) {
        this.name = name;
    }

    /**
     * Converts the Path into a user (and computer) readable String.
     * @return The string representation of the Path.
     */
    public String toString() {
        String pathString = "";
        for(Point2D point : points) {
            pathString += point.toString() + "\n";
        }

        return pathString;
    }

    /**
     * Creates a new Path from the given string.
     * @param string The string to interpret.
     * @param name The name of the new Path.
     * @return A Path containing the information in the String.
     */
    public static Path fromString(String string, String name) {
        String[] pointStrings = string.split("\n");
        Point2D[] points = new Point2D[pointStrings.length];
        int nullPoints = 0;
        for(int p=0; p<points.length; p++) {
            points[p] = Point2D.fromString(pointStrings[p]);
            if(points[p] == null) {
                nullPoints++;
                if(nullPoints > MAX_ALLOWED_NULL_POINTS) {
                    return null;
                }
            }
        }

        return new Path(points, Path.getNextColor(), name);
    }

    /**
     * Gets the next color for a new path.
     * @return The next color to use.
     */
    public static Color getNextColor() {
        Color nextColor = new Color(Color.HSBtoRGB(currentHue, Constants.NEXTCOLOR_SATURATION, Constants.NEXTCOLOR_BRIGHTNESS));
        currentHue += Constants.NEXTCOLOR_HUE_INCREMENT;
        return nextColor;
    }
}
