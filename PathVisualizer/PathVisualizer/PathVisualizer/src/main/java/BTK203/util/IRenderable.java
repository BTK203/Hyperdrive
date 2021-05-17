package BTK203.util;

import java.awt.Color;

/**
 * This interface should be implemented by anything that can be rendered by the {@code Visualizer}.
 */
public interface IRenderable {
    /**
     * Returns the number of points that are present in the Renderable.
     * @return Array of all points in the renderable.
     */
    public Point2D[] getPoints();

    /**
     * Returns the color of the renderable.
     * @return Color of the renderable.
     */
    public Color     getColor();

    /**
     * Returns whether or not the Renderable is visible to the user or not.
     * @return {@code true} if the renderable is visible, {@code false} otherwise.
     */
    public boolean   isVisible();

    /**
     * Returns whether or not the Renderable is valid and displayable.
     * @return {@code true} if the renderable can be displayed, {@code false} otherwise.
     */
    public boolean   isValid();

    /**
     * Sets the renderable's visibility.
     * @param visible {@code true} for visible, {@code false} for not.
     */
    public void      setVisible(boolean visible);

    /**
     * Returns the name of the renderable.
     * @return The name.
     */
    public String    getName();
}
