// code by ynager
// render 2d points contained in a Tensor.
package ch.ethz.idsc.owl.gui.ren;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.util.Objects;

import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.tensor.Tensor;

public class PointRender implements RenderInterface {
  private Color color;
  private Tensor points;
  private double radius;

  public PointRender(Tensor points, double radius, Color color) {
    this.points = points;
    this.color = color;
    this.radius = radius;
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    if (Objects.isNull(points))
      return;
    // draw points
    graphics.setColor(color);
    for (Tensor p : points) {
      Point2D point2d = geometricLayer.toPoint2D(p);
      graphics.draw(new Ellipse2D.Double(point2d.getX(), point2d.getY(), radius, radius));
    }
  }
}
