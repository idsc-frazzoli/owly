// code by ynager
// render 2d points contained in a Tensor.
package ch.ethz.idsc.owl.gui.ren;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.Objects;

import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.math.map.Se2Utils;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.lie.CirclePoints;

public class PointRender implements RenderInterface {
  private final Tensor points;
  private final Color color;
  private final Tensor shape;

  public PointRender(Tensor points, double radius, Color color) {
    this.points = Objects.requireNonNull(points);
    this.color = color;
    shape = CirclePoints.of(7).multiply(RealScalar.of(radius));
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    // draw points
    graphics.setColor(color);
    for (Tensor p : points) {
      geometricLayer.pushMatrix(Se2Utils.toSE2Matrix(p.extract(0, 2).copy().append(RealScalar.ZERO)));
      graphics.fill(geometricLayer.toPath2D(shape));
      geometricLayer.popMatrix();
    }
  }
}
