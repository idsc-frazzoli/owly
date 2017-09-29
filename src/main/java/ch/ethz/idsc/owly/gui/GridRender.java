// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Line2D;

import ch.ethz.idsc.tensor.Tensors;

public enum GridRender implements RenderInterface {
  INSTANCE;
  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    {
      graphics.setColor(Color.LIGHT_GRAY);
      graphics.draw(new Line2D.Double(owlyLayer.toPoint2D(Tensors.vector(-10, 1)), owlyLayer.toPoint2D(Tensors.vector(10, 1))));
      graphics.draw(new Line2D.Double(owlyLayer.toPoint2D(Tensors.vector(1, -10)), owlyLayer.toPoint2D(Tensors.vector(1, 10))));
    }
    {
      graphics.setColor(Color.GRAY);
      graphics.draw(new Line2D.Double(owlyLayer.toPoint2D(Tensors.vector(-10, 0)), owlyLayer.toPoint2D(Tensors.vector(10, 0))));
      graphics.draw(new Line2D.Double(owlyLayer.toPoint2D(Tensors.vector(0, -10)), owlyLayer.toPoint2D(Tensors.vector(0, 10))));
    }
  }
}
