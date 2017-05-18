// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Line2D;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Ceiling;

class DomainRender extends AbstractRender {
  Tensor eta;

  DomainRender(Tensor eta) {
    this.eta = eta;
  }

  @Override
  void render(AbstractLayer abstractLayer, Graphics2D graphics) {
    Tensor inv = eta.map(Scalar::invert);
    graphics.setColor(Color.LIGHT_GRAY);
    Tensor ceiling = Ceiling.of(eta);
    for (int i = 0; i < ceiling.Get(1).number().intValue(); ++i) {
      double dy = i * inv.Get(1).number().doubleValue();
      graphics.draw(new Line2D.Double( //
          abstractLayer.toPoint2D(Tensors.vector(0, dy)), //
          abstractLayer.toPoint2D(Tensors.vector(1, dy))));
    }
    for (int i = 0; i < ceiling.Get(0).number().intValue(); ++i) {
      double dx = i * inv.Get(0).number().doubleValue();
      graphics.draw(new Line2D.Double( //
          abstractLayer.toPoint2D(Tensors.vector(dx, 0)), //
          abstractLayer.toPoint2D(Tensors.vector(dx, 1))));
    }
  }
}
