package ch.ethz.idsc.owly.glc.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Line2D;

import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Ceiling;

class DomainLayer extends AbstractLayer {
  DomainLayer(GlcComponent glcComponent) {
    super(glcComponent);
  }

  @Override
  void render(Graphics2D graphics, TrajectoryPlanner trajectoryPlanner) {
    Tensor resolution = trajectoryPlanner.getResolution();
    Tensor inv = resolution.map(Scalar::invert);
    graphics.setColor(Color.LIGHT_GRAY);
    Tensor ceiling = Ceiling.of(resolution);
    for (int i = 0; i < ceiling.Get(1).number().intValue(); ++i) {
      double dy = i * inv.Get(1).number().doubleValue();
      graphics.draw(new Line2D.Double( //
          toPoint2D(Tensors.vector(0, dy)), //
          toPoint2D(Tensors.vector(1, dy))));
    }
    for (int i = 0; i < ceiling.Get(0).number().intValue(); ++i) {
      double dx = i * inv.Get(0).number().doubleValue();
      graphics.draw(new Line2D.Double( //
          toPoint2D(Tensors.vector(dx, 0)), //
          toPoint2D(Tensors.vector(dx, 1))));
    }
  }
}
