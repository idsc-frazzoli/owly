// code by jph
package ch.ethz.idsc.owl.gui.ren;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Line2D;

import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Ceiling;

public class EtaRender implements RenderInterface {
  private Tensor eta;

  public EtaRender(Tensor eta) {
    setEta(eta);
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    if (eta.length() < 2)
      return;
    Tensor inv = eta.map(Scalar::reciprocal);
    graphics.setColor(Color.LIGHT_GRAY);
    Tensor ceiling = Ceiling.of(eta);
    for (int i = 0; i < ceiling.Get(1).number().intValue(); ++i) {
      double dy = i * inv.Get(1).number().doubleValue();
      graphics.draw(new Line2D.Double( //
          geometricLayer.toPoint2D(Tensors.vector(0, dy)), //
          geometricLayer.toPoint2D(Tensors.vector(1, dy))));
    }
    for (int i = 0; i < ceiling.Get(0).number().intValue(); ++i) {
      double dx = i * inv.Get(0).number().doubleValue();
      graphics.draw(new Line2D.Double( //
          geometricLayer.toPoint2D(Tensors.vector(dx, 0)), //
          geometricLayer.toPoint2D(Tensors.vector(dx, 1))));
    }
  }

  public void setEta(Tensor eta) {
    this.eta = eta;
  }
}
