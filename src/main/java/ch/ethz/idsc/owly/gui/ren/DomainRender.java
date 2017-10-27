// code by jph
package ch.ethz.idsc.owly.gui.ren;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.util.Map;

import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

class DomainRender implements RenderInterface {
  private static final Color SHADING = Color.white;
  private static final Color FILLCOL = new Color(192, 192, 192, 64);
  public static final TensorUnaryOperator EXTRACT2 = tensor -> tensor.extract(0, 2);
  // ---
  private final Map<Tensor, GlcNode> map;
  private final Tensor eta_invert;
  private final Tensor ratios;
  private final Tensor ratios2;

  DomainRender(Map<Tensor, GlcNode> map, Tensor eta) {
    this.map = map;
    this.eta_invert = eta.extract(0, 2).map(Scalar::reciprocal);
    {
      double lo = 0; // 0.1;
      double hi = 1;// 0.9;
      ratios = Tensors.of( //
          eta_invert.pmul(Tensors.vector(lo, lo)), //
          eta_invert.pmul(Tensors.vector(hi, lo)), //
          eta_invert.pmul(Tensors.vector(hi, hi)), //
          eta_invert.pmul(Tensors.vector(lo, hi)));
    }
    {
      double lo = 0.05;
      double hi = 0.95;
      ratios2 = Tensors.of( //
          eta_invert.pmul(Tensors.vector(lo, lo)), //
          eta_invert.pmul(Tensors.vector(hi, lo)), //
          eta_invert.pmul(Tensors.vector(hi, hi)), //
          eta_invert.pmul(Tensors.vector(lo, hi)));
    }
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    map.keySet().stream().map(EXTRACT2).distinct().forEach(key -> {
      graphics.setColor(FILLCOL);
      {
        Tensor x = key.pmul(eta_invert);
        Path2D path2d = geometricLayer.toPath2D(Tensor.of(ratios.stream().map(x::add)));
        // path2d.closePath();
        graphics.fill(path2d);
      }
      graphics.setColor(SHADING);
      {
        Tensor x = key.pmul(eta_invert);
        Path2D path2d = geometricLayer.toPath2D(Tensor.of(ratios.stream().map(x::add)));
        path2d.closePath();
        graphics.draw(path2d);
      }
    });
  }
}
