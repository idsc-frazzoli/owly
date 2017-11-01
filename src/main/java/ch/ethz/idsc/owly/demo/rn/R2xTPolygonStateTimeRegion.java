// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.awt.Graphics2D;

import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.region.RegionRenders;
import ch.ethz.idsc.owly.math.Polygons;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.StateTimeRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Sin;

/** check if input tensor is inside a polygon */
public class R2xTPolygonStateTimeRegion implements StateTimeRegion, RenderInterface {
  private final Tensor polygon;

  public R2xTPolygonStateTimeRegion(Tensor polygon) {
    this.polygon = polygon.copy();
  }

  @Override
  public boolean isMember(StateTime stateTime) {
    Tensor state = stateTime.state().extract(0, 2);
    Scalar time = stateTime.time();
    Tensor v = Tensors.of(Sin.FUNCTION.apply(time), RealScalar.of(0));
    return Polygons.isInside(polygon, state.add(v));
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Scalar time = RealScalar.of(System.currentTimeMillis() * 1e-3);
    graphics.setColor(RegionRenders.COLOR);
    // FIXME formula hardcoded
    Tensor v = Tensors.of(Sin.FUNCTION.apply(time), RealScalar.of(0));
    graphics.fill(geometricLayer.toPath2D(Tensor.of(polygon.stream().map(t -> t.add(v)))));
  }
}
