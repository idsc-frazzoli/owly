// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.awt.Graphics2D;
import java.util.function.Supplier;

import ch.ethz.idsc.owly.demo.util.BijectionFamily;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.region.RegionRenders;
import ch.ethz.idsc.owly.math.Polygons;
import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.N;

/** check if input tensor is inside a polygon */
public class R2xTPolygonStateTimeRegion implements Region<StateTime>, RenderInterface {
  private final Tensor polygon;
  private final BijectionFamily bijectionFamily;
  private final Supplier<Scalar> supplier;

  // TODO JAN design with time provider is not final...
  public R2xTPolygonStateTimeRegion(Tensor polygon, BijectionFamily bijectionFamily, Supplier<Scalar> supplier) {
    this.polygon = N.DOUBLE.of(polygon);
    this.bijectionFamily = bijectionFamily;
    this.supplier = supplier;
  }

  @Override
  public boolean isMember(StateTime stateTime) {
    Tensor state = stateTime.state().extract(0, 2);
    Scalar time = stateTime.time();
    TensorUnaryOperator rev = bijectionFamily.inverse(time);
    return Polygons.isInside(polygon, rev.apply(state));
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Scalar time = supplier.get();
    graphics.setColor(RegionRenders.COLOR);
    TensorUnaryOperator fwd = bijectionFamily.forward(time);
    graphics.fill(geometricLayer.toPath2D(Tensor.of(polygon.stream().map(fwd))));
  }
}
