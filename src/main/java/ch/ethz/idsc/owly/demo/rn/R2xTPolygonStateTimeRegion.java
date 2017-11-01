// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.awt.Graphics2D;
import java.util.function.Supplier;

import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.region.RegionRenders;
import ch.ethz.idsc.owly.math.Polygons;
import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.StateTimeRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.N;
import ch.ethz.idsc.tensor.sca.Sin;

/** check if input tensor is inside a polygon */
public class R2xTPolygonStateTimeRegion implements StateTimeRegion, RenderInterface {
  private final Tensor polygon;
  private final Supplier<Scalar> supplier;

  // TODO JAN design with time provider is not final...
  public R2xTPolygonStateTimeRegion(Tensor polygon, Supplier<Scalar> supplier) {
    this.polygon = N.DOUBLE.of(polygon);
    this.supplier = supplier;
  }

  @Override
  public boolean isMember(StateTime stateTime) {
    Tensor state = stateTime.state().extract(0, 2);
    Scalar time = stateTime.time();
    TensorUnaryOperator rev = reverse(time);
    return Polygons.isInside(polygon, rev.apply(state));
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Scalar time = supplier.get();
    graphics.setColor(RegionRenders.COLOR);
    TensorUnaryOperator fwd = forward(time);
    graphics.fill(geometricLayer.toPath2D(Tensor.of(polygon.stream().map(fwd))));
  }

  private static TensorUnaryOperator reverse(Scalar time) {
    Tensor offset = translation(time);
    return tensor -> tensor.subtract(offset);
  }

  private static TensorUnaryOperator forward(Scalar time) {
    Tensor offset = translation(time);
    return tensor -> tensor.add(offset);
  }

  private static Tensor translation(Scalar time) {
    return Tensors.of(Sin.FUNCTION.apply(time.multiply(RealScalar.of(0.2))), RealScalar.ZERO);
  }
}
