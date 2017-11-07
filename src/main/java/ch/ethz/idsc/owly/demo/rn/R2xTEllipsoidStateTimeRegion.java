// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.awt.Graphics2D;
import java.util.function.Supplier;

import ch.ethz.idsc.owly.demo.util.BijectionFamily;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.region.RegionRenders;
import ch.ethz.idsc.owly.math.RegularPolygon;
import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.StateTimeRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm2Squared;

/** check if input tensor is inside a polygon */
public class R2xTEllipsoidStateTimeRegion implements StateTimeRegion, RenderInterface {
  private final Tensor invert;
  private final Tensor polygon;
  private final BijectionFamily bijectionFamily;
  private final Supplier<Scalar> supplier;

  // TODO JAN design with time provider is not final...
  public R2xTEllipsoidStateTimeRegion(Tensor radius, BijectionFamily bijectionFamily, Supplier<Scalar> supplier) {
    invert = radius.map(Scalar::reciprocal);
    this.bijectionFamily = bijectionFamily;
    this.supplier = supplier;
    polygon = RegularPolygon.elliptic(22, radius.Get(0), radius.Get(1));
  }

  @Override
  public boolean isMember(StateTime stateTime) {
    Tensor state = stateTime.state().extract(0, invert.length());
    Scalar time = stateTime.time();
    TensorUnaryOperator rev = bijectionFamily.inverse(time);
    return Scalars.lessEquals(Norm2Squared.ofVector(rev.apply(state).pmul(invert)), RealScalar.ONE);
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Scalar time = supplier.get();
    graphics.setColor(RegionRenders.COLOR);
    TensorUnaryOperator fwd = bijectionFamily.forward(time);
    graphics.fill(geometricLayer.toPath2D(Tensor.of(polygon.stream().map(fwd))));
  }
}
