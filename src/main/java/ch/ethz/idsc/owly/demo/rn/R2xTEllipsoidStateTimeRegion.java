// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.util.function.Supplier;

import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.math.map.BijectionFamily;
import ch.ethz.idsc.owl.math.planar.EllipsePoints;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.region.SphericalRegion;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.opt.TensorUnaryOperator;
import ch.ethz.idsc.tensor.red.Norm2Squared;

/** ellipsoid region that is moving with respect to time */
public class R2xTEllipsoidStateTimeRegion implements Region<StateTime>, RenderInterface {
  /** number of samples to visualize ellipsoid */
  private static final int RESOLUTION = 22;
  // ---
  private final Tensor invert;
  protected final Tensor polygon;
  protected final BijectionFamily bijectionFamily;
  protected final Supplier<Scalar> supplier;

  /** @param radius encodes principle axis of ellipsoid region
   * @param bijectionFamily with origin at center of ellipsoid region
   * @param supplier for parameter to evaluate bijectionFamily */
  public R2xTEllipsoidStateTimeRegion(Tensor radius, BijectionFamily bijectionFamily, Supplier<Scalar> supplier) {
    invert = radius.map(Scalar::reciprocal);
    this.bijectionFamily = bijectionFamily;
    this.supplier = supplier;
    polygon = EllipsePoints.of(RESOLUTION, radius.extract(0, 2));
  }

  @Override // from Region
  public boolean isMember(StateTime stateTime) {
    Tensor state = stateTime.state().extract(0, invert.length());
    Scalar time = stateTime.time();
    TensorUnaryOperator rev = bijectionFamily.inverse(time);
    return Scalars.lessEquals(Norm2Squared.ofVector(rev.apply(state).pmul(invert)), RealScalar.ONE);
  }

  public Region<Tensor> regionAtTime() {
    TensorUnaryOperator fwd = bijectionFamily.forward(supplier.get());
    return new SphericalRegion(fwd.apply(Tensors.vector(0, 0)), invert.map(Scalar::reciprocal).Get(0));
  }

  @Override // from RenderInterface
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Scalar time = supplier.get();
    TensorUnaryOperator fwd = bijectionFamily.forward(time);
    Path2D path2D = geometricLayer.toPath2D(Tensor.of(polygon.stream().map(fwd)));
    graphics.setColor(RegionRenders.COLOR);
    graphics.fill(path2D);
    graphics.setColor(RegionRenders.BOUNDARY);
    path2D.closePath();
    graphics.draw(path2D);
  }
}
