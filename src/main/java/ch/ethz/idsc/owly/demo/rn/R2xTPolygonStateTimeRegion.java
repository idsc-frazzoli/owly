// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.util.function.Supplier;

import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.region.RegionRenders;
import ch.ethz.idsc.owly.math.Polygons;
import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.se2.BijectionFamily;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.N;

/** check if input tensor is inside a polygon */
public class R2xTPolygonStateTimeRegion implements Region<StateTime>, RenderInterface {
  private final Tensor polygon;
  private final BijectionFamily bijectionFamily;
  private final Supplier<Scalar> supplier;

  /** @param polygon
   * @param bijectionFamily
   * @param supplier */
  public R2xTPolygonStateTimeRegion(Tensor polygon, BijectionFamily bijectionFamily, Supplier<Scalar> supplier) {
    this.polygon = N.DOUBLE.of(polygon);
    this.bijectionFamily = bijectionFamily;
    this.supplier = supplier;
  }

  @Override // from Region
  public boolean isMember(StateTime stateTime) {
    Tensor state = stateTime.state().extract(0, 2);
    Scalar time = stateTime.time();
    TensorUnaryOperator rev = bijectionFamily.inverse(time);
    return Polygons.isInside(polygon, rev.apply(state));
  }

  @Override // from RenderInterface
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Scalar time = supplier.get();
    TensorUnaryOperator forward = bijectionFamily.forward(time);
    Path2D path2D = geometricLayer.toPath2D(Tensor.of(polygon.stream().map(forward)));
    graphics.setColor(RegionRenders.COLOR);
    graphics.fill(path2D);
    graphics.setColor(RegionRenders.BOUNDARY);
    path2D.closePath();
    graphics.draw(path2D);
  }
}
