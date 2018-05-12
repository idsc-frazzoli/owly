// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.owl.glc.adapter.GoalAdapter;
import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.core.CostFunction;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.math.planar.ConeRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.region.So2Region;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** suggested base class for se2 goal managers.
 * all implemented methods in this layer are final.
 * 
 * class defines circle region for (x,y) component and periodic intervals in angular component */
public abstract class Se2AbstractConeGoalManager implements Region<Tensor>, CostFunction {
  private final ConeRegion coneRegion;
  private final So2Region so2Region;

  /** @param center of region with coordinates (x, y, theta)
   * @param radiusVector with 3 entries the first 2 of which have to be identical
   * @param tolerance */
  protected Se2AbstractConeGoalManager(Tensor xya, Scalar semi, Scalar tolerance) {
    coneRegion = new ConeRegion(xya, semi);
    so2Region = new So2Region(xya.Get(2), tolerance);
  }

  /** @param tensor == {px, py, angle}
   * @return signed distance of {px, py} from spherical region */
  protected final Scalar d_xy(Tensor tensor) {
    return coneRegion.distance(tensor.extract(0, 2));
  }

  /** @param tensor == {px, py, angle}
   * @return signed distance of angle from so2region */
  protected final Scalar d_angle(Tensor tensor) {
    return so2Region.apply(tensor.get(2));
  }

  @Override // from Region
  public final boolean isMember(Tensor tensor) {
    return coneRegion.isMember(tensor.extract(0, 2)) && so2Region.isMember(tensor.get(2));
  }

  public final GoalInterface getGoalInterface() {
    return new GoalAdapter(SimpleTrajectoryRegionQuery.timeInvariant(this), this);
  }
}
