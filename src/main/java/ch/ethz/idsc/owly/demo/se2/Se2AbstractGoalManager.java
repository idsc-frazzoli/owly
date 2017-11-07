// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.owly.data.DontModify;
import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.glc.adapter.GoalAdapter;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.VectorQ;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Mod;

/** suggested base class for se2 goal managers.
 * all implemented methods in this layer are final.
 * 
 * class defines circle region for (x,y) component and periodic intervals in angular component */
@DontModify
public abstract class Se2AbstractGoalManager implements Region<Tensor>, CostFunction {
  static final Mod PRINCIPAL = Mod.function(2 * Math.PI, -Math.PI);
  // ---
  protected final Tensor center;
  protected final Tensor radiusVector;

  /** @param center of region with coordinates (x, y, theta)
   * @param radiusVector with 3 entries the first 2 of which have to be identical */
  public Se2AbstractGoalManager(Tensor center, Tensor radiusVector) {
    GlobalAssert.that(radiusVector.get(0).equals(radiusVector.get(1)));
    GlobalAssert.that(VectorQ.ofLength(center, 3));
    GlobalAssert.that(VectorQ.ofLength(radiusVector, 3));
    this.center = center.unmodifiable();
    this.radiusVector = radiusVector.unmodifiable();
  }

  protected final Scalar radiusSpace() {
    return radiusVector.Get(0);
  }

  protected final Scalar radiusAngle() {
    return radiusVector.Get(2);
  }

  protected final Scalar d_xy(Tensor tensor) {
    Tensor cur_xy = tensor.extract(0, 2);
    return Norm._2.between(cur_xy, center.extract(0, 2));
  }

  protected final Scalar d_angle(Tensor tensor) {
    Scalar cur_angle = tensor.Get(2);
    return PRINCIPAL.apply(cur_angle.subtract(center.Get(2))).abs();
  }

  @Override
  public final boolean isMember(Tensor tensor) {
    boolean status = true;
    status &= Scalars.lessEquals(d_xy(tensor), radiusSpace());
    status &= Scalars.lessEquals(d_angle(tensor), radiusAngle());
    return status;
  }

  public final GoalInterface getGoalInterface() {
    return new GoalAdapter(this, SimpleTrajectoryRegionQuery.timeInvariant(this));
  }
}
