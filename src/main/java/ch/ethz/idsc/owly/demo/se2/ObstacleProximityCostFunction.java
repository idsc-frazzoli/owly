// code by ynager
package ch.ethz.idsc.owly.demo.se2;

import java.util.List;

import ch.ethz.idsc.owl.data.Lists;
import ch.ethz.idsc.owl.glc.core.CostFunction;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.mapping.OccupancyMap2d;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Clip;

/** Cost function that penalizes distance to obstacles */
public final class ObstacleProximityCostFunction implements CostFunction {
  private final Scalar penaltyFactor;
  private final Scalar distThreshold;
  private final Clip clip;
  private final OccupancyMap2d occupancyMap;

  /** @param occupancyMap instance
   * @param penaltyFactor
   * @param distThreshold if distance to obstacle is above threshold cost is zero */
  public ObstacleProximityCostFunction(OccupancyMap2d occupancyMap, Scalar penaltyFactor, Scalar distThreshold) {
    this.penaltyFactor = penaltyFactor;
    this.occupancyMap = occupancyMap;
    this.distThreshold = distThreshold;
    clip = Clip.function(distThreshold.zero(), distThreshold);
  }

  Scalar pointCost(Tensor tensor) {
    Scalar distance = occupancyMap.getL2DistToClosest(tensor);
    // commented code below uses linear interpolation, resulting in closer encounters
    // if (clip.isInside(distance)) {
    // Scalar rescale = RealScalar.ONE.subtract(clip.rescale(distance));
    // return rescale.multiply(penaltyFactor);
    // }
    // return penaltyFactor.zero();
    // TODO YN risk for division by zero
    // ... under() is not recommended for the application layer and can't prevent that :-)
    return Scalars.lessThan(distThreshold, distance) ? RealScalar.ZERO : penaltyFactor.divide(distance);
  }

  @Override // from CostIncrementFunction
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    // Tensor dts = Trajectories.deltaTimes(glcNode, trajectory);
    // Tensor cost = Tensor.of(trajectory.stream() //
    // .map(StateTime::state) //
    // .map(this::pointCost));
    // return cost.dot(dts).Get();
    StateTime stateTime = Lists.getLast(trajectory);
    return pointCost(stateTime.state());
  }

  @Override // from HeuristicFunction
  public Scalar minCostToGoal(Tensor tensor) {
    return RealScalar.ZERO;
  }
}
