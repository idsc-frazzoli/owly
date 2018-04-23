// code by ynager
package ch.ethz.idsc.owly.demo.se2;

import java.util.List;

import ch.ethz.idsc.owl.glc.core.CostFunction;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.mapping.OccupancyMap2d;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

/** Cost function that penalizes distance to obstacles */
public final class ObstacleProximityCostFunction implements CostFunction {
  private final Scalar penaltyFactor;
  private final Scalar distThreshold;
  OccupancyMap2d occupancyMap;

  /** @param occupancyMap instance
   * @param penaltyFactor
   * @param distThreshold if distance to obstacle is above threshold cost is zero */
  public ObstacleProximityCostFunction(OccupancyMap2d occupancyMap, Scalar penaltyFactor, Scalar distThreshold) {
    this.penaltyFactor = penaltyFactor;
    this.occupancyMap = occupancyMap;
    this.distThreshold = distThreshold;
  }

  Scalar pointCost(Tensor tensor) {
    Scalar dist = occupancyMap.getL2DistToClosest(tensor);
    return Scalars.lessThan(distThreshold, dist) ? RealScalar.ZERO : dist.under(penaltyFactor);
  }

  @Override // from CostIncrementFunction
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    // Tensor dts = Trajectories.deltaTimes(glcNode, trajectory);
    // Tensor cost = Tensor.of(trajectory.stream() //
    // .map(StateTime::state) //
    // .map(this::pointCost));
    // return cost.dot(dts).Get();
    return pointCost(trajectory.get(trajectory.size() - 1).state());
  }

  @Override // from HeuristicFunction
  public Scalar minCostToGoal(Tensor tensor) {
    return RealScalar.ZERO;
  }
}
