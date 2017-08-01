// code by jl
package ch.ethz.idsc.owly.glc.adapter;

import java.util.List;

import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;

//TODO JONAS: enhance functionality:
// maybe possibitly to delete Lists?
public abstract class TrajectoryGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  private final List<Region> goalRegionList;

  public TrajectoryGoalManager(List<Region> goalRegionList) {
    super(new TimeInvariantRegion(RegionUnion.wrap(goalRegionList)));
    this.goalRegionList = goalRegionList;
  }

  public List<Region> getGoalRegionList() {
    return goalRegionList;
  }
}
