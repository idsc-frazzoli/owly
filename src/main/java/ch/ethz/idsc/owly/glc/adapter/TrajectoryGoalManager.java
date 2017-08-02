// code by jl
package ch.ethz.idsc.owly.glc.adapter;

import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;

//TODO JONAS: enhance functionality:
// maybe possibitly to delete Lists?
public abstract class TrajectoryGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  private final List<Region> goalRegionList;

  public TrajectoryGoalManager(List<Region> goalRegionList) {
    super(new TimeInvariantRegion(RegionUnion.wrap(goalRegionList)));
    this.goalRegionList = goalRegionList;
  }

  public final List<Region> getGoalRegionList() {
    return goalRegionList;
  }

  public final List<Region> deleteRegionsBefore(Optional<StateTime> furthestState) {
    if (furthestState.isPresent()) {
      int deleteIndex = -1;
      int index = goalRegionList.size();
      while (index > 0) {
        index--;
        if (goalRegionList.get(index).isMember(furthestState.get().state())) {
          deleteIndex = index;
          break;
        }
      }
      final int deleteUntilIndex = deleteIndex;
      goalRegionList.removeIf(gr -> goalRegionList.indexOf(gr) < deleteUntilIndex);
    }
    return goalRegionList;
  }
}
