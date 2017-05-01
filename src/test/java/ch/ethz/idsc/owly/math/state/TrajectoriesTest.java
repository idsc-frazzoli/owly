package ch.ethz.idsc.owly.math.state;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.ZeroScalar;
import junit.framework.TestCase;

public class TrajectoriesTest extends TestCase {
  public void testDisjoint() {
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new EllipsoidRegion(Tensors.vector(10, 5), Tensors.vector(1, 1))));
    List<StateTime> trajectory = new ArrayList<>();
    trajectory.add(new StateTime(Tensors.vector(0, 5), ZeroScalar.get()));
    trajectory.add(new StateTime(Tensors.vector(5, 5), ZeroScalar.get()));
    assertEquals(goalQuery.firstMember(trajectory), -1);
    assertTrue(goalQuery.isDisjoint(trajectory));
    // ---
    trajectory.add(new StateTime(Tensors.vector(10, 5), ZeroScalar.get()));
    assertEquals(goalQuery.firstMember(trajectory), 2);
    assertFalse(goalQuery.isDisjoint(trajectory));
  }
}
