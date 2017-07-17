// code by jl
package ch.ethz.idsc.owly.math.region;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class RegionUnionTest extends TestCase {
  public void testSimple() {
    List<StateTime> stateList = new ArrayList<>();
    List<Region> regionList = new ArrayList<>();
    Tensor radius = Tensors.vector(0.1, 0.1);
    // Goalstates: {0, 0}, {1, 1},{2, 2},{3, 3},{4, 4},{5, 5},{6, 6},{7, 7}
    for (int i = 0; i < 8; i++) {
      Tensor goal = Tensors.of(RealScalar.of(1 * i), RealScalar.of(1 * i));
      stateList.add(new StateTime(goal, RealScalar.ZERO));
      regionList.add(new EllipsoidRegion(goal, radius));
    }
    Region region = new RegionUnion(regionList);
    for (int i = 0; i < 8; i++) {
      Tensor testState = Tensors.of(RealScalar.of(1 * i), RealScalar.of(1 * i));
      assertTrue(region.isMember(testState));
      assertTrue(region.isMember(testState.add(Tensors.vector(0.05, 0.05))));
      assertTrue(region.isMember(testState.add(Tensors.vector(-0.05, -0.05))));
      assertFalse(region.isMember(testState.add(Tensors.vector(0.15, 0.05))));
      assertFalse(region.isMember(testState.add(Tensors.vector(0.05, 0.15))));
      assertFalse(region.isMember(testState.add(Tensors.vector(-0.15, -0.15))));
    }
    assertFalse(region.isMember(Tensors.vector(4, 7)));
    assertFalse(region.isMember(Tensors.vector(-9, 1)));
    assertFalse(region.isMember(Tensors.vector(3, 0)));
  }

  public void TestSimple2() {
    List<Region> regionList = new ArrayList<>();
    Region region = new HyperplaneRegion(Tensors.vector(-1, 0), RealScalar.ZERO); // right halfplane going through {0,0}: x>0
    assertTrue(region.isMember(Tensors.vector(1, 1)));
    assertFalse(region.isMember(Tensors.vector(-1, 1)));
    assertFalse(region.isMember(Tensors.vector(-1, -1)));
    assertTrue(region.isMember(Tensors.vector(1, -1)));
    regionList.add(region);
    region = new HyperplaneRegion(Tensors.vector(0, -1), RealScalar.ZERO); // upper halfplane going through {0,0} y>0
    assertTrue(region.isMember(Tensors.vector(1, 1)));
    assertTrue(region.isMember(Tensors.vector(-1, 1)));
    assertFalse(region.isMember(Tensors.vector(-1, -1)));
    assertFalse(region.isMember(Tensors.vector(1, -1)));
    regionList.add(region);
    Region regionUnion = new RegionUnion(regionList);
    assertTrue(regionUnion.isMember(Tensors.vector(1, 1)));
    assertTrue(regionUnion.isMember(Tensors.vector(-1, 1)));
    assertFalse(regionUnion.isMember(Tensors.vector(-1, -1)));
    assertTrue(regionUnion.isMember(Tensors.vector(1, -1)));
    regionList.remove(region);
    assertTrue(regionUnion.isMember(Tensors.vector(1, 1)));
    assertTrue(regionUnion.isMember(Tensors.vector(-1, 1)));
    assertFalse(regionUnion.isMember(Tensors.vector(-1, -1)));
    assertFalse(regionUnion.isMember(Tensors.vector(1, -1)));
  }
}
