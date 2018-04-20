// code by jph
package ch.ethz.idsc.owl.mapping;

import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class OccupancyMapTest extends TestCase {
//  public void testGetTile() {
//    Tensor ubounds = Tensors.vector(11, 11);
//    Tensor lbounds = Tensors.vector(-0.5, 0.5);
//    Scalar gridRes = DoubleScalar.of(2.5);
//    OccupancyMap2d om = new OccupancyMap2d(lbounds, ubounds, gridRes);
//    Tensor pos1 = Tensors.vector(-0.5, 0.5);
//    Tensor idx1 = om.toTile(pos1);
//    Tensor trueidx1 = Tensors.vector(0, 0);
//    assertEquals(idx1, trueidx1);
//    Tensor pos2 = Tensors.vector(3, 3.5);
//    Tensor idx2 = om.toTile(pos2);
//    Tensor trueidx2 = Tensors.vector(1, 1);
//    assertEquals(idx2, trueidx2);
//    Tensor pos3 = Tensors.vector(11, 11);
//    Tensor idx3 = om.toTile(pos3);
//    Tensor trueidx3 = Tensors.vector(4, 4);
//    assertEquals(idx3, trueidx3);
//  }

  public void testOccupancyMap() {
    Tensor lbounds = Tensors.vector(-5, -10);
    Tensor ubounds = Tensors.vector(5, 10);
    Scalar gridRes = DoubleScalar.of(0.5);
    OccupancyMap2d om = new OccupancyMap2d(lbounds, ubounds, gridRes);
    //
    Scalar dist;
    Tensor originTileCoord = Tensors.vector(0.5, 0.5); // tile (0,0)
    Tensor entry1 = Tensors.vector(2.5, 0.5); // inside tile (2,0)
    Tensor entry2 = Tensors.vector(0.5, 1.5); // inside tile (0,1)
    Tensor entry3 = Tensors.vector(0.5, 0.5); // inside tile (0,0)
    //
    dist = om.getL2DistToClosest(originTileCoord);
    assertEquals(dist, DoubleScalar.POSITIVE_INFINITY);
    // check entry 1
    om.insert(entry1);
    dist = om.getL2DistToClosest(originTileCoord);
    assertEquals(dist.get(), RealScalar.of(2));
    // check entry 2
    om.insert(entry2);
    dist = om.getL2DistToClosest(originTileCoord);
    assertEquals(dist.get(), RealScalar.ONE);
    // check entry 3
    om.insert(entry3);
    dist = om.getL2DistToClosest(originTileCoord);
    assertEquals(dist.get(), RealScalar.ZERO);
    
  }
}