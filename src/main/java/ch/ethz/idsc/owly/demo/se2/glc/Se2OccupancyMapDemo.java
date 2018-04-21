// code by ynager
package ch.ethz.idsc.owly.demo.se2.glc;

import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.mapping.OccupancyMap2d;
import ch.ethz.idsc.owl.mapping.OccupancyMapTrajectoryRegionQuery;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.pdf.NormalDistribution;
import ch.ethz.idsc.tensor.pdf.RandomVariate;

public class Se2OccupancyMapDemo extends Se2CarDemo {
  @Override
  void configure(OwlyAnimationFrame owlyAnimationFrame) {
    CarEntity se2Entity = CarEntity.createDefault(new StateTime(Tensors.vector(2, 4, 0), RealScalar.ZERO));
    OccupancyMap2d om = new OccupancyMap2d(Tensors.vector(0, 0), Tensors.vector(8, 8), DoubleScalar.of(0.25));
    TrajectoryRegionQuery omtrq = new OccupancyMapTrajectoryRegionQuery(om, null);
    se2Entity.obstacleQuery = omtrq;
    //
    owlyAnimationFrame.set(se2Entity);
    owlyAnimationFrame.setObstacleQuery(omtrq);
    owlyAnimationFrame.addBackground(om);
    //
    while (true) {
      try {
        om.insert(RandomVariate.of(NormalDistribution.of(6, 0.2), 2));
        om.insert(RandomVariate.of(NormalDistribution.of(1.5, 0.2), 2));
        om.insert(RandomVariate.of(NormalDistribution.of(4, 0.3), 2));
        Thread.sleep(50);
        System.out.printf("TreeSize: " + om.getTreeSize() + ",  Dist: " //
            + "%.3f" + "\n", om.getL2DistToClosest(se2Entity.getStateTimeNow().state()).number().floatValue());
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }

  public static void main(String[] args) {
    new Se2OccupancyMapDemo().start();
  }
}
