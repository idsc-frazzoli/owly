// code by ynager
package ch.ethz.idsc.owly.demo.se2.glc;

import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.mapping.BayesianOccupancyGrid;
import ch.ethz.idsc.owl.math.region.Regions;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.pdf.NormalDistribution;
import ch.ethz.idsc.tensor.pdf.RandomVariate;

public class Se2BayesianOccupancyGridDemo extends Se2CarDemo {
  boolean isLaunched = true;

  // TODO incomplete
  @Override
  void configure(OwlyAnimationFrame owlyAnimationFrame) {
    CarEntity se2Entity = CarEntity.createDefault(new StateTime(Tensors.vector(2, 4, 0), RealScalar.ZERO));
    TrajectoryRegionQuery trq = createCarQuery(Regions.emptyRegion());
    se2Entity.obstacleQuery = trq;    
    // create occupancy gird
    BayesianOccupancyGrid om = new BayesianOccupancyGrid(Tensors.vector(0, 0), Tensors.vector(8, 8), DoubleScalar.of(0.15));
    om.setObstacleRadius(DoubleScalar.of(0.25));
    //
    owlyAnimationFrame.set(se2Entity);
    owlyAnimationFrame.addBackground(om);
    owlyAnimationFrame.setObstacleQuery(trq);
    //
    owlyAnimationFrame.jFrame.addWindowListener(new WindowAdapter() {
      @Override
      public void windowClosed(WindowEvent e) {
        isLaunched = false;
      }
    });
    while (isLaunched) {
      try {
        om.processObservation(RandomVariate.of(NormalDistribution.of(4, 0.2), 2), 1);
        om.processObservation(RandomVariate.of(NormalDistribution.of(2, 0.2), 2), 0);
        Thread.sleep(10);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }

  public static void main(String[] args) {
    new Se2BayesianOccupancyGridDemo().start();
  }
}
