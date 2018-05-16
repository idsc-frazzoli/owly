// code by ynager
package ch.ethz.idsc.owly.demo.se2.glc;

import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.mapping.BayesianOccupancyGrid;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.pdf.NormalDistribution;
import ch.ethz.idsc.tensor.pdf.RandomVariate;

public class Se2BayesianOccupancyGridDemo extends Se2CarDemo {
  boolean isLaunched = true;

  // TODO incomplete
  @Override
  void configure(OwlyAnimationFrame owlyAnimationFrame) {
    CarEntity se2Entity = CarEntity.createDefault(new StateTime(Tensors.vector(2, 4, 0), RealScalar.ZERO));
    // create occupancy gird
    Tensor lbounds = Tensors.vector(0, 0); // initial lower bounds of grid
    Tensor range = Tensors.vector(10, 10); // size of grid in coordinate space
    Scalar cellDim = DoubleScalar.of(0.25); // size of single cell in coordinate space
    BayesianOccupancyGrid om = BayesianOccupancyGrid.of(lbounds, range, cellDim);
    om.setObstacleRadius(DoubleScalar.of(0.25)); // cells within this radius around occupied cells become also occupied
    //
    TrajectoryRegionQuery trq = createCarQuery(om); // createCarQuery assumes om is timeInvariant
    se2Entity.obstacleQuery = trq;
    owlyAnimationFrame.set(se2Entity);
    owlyAnimationFrame.addBackground(om);
    owlyAnimationFrame.setObstacleQuery(trq);
    // ---
    //
    owlyAnimationFrame.jFrame.addWindowListener(new WindowAdapter() {
      @Override
      public void windowClosed(WindowEvent e) {
        isLaunched = false;
      }
    });
    while (isLaunched) {
      try {
        om.setPose(se2Entity.getStateTimeNow().state(), RealScalar.of(0.99));
        om.processObservation(RandomVariate.of(NormalDistribution.of(1, 0.2), 2), 1);
        om.processObservation(RandomVariate.of(NormalDistribution.of(0, 0.2), 2), 0);
        // om.processObservation(RandomVariate.of(NormalDistribution.of(4, 0.2), 2), 1);
        om.genObstacleMap();
        Thread.sleep(20);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }

  public static void main(String[] args) {
    new Se2BayesianOccupancyGridDemo().start();
  }
}
