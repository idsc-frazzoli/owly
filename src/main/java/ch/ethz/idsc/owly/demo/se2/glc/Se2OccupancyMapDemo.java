// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.util.Random;
import java.util.Timer;
import java.util.TimerTask;

import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.mapping.OccupancyMap2d;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;

public class Se2OccupancyMapDemo extends Se2CarDemo {
  @Override
  void configure(OwlyAnimationFrame owlyAnimationFrame) {
    CarEntity se2Entity = CarEntity.createDefault(new StateTime(Tensors.vector(6, 6, 1), RealScalar.ZERO));
    ImageRegion imageRegion = R2ImageRegions._SQUARE.imageRegion();
    TrajectoryRegionQuery trq = createCarQuery(imageRegion);
    se2Entity.obstacleQuery = trq;
    //
    OccupancyMap2d om = new OccupancyMap2d(Tensors.vector(0, 0), Tensors.vector(12, 12), DoubleScalar.of(0.5));
    //
    owlyAnimationFrame.set(se2Entity);
    owlyAnimationFrame.setObstacleQuery(trq);
    owlyAnimationFrame.addBackground(RegionRenders.create(imageRegion));
    owlyAnimationFrame.addBackground(om);
    //
    om.insert(Tensors.vector(4, 4));
    om.insert(Tensors.vector(8, 4));
    //
    TimerTask mapUpdate = new TimerTask() {
      @Override
      public void run() {
        Scalar d = om.getL2DistToClosest(se2Entity.getStateTimeNow().state().extract(0, 2));
        System.out.println(se2Entity.getStateTimeNow().state().extract(0, 2) + "   " + d + "\n");
        Random rand = new Random();
        Float ri = rand.nextFloat();
        if(ri < 0.5f) {
          Float rx = rand.nextFloat()*11.8f + 0.1f;
          Float ry = rand.nextFloat()*11.8f + 0.1f;
          om.insert(Tensors.vector(rx, ry));
        }
        
      }
    };
    Timer timer = new Timer("MapUpdateTimer");
    timer.scheduleAtFixedRate(mapUpdate, 10, 1000 / 10);
  }

  public static void main(String[] args) {
    new Se2OccupancyMapDemo().start();
  }
}
