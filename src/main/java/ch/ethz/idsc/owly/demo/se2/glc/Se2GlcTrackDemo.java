// code by jph, ynager
package ch.ethz.idsc.owly.demo.se2.glc;

import java.io.IOException;
import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.adapter.WaypointFollowing;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owl.sim.CameraEmulator;
import ch.ethz.idsc.owl.sim.LidarEmulator;
import ch.ethz.idsc.owly.demo.rn.R2ImageRegionWrap;
import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class Se2GlcTrackDemo extends Se2CarDemo {
  @Override
  void configure(OwlyAnimationFrame owlyAnimationFrame) throws IOException {
    R2ImageRegionWrap r2ImageRegionWrap = R2ImageRegions._EIGHT;
    CarEntity se2Entity = CarEntity.createDefault(new StateTime(Tensors.vector(6, 6, 1), RealScalar.ZERO));
    se2Entity.extraCosts.add(r2ImageRegionWrap.costFunction());
    ImageRegion imageRegion = r2ImageRegionWrap.imageRegion();
    TrajectoryRegionQuery trq = createCarQuery(imageRegion);
    se2Entity.obstacleQuery = trq;
    TrajectoryRegionQuery ray = SimpleTrajectoryRegionQuery.timeInvariant(imageRegion);
    owlyAnimationFrame.set(se2Entity);
    owlyAnimationFrame.setObstacleQuery(trq);
    owlyAnimationFrame.addBackground(RegionRenders.create(imageRegion));
    {
      RenderInterface renderInterface = new CameraEmulator( //
          48, RealScalar.of(10), se2Entity::getStateTimeNow, ray);
      owlyAnimationFrame.addBackground(renderInterface);
    }
    {
      RenderInterface renderInterface = new LidarEmulator( //
          LidarEmulator.DEFAULT, se2Entity::getStateTimeNow, ray);
      owlyAnimationFrame.addBackground(renderInterface);
    }
    //
    // define waypoints
    Tensor waypoints = Tensors.of( //
        Tensors.vector(5.5, 6.3, 1.5), //
        Tensors.vector(7.8, 8.5, 0), //
        Tensors.vector(10, 6.1, -1.5), //
        Tensors.vector(7.9, 3.8, -3.14), //
        Tensors.vector(5.5, 6.3, 1.5), //
        Tensors.vector(3.4, 8.4, -3.14), //
        Tensors.vector(1.8, 6.4, -1.5), //
        Tensors.vector(3.5, 4, 0)); //
    // start waypoint following
    WaypointFollowing wpf = new WaypointFollowing(waypoints, se2Entity, owlyAnimationFrame);
    wpf.setObstacleQuery(trq);
    wpf.setDistanceThreshold(RealScalar.of(1));
    wpf.start();
  }

  public static void main(String[] args) {
    new Se2GlcTrackDemo().start();
  }
}