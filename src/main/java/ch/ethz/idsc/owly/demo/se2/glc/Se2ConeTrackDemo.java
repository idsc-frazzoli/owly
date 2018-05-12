// code by jph, ynager
package ch.ethz.idsc.owly.demo.se2.glc;

import java.awt.Color;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.Arrays;

import ch.ethz.idsc.owl.glc.adapter.GlcWaypointFollowing;
import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.gui.ren.ArrowHeadRender;
import ch.ethz.idsc.owl.math.region.EllipsoidRegion;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.region.RegionUnion;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class Se2ConeTrackDemo extends Se2CarDemo {
  @Override
  void configure(OwlyAnimationFrame owlyAnimationFrame) {
    CarEntity se2Entity = CarEntity.createDefault(new StateTime(Tensors.vector(7, 9, 0), RealScalar.ZERO));
    ImageRegion imageRegion = R2ImageRegions._SQUARE.imageRegion();
    se2Entity.extraCosts.add(R2ImageRegions._SQUARE.costFunction());
    Region<Tensor> r1 = new EllipsoidRegion(Tensors.vector(8.5, 6.5, 0), Tensors.vector(0.2, 0.2, Double.POSITIVE_INFINITY));
    Region<Tensor> r2 = new EllipsoidRegion(Tensors.vector(4.6, 3.8, 0), Tensors.vector(0.2, 0.2, Double.POSITIVE_INFINITY));
    Region<Tensor> regions = RegionUnion.wrap(Arrays.asList(r1, r2, imageRegion));
    TrajectoryRegionQuery trq = SimpleTrajectoryRegionQuery.timeInvariant(regions);
    se2Entity.obstacleQuery = trq;
    owlyAnimationFrame.set(se2Entity);
    owlyAnimationFrame.setObstacleQuery(trq);
    owlyAnimationFrame.addBackground(RegionRenders.create(imageRegion));
    owlyAnimationFrame.addBackground(RegionRenders.create(r1));
    owlyAnimationFrame.addBackground(RegionRenders.create(r2));
    // {
    // RenderInterface renderInterface = new CameraEmulator( //
    // 48, RealScalar.of(10), se2Entity::getStateTimeNow, trq);
    // owlyAnimationFrame.addBackground(renderInterface);
    // }
    // {
    // RenderInterface renderInterface = new LidarEmulator( //
    // LidarEmulator.DEFAULT, se2Entity::getStateTimeNow, trq);
    // owlyAnimationFrame.addBackground(renderInterface);
    // }
    // define waypoints
    Tensor waypoints = Tensors.of( //
        Tensors.vector(7.0, 9.0, 0), //
        Tensors.vector(8.2, 8.7, -0.75), //
        Tensors.vector(8.5, 7.8, -1.5), //
        Tensors.vector(8.5, 6.5, -1.5), //
        Tensors.vector(8.5, 5.2, -1.5), //
        Tensors.vector(8.2, 4.1, -2.25), //
        Tensors.vector(7.2, 3.8, -3.14), //
        Tensors.vector(5.9, 3.8, -3.14), //
        Tensors.vector(4.6, 3.8, -3.14), //
        Tensors.vector(3.5, 4.1, 2.25), //
        Tensors.vector(3.2, 5.2, 1.5), //
        Tensors.vector(3.2, 6.5, 1.5), //
        Tensors.vector(3.2, 7.8, 1.5), //
        Tensors.vector(3.5, 8.7, 0.75), //
        Tensors.vector(4.5, 9.0, 0), //
        Tensors.vector(5.85, 9.0, 0)).unmodifiable(); //
    // draw waypoints
    RenderInterface renderInterface = new ArrowHeadRender(waypoints, new Color(64, 192, 64, 64));
    owlyAnimationFrame.addBackground(renderInterface);
    // start waypoint following
    GlcWaypointFollowing wpf = new GlcWaypointFollowing( //
        waypoints, se2Entity, owlyAnimationFrame.trajectoryPlannerCallback, trq);
    wpf.setDistanceThreshold(RealScalar.of(0.5));
    wpf.startNonBlocking();
    //
    owlyAnimationFrame.jFrame.addWindowListener(new WindowAdapter() {
      @Override
      public void windowClosed(WindowEvent e) {
        System.out.println("window was closed. terminating...");
        wpf.flagShutdown();
      }
    });
  }

  public static void main(String[] args) {
    new Se2ConeTrackDemo().start();
  }
}