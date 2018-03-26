// code by jph, ynager
package ch.ethz.idsc.owly.demo.rn.glc;

import java.awt.Color;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import ch.ethz.idsc.owl.glc.adapter.GlcWaypointFollowing;
import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.gui.ren.PointRender;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.rn.R2ImageRegionWrap;
import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class R2GlcTrackDemo implements DemoInterface {
  @Override
  public void start() {
    try {
      OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
      owlyAnimationFrame.configCoordinateOffset(50, 700);
      owlyAnimationFrame.jFrame.setBounds(100, 50, 1200, 800);
      owlyAnimationFrame.jFrame.setVisible(true);
      configure(owlyAnimationFrame);
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  void configure(OwlyAnimationFrame owlyAnimationFrame) {
    R2ImageRegionWrap r2ImageRegionWrap = R2ImageRegions._EIGHT;
    R2Entity r2Entity = new R2Entity(Tensors.vector(5.5, 6.3));
    // CarEntity se2Entity = CarEntity.createDefault(new StateTime(Tensors.vector(6, 6, 1), RealScalar.ZERO));
    r2Entity.extraCosts.add(r2ImageRegionWrap.costFunction());
    ImageRegion imageRegion = r2ImageRegionWrap.imageRegion();
    TrajectoryRegionQuery trq = SimpleTrajectoryRegionQuery.timeInvariant(imageRegion);
    // r2Entity.obstacleQuery = trq;
    // TrajectoryRegionQuery ray = SimpleTrajectoryRegionQuery.timeInvariant(imageRegion);
    owlyAnimationFrame.set(r2Entity);
    owlyAnimationFrame.setObstacleQuery(trq);
    owlyAnimationFrame.addBackground(RegionRenders.create(imageRegion));
    // define waypoints
    Tensor waypoints = Tensors.of( //
        Tensors.vector(5.5, 6.3), //
        Tensors.vector(7.8, 8.5), //
        Tensors.vector(10, 6.1), //
        Tensors.vector(7.9, 3.8), //
        Tensors.vector(5.5, 6.3), //
        Tensors.vector(3.4, 8.4), //
        Tensors.vector(1.8, 6.4), //
        Tensors.vector(3.5, 4)).unmodifiable(); //
    // draw waypoints
    RenderInterface renderInterface = new PointRender(waypoints, 0.07, new Color(64, 192, 64, 64));
    owlyAnimationFrame.addBackground(renderInterface);
    // start waypoint following
    GlcWaypointFollowing wpf = new GlcWaypointFollowing( //
        waypoints, r2Entity, owlyAnimationFrame.trajectoryPlannerCallback, trq);
    wpf.setDistanceThreshold(RealScalar.of(1));
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
    new R2GlcTrackDemo().start();
  }
}