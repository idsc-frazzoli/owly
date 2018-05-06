// code by ynager
package ch.ethz.idsc.owly.demo.se2.glc;

import java.awt.Color;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.img.ImageRegions;
import ch.ethz.idsc.owl.mapping.ShadowMap;
import ch.ethz.idsc.owl.math.Degree;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owl.sim.LidarEmulator;
import ch.ethz.idsc.owly.demo.se2.CarFlows;
import ch.ethz.idsc.owly.demo.se2.CarVelocityFlows;
import ch.ethz.idsc.owly.demo.se2.ShadowConstraint2;
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;

public class Se2ShadowConstraintDemo extends Se2CarDemo {
  private static final float PED_VELOCITY = 0.2f;
  private static final float PED_RADIUS = 0.1f;
  private static final Color PED_COLOR = new Color(23, 12, 200);

  @Override // from Se2CarDemo
  void configure(OwlyAnimationFrame owlyAnimationFrame) {
    CarFlows carFlows = new CarVelocityFlows(Tensors.vector(0.5, 1), Degree.of(60));
    // CarEntity se2Entity = new CarEntity(new StateTime(Tensors.vector(3.5, 0.6, 3.14 / 2), RealScalar.ZERO), carFlows); // street_1
    // CarEntity se2Entity = new CarEntity(new StateTime(Tensors.vector(1.5, 1.0, 3.14 / 2), RealScalar.ZERO), carFlows); // curve_1
    CarEntity se2Entity = new CarEntity(new StateTime(Tensors.vector(3.8, 1.0, 3.14 / 2), RealScalar.ZERO), carFlows); // street_1
    ImageRegion imageRegion = //
        ImageRegions.loadFromRepository("/scenarios/street_2.png", Tensors.vector(9, 9), false);
    TrajectoryRegionQuery trq = createCarQuery(imageRegion);
    se2Entity.obstacleQuery = trq;
    TrajectoryRegionQuery ray = SimpleTrajectoryRegionQuery.timeInvariant(imageRegion);
    owlyAnimationFrame.setObstacleQuery(trq);
    owlyAnimationFrame.addBackground(RegionRenders.create(imageRegion));
    // LIDAR
    LidarEmulator lidarEmulator = new LidarEmulator( //
        LidarEmulator.DEFAULT, se2Entity::getStateTimeNow, ray);
    owlyAnimationFrame.addBackground(lidarEmulator);
    // SHADOWMAP
    ShadowMap shadowMapPed = //
        new ShadowMap(lidarEmulator, imageRegion, se2Entity::getStateTimeNow, PED_VELOCITY, PED_RADIUS);
    shadowMapPed.setColor(PED_COLOR);
    owlyAnimationFrame.addBackground(shadowMapPed);
    shadowMapPed.startNonBlocking(10);
    ShadowConstraint2 shadowConstraintPed = new ShadowConstraint2(shadowMapPed, DoubleScalar.of(3.0), se2Entity.eta());
    se2Entity.extraConstraints.add(shadowConstraintPed);
    //
    owlyAnimationFrame.set(se2Entity);
    owlyAnimationFrame.jFrame.addWindowListener(new WindowAdapter() {
      @Override
      public void windowClosed(WindowEvent e) {
        System.out.println("window was closed. terminating...");
        shadowMapPed.flagShutdown();
      }
    });
  }

  public static void main(String[] args) {
    new Se2ShadowConstraintDemo().start();
  }
}
