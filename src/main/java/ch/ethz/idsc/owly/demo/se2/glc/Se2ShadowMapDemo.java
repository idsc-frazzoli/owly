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
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;

public class Se2ShadowMapDemo extends Se2CarDemo {
  private static final float PED_VELOCITY = 0.2f;
  private static final float PED_RADIUS = 0.15f;
  private static final Color PED_COLOR = new Color(23, 12, 200);
  private static final float CAR_VELOCITY = 0.6f;
  private static final float CAR_RADIUS = 0.3f;
  private static final Color CAR_COLOR = new Color(200, 52, 20);

  @Override
  void configure(OwlyAnimationFrame owlyAnimationFrame) {
    CarFlows carFlows = new CarVelocityFlows(Tensors.vector(-0.5, 0.5, 1), Degree.of(60));
    CarEntity se2Entity = new CarEntity(new StateTime(Tensors.vector(4.3, 1.3, 3.14 / 2), RealScalar.ZERO), carFlows); // street_1
    // R2ImageRegionWrap regionWrap = R2ImageRegions._SQUARE;
    ImageRegion imageRegion = //
        ImageRegions.loadFromRepository("/scenarios/street_2.png", Tensors.vector(9, 9), false);
    TrajectoryRegionQuery trq = createCarQuery(imageRegion);
    se2Entity.obstacleQuery = trq;
    // se2Entity.extraCosts.add(regionWrap.costFunction());
    TrajectoryRegionQuery ray = SimpleTrajectoryRegionQuery.timeInvariant(imageRegion);
    owlyAnimationFrame.setObstacleQuery(ray);
    owlyAnimationFrame.addBackground(RegionRenders.create(imageRegion));
    // LIDAR
    LidarEmulator lidarEmulator = new LidarEmulator( //
        LidarEmulator.DEFAULT, se2Entity::getStateTimeNow, ray);
    owlyAnimationFrame.addBackground(lidarEmulator);
    owlyAnimationFrame.set(se2Entity);
    // SHADOWMAP
    ShadowMap shadowMapPed = //
        new ShadowMap(lidarEmulator, imageRegion, se2Entity::getStateTimeNow, PED_VELOCITY, PED_RADIUS);
    shadowMapPed.setColor(PED_COLOR);
    owlyAnimationFrame.addBackground(shadowMapPed);
    shadowMapPed.startNonBlocking(10);
    //
    ShadowMap shadowMapCar = //
        new ShadowMap(lidarEmulator, imageRegion, se2Entity::getStateTimeNow, CAR_VELOCITY, CAR_RADIUS);
    shadowMapCar.setColor(CAR_COLOR);
    owlyAnimationFrame.addBackground(shadowMapCar);
    shadowMapCar.startNonBlocking(10);
    //
    owlyAnimationFrame.jFrame.addWindowListener(new WindowAdapter() {
      @Override
      public void windowClosed(WindowEvent e) {
        System.out.println("window was closed. terminating...");
        shadowMapPed.flagShutdown();
        shadowMapCar.flagShutdown();
      }
    });
  }

  public static void main(String[] args) {
    new Se2ShadowMapDemo().start();
  }
}
