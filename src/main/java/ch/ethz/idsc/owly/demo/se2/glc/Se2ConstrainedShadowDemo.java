// code by ynager
package ch.ethz.idsc.owly.demo.se2.glc;

import java.awt.Color;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.util.Arrays;

import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.gui.region.ImageRender;
import ch.ethz.idsc.owl.img.ImageAlpha;
import ch.ethz.idsc.owl.img.ImageTensors;
import ch.ethz.idsc.owl.mapping.ShadowMap;
import ch.ethz.idsc.owl.math.Degree;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.region.RegionIntersection;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owl.sim.LidarEmulator;
import ch.ethz.idsc.owly.demo.se2.CarFlows;
import ch.ethz.idsc.owly.demo.se2.CarVelocityFlows;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.ImageFormat;
import ch.ethz.idsc.tensor.io.ResourceData;

public class Se2ConstrainedShadowDemo extends Se2CarDemo {
  private static final float PED_VELOCITY = 0.2f;
  private static final float PED_RADIUS = 0.05f;
  private static final Color PED_COLOR = new Color(23, 12, 200);
  private static final float CAR_VELOCITY = 0.6f;
  private static final float CAR_RADIUS = 0.3f;
  private static final Color CAR_COLOR = new Color(200, 52, 20);
  private static final Tensor RANGE = Tensors.vector(5, 10);

  @Override
  void configure(OwlyAnimationFrame owlyAnimationFrame) {
    CarFlows carFlows = new CarVelocityFlows(Tensors.vector(-0.5, 0.5, 1), Degree.of(60));
    CarEntity se2Entity = new CarEntity(new StateTime(Tensors.vector(3.0, 0.3, 3.14 / 2), RealScalar.ZERO), carFlows); // street_1
    // R2ImageRegionWrap regionWrap = R2ImageRegions._SQUARE;
    Tensor image = ResourceData.of("/scenarios/multiarea.png");
    BufferedImage bufferedImage = ImageFormat.of(image);
    bufferedImage = ImageAlpha.scale(bufferedImage, 0.3);
    ImageRender imgRender = new ImageRender(bufferedImage, RANGE);
    Tensor imageCar = ImageTensors.reduceInverted(image, 1);
    Tensor imagePed = ImageTensors.reduceInverted(image, 2);
    ImageRegion imageRegionCar = new ImageRegion(imageCar, RANGE, false);
    ImageRegion imageRegionPed = new ImageRegion(imagePed, RANGE, false);
    Region<Tensor> intersectionRegion = RegionIntersection.wrap(Arrays.asList(imageRegionCar, imageRegionPed));
    TrajectoryRegionQuery trq = createCarQuery(imageRegionCar);
    se2Entity.obstacleQuery = trq;
    TrajectoryRegionQuery ray = SimpleTrajectoryRegionQuery.timeInvariant(imageRegionCar);
    TrajectoryRegionQuery rayComp = SimpleTrajectoryRegionQuery.timeInvariant(intersectionRegion);
    // imgRender.scaleAlpha(0.3f);
    owlyAnimationFrame.addBackground(imgRender);
    owlyAnimationFrame.setObstacleQuery(ray);
    // LIDAR
    LidarEmulator lidarEmulator = new LidarEmulator( //
        LidarEmulator.DEFAULT, se2Entity::getStateTimeNow, rayComp);
    owlyAnimationFrame.addBackground(lidarEmulator);
    owlyAnimationFrame.set(se2Entity);
    // SHADOWMAP
    ShadowMap shadowMapPed = //
        new ShadowMap(lidarEmulator, imageRegionPed, se2Entity::getStateTimeNow, PED_VELOCITY, PED_RADIUS);
    shadowMapPed.setColor(PED_COLOR);
    owlyAnimationFrame.addBackground(shadowMapPed);
    shadowMapPed.startNonBlocking(10);
    //
    ShadowMap shadowMapCar = //
        new ShadowMap(lidarEmulator, imageRegionCar, se2Entity::getStateTimeNow, CAR_VELOCITY, CAR_RADIUS);
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
    new Se2ConstrainedShadowDemo().start();
  }
}
