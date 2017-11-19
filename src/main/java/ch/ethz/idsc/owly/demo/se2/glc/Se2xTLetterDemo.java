// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.io.IOException;
import java.util.Arrays;

import ch.ethz.idsc.owly.demo.rn.R2ImageRegionWrap;
import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.rn.R2xTEllipsoidStateTimeRegion;
import ch.ethz.idsc.owly.demo.rn.R2xTPolygonStateTimeRegion;
import ch.ethz.idsc.owly.demo.se2.Se2PointsVsRegion;
import ch.ethz.idsc.owly.demo.se2.Se2PointsVsRegions;
import ch.ethz.idsc.owly.demo.se2.twd.CarPolicyEntity;
import ch.ethz.idsc.owly.demo.util.CameraEmulator;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.LidarEmulator;
import ch.ethz.idsc.owly.demo.util.SimpleTranslationFamily;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.gui.region.RegionRenders;
import ch.ethz.idsc.owly.math.r2.CogPoints;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.se2.BijectionFamily;
import ch.ethz.idsc.owly.math.se2.Se2Family;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class Se2xTLetterDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    CarxTEntity carxTEntity = new CarxTEntity(new StateTime(Tensors.vector(6.75, 5.4, 1 + Math.PI), RealScalar.ZERO));
    owlyAnimationFrame.set(carxTEntity);
    // ---
    R2ImageRegionWrap r2ImageRegionWrap = R2ImageRegions._GTOB;
    ImageRegion imageRegion = r2ImageRegionWrap.imageRegion();
    carxTEntity.extraCosts.add(r2ImageRegionWrap.costFunction());
    // ---
    BijectionFamily noise1 = new SimpleTranslationFamily(s -> Tensors.vector( //
        Math.sin(s.number().doubleValue() * .12) * 3.0 + 3.6, 4.0));
    Region<StateTime> region1 = new R2xTEllipsoidStateTimeRegion( //
        Tensors.vector(0.4, 0.5), noise1, () -> carxTEntity.getStateTimeNow().time());
    // ---
    BijectionFamily rigid3 = new Se2Family(s -> Tensors.vector(8.0, 5.8, s.number().doubleValue() * 0.5));
    Tensor polygon = CogPoints.of(3, RealScalar.of(1.0), RealScalar.of(0.3));
    Region<StateTime> cog0 = new R2xTPolygonStateTimeRegion( //
        polygon, rigid3, () -> carxTEntity.getStateTimeNow().time());
    // ---
    Se2PointsVsRegion se2PointsVsRegion = Se2PointsVsRegions.line(Tensors.vector(0.2, 0.1, 0, -0.1), imageRegion);
    TrajectoryRegionQuery trq = new SimpleTrajectoryRegionQuery( //
        RegionUnion.wrap(Arrays.asList( //
            new TimeInvariantRegion(se2PointsVsRegion), // <- expects se2 states
            region1, cog0 //
        )));
    // Se2PointsVsRegion se2PointsVsRegion = Se2PointsVsRegions.line(Tensors.vector(0.2, 0.1, 0, -0.1), RegionUnion.wrap(Arrays.asList( //
    // new TimeInvariantRegion(imageRegion), // <- expects se2 states
    // region1, cog0 //
    // )));
    // TrajectoryRegionQuery trq = new SimpleTrajectoryRegionQuery( //
    // );
    carxTEntity.obstacleQuery = trq;
    // abstractEntity.raytraceQuery = SimpleTrajectoryRegionQuery.timeInvariant(imageRegion);
    owlyAnimationFrame.setObstacleQuery(trq);
    owlyAnimationFrame.addBackground(RegionRenders.create(imageRegion));
    owlyAnimationFrame.addBackground((RenderInterface) region1);
    // owlyAnimationFrame.addBackground((RenderInterface) region2);
    owlyAnimationFrame.addBackground((RenderInterface) cog0);
    // ---
    final TrajectoryRegionQuery ray = new SimpleTrajectoryRegionQuery( //
        RegionUnion.wrap(Arrays.asList( //
            new TimeInvariantRegion(imageRegion), //
            region1,
            // region2,
            cog0 //
        )));
    {
      RenderInterface renderInterface = new CameraEmulator( //
          48, RealScalar.of(10), carxTEntity::getStateTimeNow, ray);
      owlyAnimationFrame.addBackground(renderInterface);
    }
    {
      RenderInterface renderInterface = new LidarEmulator( //
          LidarEmulator.RAYDEMO, carxTEntity::getStateTimeNow, ray);
      owlyAnimationFrame.addBackground(renderInterface);
    }
    {
      StateTime stateTime = new StateTime(Tensors.vector(5.600, 8.667, -1.571), RealScalar.ZERO);
      CarPolicyEntity twdPolicyEntity = new CarPolicyEntity(stateTime, ray);
      owlyAnimationFrame.add(twdPolicyEntity);
    }
    // ---
    owlyAnimationFrame.configCoordinateOffset(50, 700);
    owlyAnimationFrame.jFrame.setBounds(100, 50, 1200, 800);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) throws IOException {
    new Se2xTLetterDemo().start();
  }
}
