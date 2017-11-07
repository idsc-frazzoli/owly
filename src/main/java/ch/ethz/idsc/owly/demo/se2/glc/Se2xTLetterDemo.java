// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.io.IOException;
import java.util.Arrays;

import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.rn.R2xTEllipsoidStateTimeRegion;
import ch.ethz.idsc.owly.demo.rn.R2xTPolygonStateTimeRegion;
import ch.ethz.idsc.owly.demo.rn.glc.R2xTEllipsoidsAnimationDemo;
import ch.ethz.idsc.owly.demo.se2.Se2PointsVsRegion;
import ch.ethz.idsc.owly.demo.se2.Se2PointsVsRegions;
import ch.ethz.idsc.owly.demo.util.BijectionFamily;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.ExamplePolygons;
import ch.ethz.idsc.owly.demo.util.TranslationFamily;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.ScalarTensorFunction;
import ch.ethz.idsc.owly.math.noise.SimplexContinuousNoise;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Sin;

public class Se2xTLetterDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    Se2xTEntity abstractEntity = new Se2xTEntity(Tensors.vector(6, 6, 1));
    owlyAnimationFrame.set(abstractEntity);
    // ---
    switch (3) {
    case 1: {
      ImageRegion imageRegion = R2ImageRegions.inside_gtob();
      Se2PointsVsRegion se2PointsVsRegion = Se2PointsVsRegions.line(Tensors.vector(.2, .1, 0, -.1), imageRegion);
      TrajectoryRegionQuery trq = SimpleTrajectoryRegionQuery.timeInvariant(se2PointsVsRegion);
      abstractEntity.obstacleQuery = trq;
      owlyAnimationFrame.setObstacleQuery(trq);
      owlyAnimationFrame.addRegionRender(imageRegion);
      break;
    }
    case 2: {
      BijectionFamily shift = new TranslationFamily( //
          scalar -> Tensors.of(Sin.FUNCTION.apply(scalar.multiply(RealScalar.of(0.2))), RealScalar.ZERO));
      Region<StateTime> region = new R2xTPolygonStateTimeRegion( //
          ExamplePolygons.CORNER_TOP_LEFT, shift, () -> abstractEntity.getStateTimeNow().time());
      abstractEntity.obstacleQuery = new SimpleTrajectoryRegionQuery(region);
      owlyAnimationFrame.setObstacleQuery(abstractEntity.obstacleQuery);
      // owlyAnimationFrame.addRegionRender(imageRegion);
      owlyAnimationFrame.addBackground((RenderInterface) region);
      break;
    }
    case 3: {
      ImageRegion imageRegion = R2ImageRegions.inside_gtob();
      Se2PointsVsRegion se2PointsVsRegion = Se2PointsVsRegions.line(Tensors.vector(.2, .1, 0, -.1), imageRegion);
      // ---
      ScalarTensorFunction stf1 = R2xTEllipsoidsAnimationDemo.wrap1DTensor(SimplexContinuousNoise.FUNCTION, Tensors.vector(0, 2), 0.03, 6.3);
      BijectionFamily noise1 = new TranslationFamily( //
          s -> Tensors.vector(6, 6).add(stf1.apply(s)));
      Region<StateTime> region1 = new R2xTEllipsoidStateTimeRegion( //
          Tensors.vector(0.6, 0.8), noise1, () -> abstractEntity.getStateTimeNow().time());
      ScalarTensorFunction stf2 = R2xTEllipsoidsAnimationDemo.wrap1DTensor(SimplexContinuousNoise.FUNCTION, Tensors.vector(1, 3), 0.03, 6.3);
      BijectionFamily noise2 = new TranslationFamily( //
          s -> Tensors.vector(6, 6).add(stf2.apply(s)));
      Region<StateTime> region2 = new R2xTEllipsoidStateTimeRegion( //
          Tensors.vector(0.8, 0.6), noise2, () -> abstractEntity.getStateTimeNow().time());
      TrajectoryRegionQuery trq = new SimpleTrajectoryRegionQuery( //
          RegionUnion.wrap(Arrays.asList( //
              new TimeInvariantRegion(se2PointsVsRegion), //
              region1, region2 //
          )));
      abstractEntity.obstacleQuery = trq;
      owlyAnimationFrame.setObstacleQuery(trq);
      owlyAnimationFrame.addRegionRender(imageRegion);
      owlyAnimationFrame.addBackground((RenderInterface) region1);
      owlyAnimationFrame.addBackground((RenderInterface) region2);
      break;
    }
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
