// code by ynager
package ch.ethz.idsc.owl.math.map;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.geom.Area;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.util.Timer;
import java.util.TimerTask;
import java.util.function.Supplier;

import ch.ethz.idsc.owl.gui.AffineTransforms;
import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.img.ImageArea;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.sim.LidarEmulator;
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.mat.DiagonalMatrix;
import ch.ethz.idsc.tensor.mat.IdentityMatrix;

public class ShadowMap implements RenderInterface {
  //
  private static final Color COLOR_SHADDOW_FILL = new Color(255, 50, 74, 16);
  private static final Color COLOR_SHADDOW_DRAW = new Color(255, 50, 74, 64);
  // ---
  private final LidarEmulator lidar;
  private final Supplier<StateTime> stateTimeSupplier;
  private boolean isPaused = false;
  private float strokeWidth;
  private final Area obstacleArea;
  private Area shadowArea;
  private Timer increaserTimer;
  private final int updateRate; // [Hz]

  public ShadowMap(LidarEmulator lidar, ImageRegion imageRegion, Supplier<StateTime> stateTimeSupplier, int updateRate) {
    this.lidar = lidar;
    this.stateTimeSupplier = stateTimeSupplier;
    this.updateRate = updateRate;
    BufferedImage bufferedImage = RegionRenders.image(imageRegion.image());
    // TODO 244 and 5 magic const, redundant to values specified elsewhere
    Area area = ImageArea.fromImage(bufferedImage, new Color(244, 244, 244), 5);
    //
    // convert imageRegion into Area
    Tensor scale = imageRegion.scale();
    Tensor invsc = DiagonalMatrix.of( //
        scale.Get(0).reciprocal().number().doubleValue(), //
        -scale.Get(1).reciprocal().number().doubleValue(), 1);
    Tensor translate = IdentityMatrix.of(3);
    translate.set(RealScalar.of(-bufferedImage.getHeight()), 1, 2);
    Tensor tmatrix = invsc.dot(translate);
    obstacleArea = area.createTransformedArea(AffineTransforms.toAffineTransform(tmatrix));
    //
    // define initial shadow area
    Rectangle2D rInit = new Rectangle2D.Double();
    rInit.setFrame(obstacleArea.getBounds());
    this.shadowArea = new Area(rInit);
    Stroke stroke = new BasicStroke(0.1f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_BEVEL);
    Shape strokeShape = stroke.createStrokedShape(obstacleArea);
    obstacleArea.add(new Area(strokeShape));
    //
    // subtract obstacles from shadow area
    shadowArea.subtract(obstacleArea);
    //
    float vMax = 0.2f; // magic const, max pedestrian velocity in [m/s]
    strokeWidth = 2 * vMax / updateRate; // TODO: check if correct
  }

  public void updateMap() {
    Se2Bijection se2Bijection = new Se2Bijection(stateTimeSupplier.get().state());
    GeometricLayer geom = new GeometricLayer(se2Bijection.forward_se2(), Tensors.vectorInt(0, 0, 0));
    Path2D lidarPath2D = geom.toPath2D(lidar.getPolygon());
    // subtract current LIDAR measurement from shadow area
    shadowArea.subtract(new Area(lidarPath2D));
    // dilate shadow area
    Stroke stroke = new BasicStroke(strokeWidth, BasicStroke.CAP_ROUND, BasicStroke.JOIN_BEVEL);
    Shape strokeShape = stroke.createStrokedShape(shadowArea);
    shadowArea.add(new Area(strokeShape));
    shadowArea.subtract(obstacleArea);
  }

  public final void startNonBlocking() {
    TimerTask mapUpdate = new TimerTask() {
      @Override
      public void run() {
        if (!isPaused)
          updateMap();
      }
    };
    increaserTimer = new Timer("MapUpdateTimer");
    increaserTimer.scheduleAtFixedRate(mapUpdate, 10, 1000 / updateRate);
  }

  public final void flagShutdown() {
    increaserTimer.cancel();
  }

  public final void pause() {
    isPaused = true;
  }

  public final void resume() {
    isPaused = false;
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    final Tensor matrix = geometricLayer.getMatrix();
    Area plotArea = new Area(shadowArea.createTransformedArea(AffineTransforms.toAffineTransform(matrix)));
    graphics.setColor(COLOR_SHADDOW_FILL);
    graphics.fill(plotArea);
    graphics.setColor(COLOR_SHADDOW_DRAW);
    graphics.draw(plotArea);
  }
}
