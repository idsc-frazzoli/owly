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
import ch.ethz.idsc.owl.math.region.ImageArea;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.sim.LidarEmulator;
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.mat.DiagonalMatrix;
import ch.ethz.idsc.tensor.mat.IdentityMatrix;
import ch.ethz.idsc.tensor.mat.Inverse;

public class ShadowMap implements RenderInterface {
  //
  private final int resolution = 100;
  private final Tensor shadowMapTransform = Tensors.matrix(new Number[][] //
  { { resolution, 0, 0 }, { 0, -resolution, 0 }, { 0, 0, 1 } }).unmodifiable();
  private final int updateRate = 10; //[Hz]
  private final LidarEmulator lidar;
  private final Supplier<StateTime> stateTimeSupplier;
  private final Rectangle2D rInit;
  private static final Color COLOR_SHADDOW_FILL = new Color(255, 50, 74, 16);
  private static final Color COLOR_SHADDOW_DRAW = new Color(255, 50, 74, 64);
  private final Area obstacleArea;
  private Area shadowArea;
  private Timer increaserTimer;
  private boolean isPaused = false;
  private float strokeVelocity = 0;

  public ShadowMap(LidarEmulator lidar, ImageRegion imageRegion, Supplier<StateTime> stateTimeSupplier) {
    this.lidar = lidar;
    this.stateTimeSupplier = stateTimeSupplier;
    BufferedImage bufferedImage = RegionRenders.image(imageRegion.image());
    ImageArea imageArea = new ImageArea(bufferedImage, new Color(244, 244, 244), 5);
    //
    // convert imageRegion into Area
    Tensor scale = imageRegion.scale();
    Tensor invsc = DiagonalMatrix.of( //
        scale.Get(0).reciprocal().number().doubleValue(), //
        -scale.Get(1).reciprocal().number().doubleValue(), 1);
    Tensor translate = IdentityMatrix.of(3);
    translate.set(RealScalar.of(-bufferedImage.getHeight()), 1, 2);
    Tensor tmatrix = shadowMapTransform.dot(invsc).dot(translate);
    obstacleArea = imageArea.createTransformedArea(AffineTransforms.toAffineTransform(tmatrix));
    //
    // define initial shadow area
    this.rInit = new Rectangle2D.Double();
    this.rInit.setFrame(obstacleArea.getBounds());
    this.shadowArea = new Area(rInit);
    Stroke stroke = new BasicStroke(4, BasicStroke.CAP_ROUND, BasicStroke.JOIN_BEVEL);
    Shape strokeShape = stroke.createStrokedShape(obstacleArea);
    obstacleArea.add(new Area(strokeShape));
    //
    // subtract obstacles from shadow area
    shadowArea.subtract(obstacleArea);
    //
    strokeVelocity = velocityToStroke(1);
  }

  public void updateMap() {
    GeometricLayer geom = new GeometricLayer(shadowMapTransform, Tensors.vectorInt(0, 0, 0));
    Se2Bijection se2Bijection = new Se2Bijection(stateTimeSupplier.get().state());
    geom.pushMatrix(se2Bijection.forward_se2());
    Path2D lidarPath2D = geom.toPath2D(lidar.getPolygon());
    // subtract current lidar measurement from shadow area
    shadowArea.subtract(new Area(lidarPath2D));
    // inflate shadow area
    // TODO: calculate stroke based on resolution, v_max and updateRate
    Stroke stroke = new BasicStroke(strokeVelocity, BasicStroke.CAP_ROUND, BasicStroke.JOIN_BEVEL);
    Shape strokeShape = stroke.createStrokedShape(shadowArea);
    shadowArea.add(new Area(strokeShape));
    shadowArea.subtract(obstacleArea);
  }
  
  private float velocityToStroke(double vMax) {
    float strokePerMeter = resolution/3; //TODO: find a way to get the actual value of this
    return (float) (strokePerMeter*vMax/updateRate);
  }

  public final void startNonBlocking() {
    TimerTask mapUpdate = new TimerTask() {
      public void run() {
        if(!isPaused)
          updateMap();
      }
    };
    increaserTimer = new Timer("MapUpdateTimer");
    increaserTimer.scheduleAtFixedRate(mapUpdate, 10, 1000/updateRate); // call mapUpdate at 10Hz
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
    final Tensor matrix = geometricLayer.getMatrix().dot(Inverse.of(shadowMapTransform));
    Area plotArea = new Area(shadowArea.createTransformedArea(AffineTransforms.toAffineTransform(matrix)));
    graphics.setColor(COLOR_SHADDOW_FILL);
    graphics.fill(plotArea);
    graphics.setColor(COLOR_SHADDOW_DRAW);
    graphics.draw(plotArea);
  }
}
