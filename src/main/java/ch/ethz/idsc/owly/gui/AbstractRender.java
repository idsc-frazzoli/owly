// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Graphics2D;

abstract class AbstractRender {
  abstract void render(AbstractLayer abstractLayer, Graphics2D graphics);
}
