// code by jph
package ch.ethz.idsc.owly.data;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Point;
import java.awt.image.BufferedImage;
import java.io.IOException;

import javax.imageio.ImageIO;

import ch.ethz.idsc.owly.demo.util.UserHome;

/** creates an image with unicode characters drawn inside */
public class CharImage {
  public static CharImage fillBlack(Dimension dimension) {
    return new CharImage(dimension, Color.BLACK, Color.WHITE);
  }

  public static CharImage fillWhite(Dimension dimension) {
    return new CharImage(dimension, Color.WHITE, Color.BLACK);
  }

  private final BufferedImage bufferedImage;
  private final Graphics graphics;

  private CharImage(Dimension dimension, Color fill, Color draw) {
    bufferedImage = new BufferedImage(dimension.width, dimension.height, BufferedImage.TYPE_BYTE_GRAY);
    graphics = bufferedImage.getGraphics();
    graphics.setColor(fill);
    graphics.fillRect(0, 0, dimension.width, dimension.height);
    graphics.setFont(new Font(Font.DIALOG, Font.PLAIN, dimension.height * 5 / 4));
    graphics.setColor(draw);
  }

  public void setFont(Font font) {
    graphics.setFont(font);
  }

  public void draw(char chr, Point point) {
    graphics.drawString("" + chr, point.x, point.y);
  }

  public BufferedImage getBufferedImage() {
    return bufferedImage;
  }

  // demo
  public static void main(String[] args) throws IOException {
    CharImage charImage = CharImage.fillBlack(new Dimension(256, 256));
    charImage.draw('\u0b36', new Point(0, 240)); // 0b14
    // charImage.draw('\u0b14', new Point(100, 240)); // 0b14
    BufferedImage bufferedImage = charImage.getBufferedImage();
    ImageIO.write(bufferedImage, "png", UserHome.Pictures("letter.png"));
  }
}
