// code by jph
package ch.ethz.idsc.owly.gui.ani;

// EXPERIMENTAL !!! UNDER DEVELOPMENT !!!
enum SimpleDemo {
  ;
  public static void main(String[] args) {
    OwlyAnimationFrame f = new OwlyAnimationFrame();
    // f.add(new Rice2Entity());
    f.add(new R2Entity());
    f.jFrame.setVisible(true);
  }
}
