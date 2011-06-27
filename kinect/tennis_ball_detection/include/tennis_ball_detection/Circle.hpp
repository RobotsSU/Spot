#ifndef __CIRCLE__HPP
#define __CIRCLE__HPP

class Circle {
private:
  int x_, y_, r_, color_, threshold_;
public:
  Circle() {x_ = 0; y_ = 0; r_ = 0; color_ = 0; threshold_ = 0;}
  Circle(int x, int y, int r, int color = 0, int threshold = 0) {x_ = x; y_ = y; r_ = r; color_ = color; threshold_ = threshold;}
  void setValues(int x, int y, int r, int color = 0, int threshold = 0) {x_ = x; y_ = y; r_ = r; color_ = color; threshold_ = threshold;}
  int getCenterX() const {return x_;}
  int getCenterY() const {return y_;}
  int getRadius() const {return r_;}
  int getColor() const {return color_;}
  int getThreshold() const {return threshold_;}
};

#endif
