#ifndef __ROTATIONMATRIX_H__
#define __ROTATIONMATRIX_H__

#include <array>
#include <cmath>

#include <xarm6_demo_app1/Vector3.h>

class RotationMatrix {
public:
  std::array<float, 9> elements;
  RotationMatrix(std::array<float, 9> elements): elements(elements) {}
  static RotationMatrix rotationX(float angle) {
    auto c = std::cos(angle);
    auto s = std::sin(angle);
    return RotationMatrix({
        1, 0, 0,
        0, c, s,
        0, -s, c
        });
  }
  static RotationMatrix rotationY(float angle) {
    auto c = std::cos(angle);
    auto s = std::sin(angle);
    return RotationMatrix({
        c, 0, -s,
        0, 1, 0,
        s, 0, c
        });
  }
  static RotationMatrix rotationZ(float angle) {
    auto c = std::cos(angle);
    auto s = std::sin(angle);
    return RotationMatrix({
        c, s, 0,
        -s, c, 0,
        0, 0, 1
        });
  }
  RotationMatrix operator*(const RotationMatrix m) const {
    return RotationMatrix({
        elements[0] * m.elements[0] + elements[3] * m.elements[1] + elements[6] * m.elements[2],
        elements[1] * m.elements[0] + elements[4] * m.elements[1] + elements[7] * m.elements[2],
        elements[2] * m.elements[0] + elements[5] * m.elements[1] + elements[8] * m.elements[2],
        elements[0] * m.elements[3] + elements[3] * m.elements[4] + elements[6] * m.elements[5],
        elements[1] * m.elements[3] + elements[4] * m.elements[4] + elements[7] * m.elements[5],
        elements[2] * m.elements[3] + elements[5] * m.elements[4] + elements[8] * m.elements[5],
        elements[0] * m.elements[6] + elements[3] * m.elements[7] + elements[6] * m.elements[8],
        elements[1] * m.elements[6] + elements[4] * m.elements[7] + elements[7] * m.elements[8],
        elements[2] * m.elements[6] + elements[5] * m.elements[7] + elements[8] * m.elements[8]
        });
  }
  xarm6_demo_app1::Vector3 operator*(const xarm6_demo_app1::Vector3 v) const {
    return xarm6_demo_app1::Vector3(
        elements[0] * v.x + elements[3] * v.y + elements[6] * v.z,
        elements[1] * v.x + elements[4] * v.y + elements[7] * v.z,
        elements[2] * v.x + elements[5] * v.y + elements[8] * v.z
        );
  }
  float& operator[](const size_t index) {
    return elements[index];
  }
  float& at(const size_t row, const size_t column) {
    return elements[row + column * 3];
  }
};

#endif // __ROTATIONMATRIX_H__
