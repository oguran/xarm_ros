#ifndef __VECTOR3_H__
#define __VECTOR3_H__

namespace xarm6_demo_app1 {

class Vector3 {
public:
  float x;
  float y;
  float z;
  Vector3(float x, float y, float z): x(x), y(y), z(z) {}
};

} // namespace xarm6_demo_app1

#endif // __VECTOR3_H__
