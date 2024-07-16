#include "Object3D.h"

bool Sphere::intersect(const Ray &r, float tmin, Hit &h) const
{
    // BEGIN STARTER

    // We provide sphere intersection code for you.
    // You should model other intersection implementations after this one.

    // Locate intersection point ( 2 pts )
    const Vector3f &rayOrigin = r.getOrigin(); //Ray origin in the world coordinate
    const Vector3f &dir = r.getDirection();

    Vector3f origin = rayOrigin - _center;      //Ray origin in the sphere coordinate

    // 直线为rayOrigin + t * dir，代入圆的方程为 [(rayOrigin + t * dir) - _center]^2 = r^2
    float a = dir.absSquared();
    float b = 2 * Vector3f::dot(dir, origin);
    float c = origin.absSquared() - _radius * _radius;

    // no intersection
    if (b * b - 4 * a * c < 0) {
        return false;
    }

    float d = sqrt(b * b - 4 * a * c);

    float tplus = (-b + d) / (2.0f*a);
    float tminus = (-b - d) / (2.0f*a);

    // the two intersections are at the camera back
    if ((tplus < tmin) && (tminus < tmin)) {
        return false;
    }

    float t = 10000;
    // the two intersections are at the camera front
    if (tminus > tmin) {
        t = tminus;
    }

    // one intersection at the front. one at the back 
    if ((tplus > tmin) && (tminus < tmin)) {
        t = tplus;
    }

    if (t < h.getT()) {
        Vector3f normal = r.pointAtParameter(t) - _center;
        normal = normal.normalized();
        h.set(t, this->material, normal);
        return true;
    }
    // END STARTER
    return false;
}

// Add object to group
void Group::addObject(Object3D *obj) {
    m_members.push_back(obj);
}

// Return number of objects in group
int Group::getGroupSize() const {
    return (int)m_members.size();
}

bool Group::intersect(const Ray &r, float tmin, Hit &h) const
{
    // BEGIN STARTER
    // we implemented this for you
    bool hit = false;
    for (Object3D* o : m_members) {
        if (o->intersect(r, tmin, h)) {
            hit = true;
        }
    }
    return hit;
    // END STARTER
}
bool approx(Vector3f& v1, Vector3f& v2){
    const float epsilon = 1e-8;
    return (v1 - v2).absSquared() < epsilon;
}

// Plane::Plane(const Vector3f &normal, float d, Material *m) : Object3D(m) {
//     // implement Plane constructor
// }

bool Plane::intersect(const Ray &r, float tmin, Hit &h) const
{
    // 1. 平行无交点
    if(fabs(Vector3f::dot(_normal, r.getDirection())) < 1e-6)
        return false;
    // 2. t的下限，不能落在摄影机之后
    float t = Vector3f::dot(_dist * _normal - r.getOrigin(), _normal) / Vector3f::dot(_normal, r.getDirection());
    // std::cout << "t is " << t << " while tmin is " << tmin << std::endl;
    if (t < tmin)
    	return false;
    if (t < h.getT()) {
        h.set(t, this->material, _normal);
        return true;
    }
    return false;
}

bool Triangle::intersect(const Ray &r, float tmin, Hit &h) const 
{
    // assert na nb nc 一样
    Vector3f normal = _normals[0];
    // Möller-Trumbore 算法
    Matrix3f A(-r.getDirection(), _v[1] - _v[0], _v[2] - _v[0]);
    Vector3f ans = (A.inverse() * (r.getOrigin() - _v[0]));
    float t = ans[0];
    if(!(ans[0] >= tmin && ans[1] > 0 && ans[1] < 1 && ans[2] > 0 && ans[2] < 1 && ans[1] + ans[2] < 1))
        return false;

    if (t < h.getT()) {
        normal = Matrix3f(_normals[0], _normals[1], _normals[2]) * Vector3f(1-ans[1]-ans[2], ans[1], ans[2]);
        h.set(t, this->material, normal.normalized());
        return true;
    }
    return false;
}


Transform::Transform(const Matrix4f &m,
    Object3D *obj) : _object(obj){
    _m = m;
}
bool Transform::intersect(const Ray &r, float tmin, Hit &h) const
{
    // 光线转到局部坐标系
    Ray rLocal(
        (_m.inverse() * Vector4f(r.getOrigin(), 1)).xyz(), 
        (_m.inverse() * Vector4f(r.getDirection(), 0)).xyz()
    );
    if(_object->intersect(rLocal, tmin, h)){
        // 将h中的局部法向量转回世界坐标系
        h.set(
            h.getT(), 
            h.getMaterial(), 
            (_m.inverse().transposed().getSubmatrix3x3(0, 0) * h.getNormal()).normalized()
        );        
        return true;
    }
    return false;
}
