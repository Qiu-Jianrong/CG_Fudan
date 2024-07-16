#include "Material.h"
Vector3f Material::shade(const Ray &ray,
    const Hit &hit,
    const Vector3f &dirToLight,
    const Vector3f &lightIntensity)
{
    Vector3f I_diffuse(0);
    Vector3f I_spectular(0);
    float dotLN = Vector3f::dot(dirToLight, hit.getNormal());
    if(dotLN > 0){
        // 定义三元向量的乘为各分量分别相乘，符合题意
        // 注意这里的L和内积中的L不一样！
        I_diffuse = dotLN * lightIntensity * _diffuseColor;
    }
    // 对称向量为 2 * p - a，p为在法线上的投影
    Vector3f project = Vector3f::dot(-ray.getDirection(), hit.getNormal()) * hit.getNormal();
    float dotLR = Vector3f::dot(2 * project + ray.getDirection(), dirToLight);
    if(dotLR > 0)
        I_spectular = powf(dotLR, _shininess) * lightIntensity * _specularColor;
    return I_diffuse + I_spectular;
}

