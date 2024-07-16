#ifndef RENDERER_H
#define RENDERER_H

#include <string>

#include "SceneParser.h"
#include "ArgParser.h"

class Hit;
class Vector3f;
class Ray;

class Renderer
{
  public:
    // Instantiates a renderer for the given scene.
    Renderer(const ArgParser &args);
    void Render();
    // Vector3f generate_and_trace(Camera* cam, Hit& h, float ndcx, float ndcy, float wbase, float hbase);
  private:
    Vector3f traceRay(const Ray &ray, float tmin, int bounces, 
                      Hit &hit) const;

    ArgParser _args;
    SceneParser _scene;
};

#endif // RENDERER_H

