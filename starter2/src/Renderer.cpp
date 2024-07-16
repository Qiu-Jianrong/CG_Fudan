#include "Renderer.h"

#include "ArgParser.h"
#include "Camera.h"
#include "Image.h"
#include "Ray.h"
#include "VecUtils.h"

#include <limits>


Renderer::Renderer(const ArgParser &args) :
    _args(args),
    _scene(args.input_file)
    
{
    srand(time(NULL));
}

Vector3f dowm_sample(Matrix3f& kernal, Image& data){
    Vector3f ans(0);
    for(int i = 0; i < 3; ++i){
        for(int j = 0; j < 3; ++j){
            ans += (kernal(i, j) * data.getPixel(i, j));
        }
    }
    return ans;
}

void fillLocal(Image& global, Image& local, int i, int j){
    for(int _j = 0; _j < 3; ++_j){
        for(int _i = 0; _i < 3; ++_i){
            local.setPixel(_i , _j, global.getPixel(i * 3 + _i, j * 3 + _j));
        }
    }    
}


void
Renderer::Render()
{
    int w = _args.width;
    int h = _args.height;
    // 3倍长宽进行渲染
    if(_args.filter){
        w *= 3;
        h *= 3; 
    }
    Image image(w, h);
    Image nimage(w, h);
    Image dimage(w, h);



    // loop through all the pixels in the image
    // generate all the samples

    // This look generates camera rays and callse traceRay.
    // It also write to the color, normal, and depth images.
    // You should understand what this code does.
    Camera* cam = _scene.getCamera();
    float wbase = 2.0 / (w - 1.0f);
    float hbase = 2.0 / (h - 1.0f);
    for (int y = 0; y < h; ++y) {
        // 将像素坐标转换为标准化坐标 [-1, 1] * [-1, 1]
        // 用于生成从摄像机到场景的光线，这样可以确保生成的光线覆盖整个视野
        float ndcy = 2 * (y / (h - 1.0f)) - 1.0f;
        for (int x = 0; x < w; ++x) {
            float ndcx = 2 * (x / (w - 1.0f)) - 1.0f;
            // Use PerspectiveCamera to generate a ray.
            // You should understand what generateRay() does.
            float random_x = 0;
            float random_y = 0;
            if (_args.jitter){
                random_x = (1.0 * rand()) / RAND_MAX;
                random_y = (1.0 * rand()) / RAND_MAX;                    
            }
            random_x = ndcx + random_x * wbase;
            random_y = ndcy + random_y * hbase;
            
            Ray r = cam->generateRay(Vector2f(random_x, random_y));

            Hit h;
            Vector3f color = traceRay(r, cam->getTMin(), _args.bounces, h);

            // 剩余15次采样取平均
            if(_args.jitter){
                Vector3f avg_normal(h.getNormal());
                float avg_t(h.getT());
                Vector3f avg_color(color);

                for(int i = 1; i < 16; ++i){
                    Hit _h = Hit();
                    random_x = (1.0 * rand()) / RAND_MAX;
                    random_y = (1.0 * rand()) / RAND_MAX;   
                    random_x = ndcx + random_x * wbase;
                    random_y = ndcy + random_y * hbase;
                    
                    Ray r = cam->generateRay(Vector2f(random_x, random_y));
                    Vector3f _color = traceRay(r, cam->getTMin(), _args.bounces, _h);    
                    //_color.print();   

                    avg_normal += _h.getNormal();
                    avg_t += _h.getT();
                    avg_color += _color;              
                }
                h.set(avg_t / 16.0f, h.getMaterial(), avg_normal / 16.0f);
                color = avg_color / 16.0f;
            }


            image.setPixel(x, y, color);
            nimage.setPixel(x, y, (h.getNormal() + 1.0f) / 2.0f);
            float range = (_args.depth_max - _args.depth_min);
            if (range) {
                dimage.setPixel(x, y, Vector3f((h.t - _args.depth_min) / range));
            }
        }
    }

    // 渲染完毕后下采样
    if(_args.filter){
        Image k_image(w / 3, h / 3);
        Image k_nimage(w / 3, h / 3);
        Image k_dimage(w / 3, h / 3);
        Matrix3f kernal(
            1, 2, 1,
            2, 4, 2,
            1, 2, 1
        );
        kernal = kernal * (1.0 / 16);
        for(int j = 0; j < h / 3; ++j){
            for(int i = 0; i < w / 3; ++i){
                Image local(3, 3);
                Image nlocal(3, 3);
                Image dlocal(3, 3);
                // std::cout << "Got point0." << std::endl;
                fillLocal(image, local, i, j);
                fillLocal(nimage, nlocal, i, j);
                fillLocal(dimage, dlocal, i, j);
                // std::cout << "Got point1." << std::endl;
                // Vector3f downSample = dowm_sample(kernal, local);
                k_image.setPixel(i, j, dowm_sample(kernal, local));
                k_nimage.setPixel(i, j, dowm_sample(kernal, nlocal));
                k_dimage.setPixel(i, j, dowm_sample(kernal, dlocal));
                // std::cout << "Got point2." << std::endl;
            }
        }
        image = k_image;
        dimage = k_dimage;
        nimage = k_nimage;
    }


    // END SOLN

    // save the files 
    if (_args.output_file.size()) {
        image.savePNG(_args.output_file);
    }
    if (_args.depth_file.size()) {
        dimage.savePNG(_args.depth_file);
    }
    if (_args.normals_file.size()) {
        nimage.savePNG(_args.normals_file);
    }
}



Vector3f
Renderer::traceRay(const Ray &r,
    float tmin,
    int bounces,
    Hit &h) const
{
    // The starter code only implements basic drawing of sphere primitives.
    // You will implement phong shading, recursive ray tracing, and shadow rays.

    if (_scene.getGroup()->intersect(r, tmin, h)) {
        Vector3f res(_scene.getAmbientLight() * h.getMaterial()->getDiffuseColor());
        for(int i = 0; i < _scene.lights.size(); ++i){
            Light* light = _scene.getLight(i);
            Vector3f Point = r.pointAtParameter(h.getT());
            Vector3f dirToLight(0);
            Vector3f lightIntensity(0);
            float distToLight = 0;
            light->getIllumination(Point, dirToLight, lightIntensity, distToLight);

            // 计算阴影
            if(_args.shadows){
                Ray Inverse(Point, dirToLight);
                Hit InverseHit = Hit();
                if(_scene.getGroup()->intersect(Inverse, 0.0001, InverseHit))
                    continue;
            }

            res += h.getMaterial()->shade(r, h, dirToLight, lightIntensity);
        }
        // 最大反射深度bounces
        if(bounces > 0){
            Vector3f Project = Vector3f::dot(-r.getDirection(), h.getNormal()) * h.getNormal();
            Vector3f R = (r.getDirection() + 2 * Project).normalized();
            Ray right(r.pointAtParameter(h.getT()) + 0.01 * R, R);
            Hit new_hit = Hit();
            res += traceRay(right, 0, bounces - 1, new_hit) * (h.getMaterial()->getSpecularColor());
        }
        return res;
    } else {
	    return _scene.getBackgroundColor(r.getDirection());
    };
}


