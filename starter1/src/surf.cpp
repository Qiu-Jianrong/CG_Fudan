#include "surf.h"
#include "vertexrecorder.h"
using namespace std;

namespace
{
    
    // We're only implenting swept surfaces where the profile curve is
    // flat on the xy-plane.  This is a check function.
    static bool checkFlat(const Curve &profile)
    {
        for (unsigned i=0; i<profile.size(); i++)
            if (profile[i].V[2] != 0.0 ||
                profile[i].T[2] != 0.0 ||
                profile[i].N[2] != 0.0)
                return false;
    
        return true;
    }
}

// DEBUG HELPER
Surface quad() { 
	Surface ret;
	ret.VV.push_back(Vector3f(-1, -1, 0));
	ret.VV.push_back(Vector3f(+1, -1, 0));
	ret.VV.push_back(Vector3f(+1, +1, 0));
	ret.VV.push_back(Vector3f(-1, +1, 0));

	ret.VN.push_back(Vector3f(0, 0, 1));
	ret.VN.push_back(Vector3f(0, 0, 1));
	ret.VN.push_back(Vector3f(0, 0, 1));
	ret.VN.push_back(Vector3f(0, 0, 1));

	ret.VF.push_back(Tup3u(0, 1, 2));
	ret.VF.push_back(Tup3u(0, 2, 3));
	return ret;
}

Surface makeSurfRev(const Curve &profile, unsigned steps)
{
    Surface surface;
	surface = quad();
    const float pi = 3.14159265358979323846f;
    
    if (!checkFlat(profile))
    {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    vector<Vector3f> VV;
    vector< Vector3f > VN;
    vector< Tup3u > VF;
    // 原profile
    for(unsigned i = 0; i < profile.size(); ++i){
        VV.push_back(profile[i].V);
        VN.push_back(-profile[i].N);
    }
    // 旋转矩阵，theta角
    float theta = 2 * pi / steps;
    Matrix4f M = Matrix4f::rotateY(theta);
    for(unsigned i = 0; i < steps; ++i){
        for(unsigned j = 0; j < profile.size(); ++j){
            int offset = i * profile.size() + j;

            // 点坐标
            Vector4f P_dot = M * Vector4f(VV[offset], 0);
            VV.push_back(P_dot.xyz());

            // 法向量
            Vector4f N_dot = M.inverse().transposed() * Vector4f(VN[offset], 0);
            VN.push_back(N_dot.xyz().normalized());

            // 三角形
            if(j < profile.size() - 1)// BAC
                VF.push_back(Tup3u(offset + profile.size(), offset, offset + 1));
            if(j > 0)// DBC
                VF.push_back(Tup3u(offset + profile.size(), offset + profile.size() - 1, offset));
        }
    }

    surface.VF = VF;
    surface.VV = VV;
    surface.VN = VN;
    return surface;
}

void DrawProfile(Matrix4f& M, const Curve &profile, vector<Vector3f>& VV, vector< Vector3f >& VN, vector< Tup3u >& VF, int i){
    for(unsigned j = 0; j < profile.size(); ++j){
        // 点坐标
        Vector4f PDot = M * Vector4f(profile[j].V, 1);
        VV.push_back(PDot.xyz());

        // 法向量
        Vector4f N_dot = M.inverse().transposed() * Vector4f(profile[j].N, 1);
        VN.push_back(-N_dot.xyz().normalized());

        // 三角形，相邻两圈之间的对应点绘制
        if(i != 0){
            unsigned index = (i - 1) * profile.size() + j;
            VF.push_back(Tup3u(
                index, 
                index + 1 == i * profile.size() ? index - j : index + 1, 
                index + profile.size()
            ));
            VF.push_back(Tup3u(
                index, 
                index + profile.size(), 
                index + profile.size() - 1 == i * profile.size() - 1 ? (i + 1) * profile.size() - 1 : index + profile.size() - 1
            ));
        }
    }
}

bool approx(const Vector3f& lhs, const Vector3f& rhs)
{
	const float eps = 1e-8f;
	return (lhs - rhs).absSquared() < eps;
}

Surface makeGenCyl(const Curve &profile, const Curve &sweep )
{
    Surface surface;
	surface = quad();

    if (!checkFlat(profile))
    {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    vector<Vector3f> VV;
    vector< Vector3f > VN;
    vector< Tup3u > VF;

    // 是否存在曲面不闭合问题
    bool inCont = approx(sweep[0].T, sweep[sweep.size() - 1].T) && !approx(sweep[0].N, sweep[sweep.size() - 1].N);

    // 沿着sweep拷贝profile
    for(unsigned i = 0; i < sweep.size(); ++i){
        CurvePoint inter(sweep[i]);
        if(inCont){
            float theta = -acosf(Vector3f::dot(sweep[0].N, sweep[sweep.size() - 1].N)) / sweep.size() * (i + 1);
            inter.N = cos(theta) * inter.N + sin(theta) * inter.B;
            inter.B = Vector3f::cross(inter.T, inter.N);
        }
        Matrix4f M(
            inter.N[0], inter.B[0], inter.T[0], inter.V[0],
            inter.N[1], inter.B[1], inter.T[1], inter.V[1],
            inter.N[2], inter.B[2], inter.T[2], inter.V[2],
            0, 0, 0, 1
        );
        DrawProfile(M, profile, VV, VN, VF, i);
    }

    surface.VF = VF;
    surface.VV = VV;
    surface.VN = VN;
    return surface;
}

void recordSurface(const Surface &surface, VertexRecorder* recorder) {
	const Vector3f WIRECOLOR(0.4f, 0.4f, 0.4f);
    for (int i=0; i<(int)surface.VF.size(); i++)
    {
		recorder->record(surface.VV[surface.VF[i][0]], surface.VN[surface.VF[i][0]], WIRECOLOR);
		recorder->record(surface.VV[surface.VF[i][1]], surface.VN[surface.VF[i][1]], WIRECOLOR);
		recorder->record(surface.VV[surface.VF[i][2]], surface.VN[surface.VF[i][2]], WIRECOLOR);
    }
}

void recordNormals(const Surface &surface, VertexRecorder* recorder, float len)
{
	const Vector3f NORMALCOLOR(0, 1, 1);
    for (int i=0; i<(int)surface.VV.size(); i++)
    {
		recorder->record_poscolor(surface.VV[i], NORMALCOLOR);
		recorder->record_poscolor(surface.VV[i] + surface.VN[i] * len, NORMALCOLOR);
    }
}

void outputObjFile(ostream &out, const Surface &surface)
{
    
    for (int i=0; i<(int)surface.VV.size(); i++)
        out << "v  "
            << surface.VV[i][0] << " "
            << surface.VV[i][1] << " "
            << surface.VV[i][2] << endl;

    for (int i=0; i<(int)surface.VN.size(); i++)
        out << "vn "
            << surface.VN[i][0] << " "
            << surface.VN[i][1] << " "
            << surface.VN[i][2] << endl;

    out << "vt  0 0 0" << endl;
    
    for (int i=0; i<(int)surface.VF.size(); i++)
    {
        out << "f  ";
        for (unsigned j=0; j<3; j++)
        {
            unsigned a = surface.VF[i][j]+1;
            out << a << "/" << "1" << "/" << a << " ";
        }
        out << endl;
    }
}

