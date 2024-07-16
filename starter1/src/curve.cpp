#include "curve.h"
#include "vertexrecorder.h"
using namespace std;

const float c_pi = 3.14159265358979323846f;

namespace
{
// Approximately equal to.  We don't want to use == because of
// precision issues with floating point.
inline bool approx(const Vector3f& lhs, const Vector3f& rhs)
{
	const float eps = 1e-8f;
	return (lhs - rhs).absSquared() < eps;
}


}


Curve evalBezier(const vector< Vector3f >& P, unsigned steps)
{
	// Check
	if (P.size() < 4 || P.size() % 3 != 1)
	{
		cerr << "evalBezier must be called with 3n+1 control points." << endl;
		exit(0);
	}

	Matrix4f M_BEZ(
		1, -3, 3, -1,
		0, 3, -6, 3,
		0, 0, 3, -3,
		0, 0, 0, 1
	);

	// 对于曲线来说则是段数 * step+1个点
	Curve R(steps * (P.size() - 1) / 3 + 1);

	// [p0, p1, p2, p3] 三行四列，底部补充一行形成四维矩阵方便计算，最后去掉结果末位
	Matrix4f Points(
		Vector4f(P[0][0], P[1][0], P[2][0], P[3][0]),
		Vector4f(P[0][1], P[1][1], P[2][1], P[3][1]),
		Vector4f(P[0][2], P[1][2], P[2][2], P[3][2]),
		Vector4f(0, 0, 0, 0),
		false
	);

	Vector4f temp = Points * M_BEZ * Vector4f(1, 0, 0, 0);
	Vector4f temp2 = Points * M_BEZ * Vector4f(0, 1, 0, 0);

	R[0].V = Vector3f(temp[0], temp[1], temp[2]);
	R[0].T = Vector3f(temp2[0], temp2[1], temp2[2]).normalized();
	R[0].N = (Vector3f::cross(Vector3f(0, 0, 1), R[0].T)).normalized();
	R[0].B = (Vector3f::cross(R[0].T, R[0].N)).normalized();

	// 这 (P.size() - 1) / 3 段首尾相连
	for(unsigned i = 0; i < (P.size() - 1) / 3; ++i){
		// P[offset], P[offset + 1] ... P[offset + 3] 作为参照点
		int offset = i * 3;
		Matrix4f Points(
			Vector4f(P[offset][0], P[offset + 1][0], P[offset + 2][0], P[offset + 3][0]),
			Vector4f(P[offset][1], P[offset + 1][1], P[offset + 2][1], P[offset + 3][1]),
			Vector4f(P[offset][2], P[offset + 1][2], P[offset + 2][2], P[offset + 3][2]),
			Vector4f(0, 0, 0, 0),
			false
		);
		for(unsigned j = 1; j <= steps; ++j){
			int index = steps * i + j;
			float t = 1.0 * j / steps;// 下意识写成int可还行
			Vector4f temp = Points * M_BEZ * Vector4f(1, t, t * t, t * t * t);
			Vector4f temp2 = Points * M_BEZ * Vector4f(0, 1, 2 * t, 3 * t * t);
			R[index].V = Vector3f(temp[0], temp[1], temp[2]);
			R[index].T = Vector3f(temp2[0], temp2[1], temp2[2]).normalized();
			R[index].N = (Vector3f::cross(R[index - 1].B, R[index].T)).normalized();
			R[index].B = (Vector3f::cross(R[index].T, R[index].N)).normalized();
		}
	}
	return R;
}

Curve evalBspline(const vector< Vector3f >& P, unsigned steps)
{
	// Check
	if (P.size() < 4)
	{
		cerr << "evalBspline must be called with 4 or more control points." << endl;
		exit(0);
	}

	Matrix4f Bsp(
		1, -3, 3, -1, 
		4, 0, -6, 3,
		1, 3, 3, -3,
		0, 0, 0, 1	
	);
	Bsp = Bsp * (1.0 / 6);
	// TODO:
	// It is suggested that you implement this function by changing
	// basis from B-spline to Bezier.  That way, you can just call
	// your evalBezier function.
	Curve R((P.size() - 3) * steps + 1);

	Matrix4f Points(
		Vector4f(P[0][0], P[1][0], P[2][0], P[3][0]),
		Vector4f(P[0][1], P[1][1], P[2][1], P[3][1]),
		Vector4f(P[0][2], P[1][2], P[2][2], P[3][2]),
		Vector4f(0, 0, 0, 0),
		false
	);

	Vector4f temp = Points * Bsp * Vector4f(1, 0, 0, 0);
	Vector4f temp2 = Points * Bsp * Vector4f(0, 1, 0, 0);

	R[0].V = Vector3f(temp[0], temp[1], temp[2]);
	R[0].T = Vector3f(temp2[0], temp2[1], temp2[2]).normalized();
	R[0].N = (Vector3f::cross(Vector3f(0, 0, 1), R[0].T)).normalized();
	R[0].B = (Vector3f::cross(R[0].T, R[0].N)).normalized();

	for(unsigned i = 3; i < P.size(); ++i){
		Matrix4f Points(
			Vector4f(P[i - 3][0], P[i - 2][0], P[i - 1][0], P[i][0]),
			Vector4f(P[i - 3][1], P[i - 2][1], P[i - 1][1], P[i][1]),
			Vector4f(P[i - 3][2], P[i - 2][2], P[i - 1][2], P[i][2]),
			Vector4f(0, 0, 0, 0),
			false
		);
		for(unsigned j = 1; j <= steps; ++j){
			int index = (i - 3) * steps + j;
			float t = 1.0 * j / steps;
			Vector4f temp = Points * Bsp * Vector4f(1, t, t * t, t * t * t);
			Vector4f temp2 = Points * Bsp * Vector4f(0, 1, 2 * t, 3 * t * t);
			R[index].V = Vector3f(temp[0], temp[1], temp[2]);
			R[index].T = Vector3f(temp2[0], temp2[1], temp2[2]).normalized();
			R[index].N = (Vector3f::cross(R[index - 1].B, R[index].T)).normalized();
			R[index].B = (Vector3f::cross(R[index].T, R[index].N)).normalized();
		}
	}
	return R;
}

Curve evalCircle(float radius, unsigned steps)
{
	// This is a sample function on how to properly initialize a Curve
	// (which is a vector< CurvePoint >).

	// Preallocate a curve with steps+1 CurvePoints
	Curve R(steps + 1);

	// Fill it in counterclockwise
	for (unsigned i = 0; i <= steps; ++i)
	{
		// step from 0 to 2pi
		float t = 2.0f * c_pi * float(i) / steps;

		// Initialize position
		// We're pivoting counterclockwise around the y-axis
		R[i].V = radius * Vector3f(cos(t), sin(t), 0);

		// Tangent vector is first derivative
		R[i].T = Vector3f(-sin(t), cos(t), 0);

		// Normal vector is second derivative
		R[i].N = Vector3f(-cos(t), -sin(t), 0);

		// Finally, binormal is facing up.
		R[i].B = Vector3f(0, 0, 1);
	}

	return R;
}

void recordCurve(const Curve& curve, VertexRecorder* recorder)
{
	const Vector3f WHITE(1, 1, 1);
	for (int i = 0; i < (int)curve.size() - 1; ++i)
	{
		recorder->record_poscolor(curve[i].V, WHITE);
		recorder->record_poscolor(curve[i + 1].V, WHITE);
	}
}
void recordCurveFrames(const Curve& curve, VertexRecorder* recorder, float framesize)
{
	Matrix4f T;
	const Vector3f RED(1, 0, 0);
	const Vector3f GREEN(0, 1, 0);
	const Vector3f BLUE(0, 0, 1);
	
	const Vector4f ORGN(0, 0, 0, 1);
	const Vector4f AXISX(framesize, 0, 0, 1);
	const Vector4f AXISY(0, framesize, 0, 1);
	const Vector4f AXISZ(0, 0, framesize, 1);

	for (int i = 0; i < (int)curve.size(); ++i)
	{
		T.setCol(0, Vector4f(curve[i].N, 0));
		T.setCol(1, Vector4f(curve[i].B, 0));
		T.setCol(2, Vector4f(curve[i].T, 0));
		T.setCol(3, Vector4f(curve[i].V, 1));
 
		// Transform orthogonal frames into model space
		Vector4f MORGN  = T * ORGN;
		Vector4f MAXISX = T * AXISX;
		Vector4f MAXISY = T * AXISY;
		Vector4f MAXISZ = T * AXISZ;

		// Record in model space
		recorder->record_poscolor(MORGN.xyz(), RED);
		recorder->record_poscolor(MAXISX.xyz(), RED);

		recorder->record_poscolor(MORGN.xyz(), GREEN);
		recorder->record_poscolor(MAXISY.xyz(), GREEN);

		recorder->record_poscolor(MORGN.xyz(), BLUE);
		recorder->record_poscolor(MAXISZ.xyz(), BLUE);
	}
}


