#ifndef GFX_H
#define GFX_H

#include <Math/Quaternionf.h>
#include <Math/Vector3f.h>

namespace utad
{
namespace Gfx
{
void clear(Vector3f c);
void setColor(Vector3f c);
void setPointSize(float s);
void drawPoint(Vector3f x);
void setLineWidth(float w);
void drawLine(Vector3f x0, Vector3f x1);
void drawArrow(Vector3f base, Vector3f vector, float thick);
void drawAxes();

void drawTri(Vector3f x0, Vector3f x1, Vector3f x2);
void drawQuad(Vector3f x0, Vector3f x1, Vector3f x2, Vector3f x3);

// for smooth shading, provide surface normal at each vertex
void drawTri(Vector3f x0, Vector3f x1, Vector3f x2, Vector3f n0, Vector3f n1, Vector3f n2);
void drawQuad(Vector3f x0, Vector3f x1, Vector3f x2, Vector3f x3, Vector3f n0, Vector3f n1, Vector3f n2, Vector3f n3);

// see the two boolean flags at the end:
// draw the surface of the shape, and/or draw some outline edges/curves.
// keep the latter enabled so you can see the rotation of a sphere.
// disable the former when you need to debug collision points.
void drawBox(Vector3f xmin, Vector3f xmax, bool drawSurf = true, bool drawWire = true);
void drawSphere(Vector3f center, float radius, bool drawSurf = true, bool drawWire = true);

void translate(Vector3f x);
void rotate(Quaternionf q);
void pushTransform();
void popTransform();

// to draw a shape in world space, use the following approach:
// pushTransform();
// translate(x);
// rotate(q);
// /* draw shape in body space here */
// popTransform();
}  // namespace Gfx
}  // namespace utad

#endif