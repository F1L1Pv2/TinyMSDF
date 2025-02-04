#include <omp.h>
#include <stdio.h>
#include <malloc.h>
#include <vector>
#include <string>
#include <stdint.h>
#include <limits>
#include "3dmath.h"
#include <algorithm>

using namespace std;

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define IMAGE_WIDTH 128
#define IMAGE_HEIGHT 128
uint32_t* pixels;

double cross(const vec2& a, const vec2& b){
    return a.x*b.y - a.y*b.x;
}

uint32_t blendColors(uint32_t color1, uint32_t color2, float blendFactor) {
    if (blendFactor < 0.0f) blendFactor = 0.0f;
    if (blendFactor > 1.0f) blendFactor = 1.0f;

    uint8_t r1 = (color1 >> 24) & 0xFF;
    uint8_t g1 = (color1 >> 16) & 0xFF;
    uint8_t b1 = (color1 >> 8) & 0xFF;
    uint8_t a1 = color1 & 0xFF;

    uint8_t r2 = (color2 >> 24) & 0xFF;
    uint8_t g2 = (color2 >> 16) & 0xFF;
    uint8_t b2 = (color2 >> 8) & 0xFF;
    uint8_t a2 = color2 & 0xFF;

    uint8_t r = static_cast<uint8_t>(r1 * (1.0f - blendFactor) + r2 * blendFactor);
    uint8_t g = static_cast<uint8_t>(g1 * (1.0f - blendFactor) + g2 * blendFactor);
    uint8_t b = static_cast<uint8_t>(b1 * (1.0f - blendFactor) + b2 * blendFactor);
    uint8_t a = static_cast<uint8_t>(a1 * (1.0f - blendFactor) + a2 * blendFactor);

    uint32_t blendedColor = (r << 24) | (g << 16) | (b << 8) | a;

    return blendedColor;
}

uint8_t extractChannel(uint32_t color, int shift) {
    return (color >> shift) & 0xFF;
}

uint32_t changeAlpha(uint32_t color, float alpha){
    return (color & 0x00FFFFFF) | (((uint32_t)(alpha*255) << 8*3));
}

uint32_t mixColors(uint32_t color1, uint32_t color2) {
    uint8_t alpha1 = extractChannel(color1, 24);
    uint8_t red1   = extractChannel(color1, 16);
    uint8_t green1 = extractChannel(color1, 8);
    uint8_t blue1  = extractChannel(color1, 0);

    uint8_t alpha2 = extractChannel(color2, 24);
    uint8_t red2   = extractChannel(color2, 16);
    uint8_t green2 = extractChannel(color2, 8);
    uint8_t blue2  = extractChannel(color2, 0);

    float alpha1Normalized = alpha1 / 255.0f;
    float alpha2Normalized = alpha2 / 255.0f;

    float resultAlphaNormalized = alpha1Normalized + alpha2Normalized * (1.0f - alpha1Normalized);
    uint8_t resultAlpha = static_cast<uint8_t>(resultAlphaNormalized * 255);

    uint8_t resultRed = static_cast<uint8_t>(
        (red1 * alpha1Normalized + red2 * alpha2Normalized * (1.0f - alpha1Normalized)) / resultAlphaNormalized);
    uint8_t resultGreen = static_cast<uint8_t>(
        (green1 * alpha1Normalized + green2 * alpha2Normalized * (1.0f - alpha1Normalized)) / resultAlphaNormalized);
    uint8_t resultBlue = static_cast<uint8_t>(
        (blue1 * alpha1Normalized + blue2 * alpha2Normalized * (1.0f - alpha1Normalized)) / resultAlphaNormalized);

    uint32_t resultColor = (resultAlpha << 24) | (resultRed << 16) | (resultGreen << 8) | resultBlue;
    return resultColor;
}

void setPixel(int x, int y, uint32_t color) {
    if (x >= 0 && x < IMAGE_WIDTH && y >= 0 && y < IMAGE_HEIGHT) {
        pixels[(IMAGE_HEIGHT - y) * IMAGE_WIDTH + x] = color;
    }
}

void drawCircle(int xc, int yc, int radius, uint32_t color) {
    for (int x = -radius; x <= radius; x++) {
        for (int y = -radius; y <= radius; y++) {
            if (x * x + y * y <= radius * radius) {
                setPixel(xc + x, yc + y, color);
            }
        }
    }
}

struct EdgeLine {
    vec2 start;
    vec2 end;


    vec2 point(double t) const{
        return start + (end - start) * t;
    }

    vec2 derivative(double t) const{
        return end - start;
    }

    double distanceTo(const vec2& point) const {
        vec2 lineVec = end - start;
        vec2 pointVec = point - start;

        double lineLenSquared = lineVec.dot(lineVec);
        double t = std::max(0.0, std::min(1.0, pointVec.dot(lineVec) / lineLenSquared));

        vec2 projection = start + lineVec * t;
        return (point - projection).mag();
    }

    double orthogonality(const vec2& p) const {
        vec2 lineVec = end - start;
        vec2 pointVec = p - start;

        double lineLenSquared = lineVec.dot(lineVec);
        if (lineLenSquared < 1e-10) {
            // The line is degenerate (start and end are the same)
            return 0.0;
        }

        double t = pointVec.dot(lineVec) / lineLenSquared; // t can be any real value
        vec2 closestPoint = point(t);
        vec2 tangent = derivative(t);

        // Handle degenerate tangent vectors
        double tangentMag = tangent.mag();
        if (tangentMag < 1e-10) {
            return 0.0;
        }

        vec2 normal = { -tangent.y, tangent.x }; // Perpendicular to tangent
        normal.normalize();

        vec2 pointToClosest = p - closestPoint;
        double pointToClosestMag = pointToClosest.mag();
        if (pointToClosestMag < 1e-10) {
            return 0.0; // Point is exactly on the line
        }

        // Return the signed cross product (direction matters)
        return cross(tangent / tangentMag, pointToClosest / pointToClosestMag);
    }

    double signedDistanceTo(const vec2& point) const {
        vec2 lineVec = end - start;
        vec2 pointVec = point - start;

        double lineLenSquared = lineVec.dot(lineVec);
        if (lineLenSquared < 1e-10) {
            // The line is degenerate (start and end are the same)
            return (point - start).mag();
        }

        double t = pointVec.dot(lineVec) / lineLenSquared;
        vec2 projection;

        if (t < 0.0) {
            // Closest to the start point
            projection = start;
        } else if (t > 1.0) {
            // Closest to the end point
            projection = end;
        } else {
            // Closest to the line segment
            projection = start + lineVec * t;
        }

        vec2 perp = { -lineVec.y, lineVec.x }; // Perpendicular vector
        perp.normalize();
        double side = (point - projection).dot(perp);

        return side > 0 ? -(point - projection).mag() : (point - projection).mag();
    }

    void draw(uint32_t color, int thickness) {
        int x0 = static_cast<int>(start.x * IMAGE_WIDTH);
        int y0 = static_cast<int>(start.y * IMAGE_HEIGHT);
        int x1 = static_cast<int>(end.x * IMAGE_WIDTH);
        int y1 = static_cast<int>(end.y * IMAGE_HEIGHT);

        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;

        while (true) {
            drawCircle(x0, y0, thickness, color);
            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }
};

struct EdgeBezier {
    vec2 start;
    vec2 end;
    vec2 control;

    vec2 point(double t) const{
        return start + (control - start) * (2 * t) + (end - control * 2 + start) * (t * t);
    }

    vec2 derivative(double t)const{
        return (end - control * 2  + start) * (2*t) + (control - start) * 2;
    }

    double distanceTo(const vec2& p) const {
        double minDistance = std::numeric_limits<double>::max();
        for (int i = 0; i <= 100; ++i) {
            double t = i / 100.0;
            vec2 curvePoint = point(t);
            double distance = (p - curvePoint).mag();
            if (distance < minDistance) {
                minDistance = distance;
            }
        }
        return minDistance;
    }

    double orthogonality(const vec2& p) const {
        double minDistance = std::numeric_limits<double>::max();
        double minT = 0.0;

        // Use a binary search to find an approximate closest point
        const int subdivisions = 20; // Adjust for precision vs performance
        for (int i = 0; i <= subdivisions; ++i) {
            double t = i / (double)subdivisions;
            vec2 curvePoint = point(t);
            double distance = (p - curvePoint).mag();
            if (distance < minDistance) {
                minDistance = distance;
                minT = t;
            }
        }

        // Refine the closest point using a simple iterative method
        const double epsilon = 1e-6;
        for (int i = 0; i < 10; ++i) { // Limit iterations for safety
            vec2 curvePoint = point(minT);
            vec2 tangent = derivative(minT);
            vec2 pointToCurve = p - curvePoint;

            // Project the point-to-curve vector onto the tangent vector
            double projection = pointToCurve.dot(tangent) / tangent.dot(tangent);

            // Update t based on the projection
            minT = minT + projection;

            // Clamp t to the valid range [0, 1]
            minT = std::max(0.0, std::min(1.0, minT));

            // Check for convergence
            if (fabs(projection) < epsilon) break;
        }

        vec2 closestPoint = point(minT);
        vec2 tangent = derivative(minT);

        // Handle degenerate tangent vectors
        double tangentMag = tangent.mag();
        if (tangentMag < 1e-10) {
            return 0.0;
        }

        vec2 normal = { -tangent.y, tangent.x }; // Perpendicular to tangent
        normal.normalize();

        vec2 pointToClosest = p - closestPoint;
        double pointToClosestMag = pointToClosest.mag();
        if (pointToClosestMag < 1e-10) {
            return 0.0; // Point is exactly on the curve
        }

        // Return the signed cross product (direction matters)
        return cross(tangent / tangentMag, pointToClosest / pointToClosestMag);
    }

    double signedDistanceTo(const vec2& p) const {
        double minDistance = std::numeric_limits<double>::max();
        double minT = 0.0;

        // Use more subdivisions for better accuracy
        const int subdivisions = 200;
        for (int i = 0; i <= subdivisions; ++i) {
            double t = i / (double)subdivisions;
            vec2 curvePoint = point(t);
            double distance = (p - curvePoint).mag();
            if (distance < minDistance) {
                minDistance = distance;
                minT = t;
            }
        }

        // Calculate the signed distance
        vec2 closestPoint = point(minT);
        vec2 tangent = derivative(minT);
        vec2 normal = { -tangent.y, tangent.x }; // Perpendicular to tangent
        normal.normalize();

        // Signed distance is positive if p is on the "left" side of the curve
        double sign = cross(tangent, p - closestPoint) >= 0 ? -1.0 : 1.0;

        return sign * minDistance;
    }

    void draw(uint32_t color, int thickness) {
        vec2 prev = start;
        for (int i = 1; i <= 100; i++) {
            double t = i / 100.0;
            vec2 current = point(t);
            EdgeLine line = { prev, current };
            line.draw(color, thickness);
            prev = current;
        }
    }
};

struct EdgeType {
    union {
        EdgeLine line;
        EdgeBezier bezier;
    };

    EdgeType() {}
};

enum EDGETYPE {
    EDGETYPENONE,
    EDGETYPELINE,
    EDGETYPEBEZIER
};

struct Edge {
    EDGETYPE type = EDGETYPENONE;
    EdgeType as;
    uint32_t color = 0xFFFFFFFF;
    uint32_t id = 0;
    static uint32_t global_id;

    Edge(EdgeLine item) {
        type = EDGETYPELINE;
        as.line = item;
        id = Edge::global_id;
        Edge::global_id++;
    }

    Edge(EdgeBezier item) {
        type = EDGETYPEBEZIER;
        as.bezier = item;
        id = Edge::global_id;
        Edge::global_id++;
    }

    vec2 point(double t) const{
        switch (type) {
        case EDGETYPELINE: return as.line.point(t);
        case EDGETYPEBEZIER: return as.bezier.point(t);
        default:
            printf("Point: Unknown type\n");
            exit(1);
        }
    }

    vec2 derivative(double t) const{
        switch (type) {
        case EDGETYPELINE: return as.line.derivative(t);
        case EDGETYPEBEZIER: return as.bezier.derivative(t);
        default:
            printf("Derivative: Unknown type\n");
            exit(1);
        }
    }

    void draw(uint32_t color, int thickness) {
        switch (type) {
        case EDGETYPELINE: as.line.draw(color, thickness); break;
        case EDGETYPEBEZIER: as.bezier.draw(color, thickness); break;
        default:
            printf("Draw: Unknown type\n");
            exit(1);
        }
    }

    float distanceTo(const vec2& p) const{
        switch (type) {
        case EDGETYPELINE: return as.line.distanceTo(p); break;
        case EDGETYPEBEZIER: return as.bezier.distanceTo(p); break;
        default:
            printf("DistanceTo: Unknown type\n");
            exit(1);
        }
    }

    float orthogonality(const vec2& p) const{
        switch (type) {
        case EDGETYPELINE: return as.line.orthogonality(p); break;
        case EDGETYPEBEZIER: return as.bezier.orthogonality(p); break;
        default:
            printf("DistanceTo: Unknown type\n");
            exit(1);
        }
    }

    float signedDistanceTo(const vec2& p) const {
        switch (type) {
        case EDGETYPELINE: return as.line.signedDistanceTo(p); break;
        case EDGETYPEBEZIER: return as.bezier.signedDistanceTo(p); break;
        default:
            printf("SignedDistanceTo: Unknown type\n");
            exit(1);
        }
    }
};

uint32_t Edge::global_id = 0;

struct Contour {
    vector<Edge> edges;

    void draw(uint32_t color, int thickness) {
        for (auto& edge : edges) {
            edge.draw(color, thickness);
        }
    }
};

bool AreSame(double a, double b, double epsilon = 1e-10)
{
    return fabs(a - b) < epsilon;
}

bool isCornerSharp(const Edge* a,const Edge* b, double epsilon=PI/20){
    return !(abs(
        cross(
            a->derivative(1)/abs(a->derivative(1).mag()),
            b->derivative(0)/abs(b->derivative(0).mag())
        )
        ) <= sin(epsilon));
}

struct Shape {
    vector<Contour> contours;

    void draw(uint32_t color, int thickness) {
        for (auto& contour : contours) {
            contour.draw(color, thickness);
        }
    }

    Edge* closestEdge(const vec2& P, uint32_t channel) {
        float dMin = std::numeric_limits<float>::infinity();
        float oMin = std::numeric_limits<float>::infinity();
        Edge* eMin = nullptr;

        for (auto &contour : contours) {
            for (auto &edge : contour.edges) {
                // Skip edges that don't match the channel
                if ((edge.color & channel) == 0) continue;

                float d = edge.signedDistanceTo(P);
                float o = edge.orthogonality(P);

                // If the current edge is closer, or equally close but more orthogonal
                if (fabs(d) < fabs(dMin) || (AreSame(fabs(d), fabs(dMin)) && o > oMin)) {
                    eMin = &edge;
                    dMin = d;
                    oMin = o;
                }
            }
        }

        return eMin;
    }

    //coloring edges
    void edgeColoring(){
        for(auto &contour : contours){
            if(contour.edges.size() == 1){
                contour.edges[0].color = 0xFFFFFFFF;
                continue;
            }
            
            uint32_t current = 0xFFFF00FF;

            for(int i = 0; i < contour.edges.size(); i ++){
                contour.edges[i].color = current;
                
                Edge* a = &contour.edges[i];
                Edge* b = &contour.edges[(i+1)%contour.edges.size()];

                if(isCornerSharp(a,b)){
                    if(current == 0xFF00FFFF){
                        current = 0xFFFFFF00;
                    }else{
                        current = 0xFF00FFFF;
                    }
                }
            }
        }
    }
};

uint32_t rgb(float r, float g, float b, float a){
    return (((uint32_t)(a*255)) << 8*3) | (((uint32_t)(b*255)) << 8*2) | (((uint32_t)(g*255)) << 8*1) | (((uint32_t)(r*255)) << 8*0);
}

// uint32_t generatePixel(Shape& shape, const vec2& P){
//     Edge* e = shape.closestEdge(P, 0xFFFFFFFF);
//     float distance = e->signedDistanceTo(P);

//     //distance color
//     distance += 0.5;
//     return blendColors(e->color,0xFF000000,distance);
// }

// float distanceColor(float distance){
//     // return (distance+1.0)/2 > 0.5 ? 1.0 : 0.0;
//     return (distance+1.0)/2;
// }

// uint32_t generatePixel(Shape& shape, const vec2& P) {
//     Edge* eR = shape.closestEdge(P, 0x000000FF);
//     Edge* eG = shape.closestEdge(P, 0x0000FF00);
//     Edge* eB = shape.closestEdge(P, 0x00FF0000);

//     if (!eR || !eG || !eB) {
//         return 0xFF000000; // Black if no edge is found
//     }

//     float dR = distanceColor(eR->signedDistanceTo(P));
//     float dG = distanceColor(eG->signedDistanceTo(P));
//     float dB = distanceColor(eB->signedDistanceTo(P));

//     return rgb(dR, dG, dB, 1.0f);
// }

float smoothStep(float edge0, float edge1, float x) {
    x = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return x * x * (3 - 2 * x);
}

float smoothDistance(float distance) {
    // return smoothStep(-1.0f, 1.0f, distance);
    return smoothStep(-1.0f, 1.0f, distance) > 0.5 ? 1.0f : 0.0f;
}

uint32_t generatePixel(Shape& shape, const vec2& P) {
    Edge* eR = shape.closestEdge(P, 0x000000FF);
    Edge* eG = shape.closestEdge(P, 0x0000FF00);
    Edge* eB = shape.closestEdge(P, 0x00FF0000);

    if (!eR || !eG || !eB) {
        return 0xFF000000; // Black if no edge is found
    }

    float dR = smoothDistance(eR->signedDistanceTo(P));
    float dG = smoothDistance(eG->signedDistanceTo(P));
    float dB = smoothDistance(eB->signedDistanceTo(P));

    return rgb(dR, dG, dB, 1.0f);
}

int main() {
    pixels = (uint32_t*)malloc(IMAGE_WIDTH * IMAGE_HEIGHT * 4);

    // random shape

    // vec2 Offsetter = {0.01f,0.5f};
    // vec2 a  = { Offsetter.x+0.00f,Offsetter.y+0.0f};
    // vec2 aI = { Offsetter.x+0.20f,Offsetter.y+0.0f};
    // vec2 b  = { Offsetter.x+0.40f,Offsetter.y+0.2f};
    // vec2 bI = { Offsetter.x+0.60f,Offsetter.y+0.4f};
    // vec2 c  = {-Offsetter.x+1.00f,Offsetter.y+0.3f};
    // vec2 cI = { Offsetter.x+0.70f,Offsetter.y+0.26f};
    // vec2 d  = { Offsetter.x+0.75f,Offsetter.y+0.05f};
    // vec2 dI = { Offsetter.x+0.80f,Offsetter.y+-0.16f};
    // vec2 e  = {-Offsetter.x+1.00f,Offsetter.y+-0.1f};
    // vec2 eI = { Offsetter.x+0.80f,Offsetter.y+-0.25f};
    // vec2 f =  { Offsetter.x+0.50f,Offsetter.y+-0.3f};
    // vec2 fI = { Offsetter.x+0.20f,Offsetter.y+-0.3f};

    // AAAAAAAAAA

    vec2 p1 = {0.1,0.1};
    vec2 p2 = {0.3,0.9};
    vec2 p3 = {0.7,0.9};
    vec2 p4 = {0.9,0.1};
    vec2 p5 = {0.7,0.1};
    vec2 p6 = {0.6,0.4};
    vec2 p7 = {0.4,0.4};
    vec2 p8 = {0.3,0.1};

    // EEEEEEEEE

    // vec2 p1 = {0.106,0.486};
    // vec2 p2 = {0.1,0.9};
    // vec2 p3 = {0.5,0.9};
    // vec2 p4 = {0.8,0.9};
    // vec2 p5 = {0.8,0.5};
    // vec2 p6 = {0.3,0.5};
    // vec2 p7 = {0.295,0.24};
    // vec2 p8 = {0.8,0.3};
    // vec2 p9 = {0.8,0.15};
    // vec2 p10 = {0.1,0.05};


    Shape test{
        {
            Contour{
                {

                    // random shape

                    // Edge(EdgeBezier{
                    //     a,
                    //     b,
                    //     aI
                    // }),
                    // Edge(EdgeBezier{
                    //     b,
                    //     c,
                    //     bI
                    // }),
                    // Edge(EdgeBezier{
                    //     c,
                    //     d,
                    //     cI
                    // }),
                    // Edge(EdgeBezier{
                    //     d,
                    //     e,
                    //     dI
                    // }),
                    // Edge(EdgeBezier{
                    //     e,
                    //     f,
                    //     eI
                    // }),
                    // Edge(EdgeBezier{
                    //     f,
                    //     a,
                    //     fI
                    // }),

                    // AAAAAAAAAAA

                    Edge(EdgeLine{
                            p1,
                            p2
                        }
                    ),
                    Edge(EdgeLine{
                            p2,
                            p3
                        }
                    ),
                    Edge(EdgeLine{
                            p3,
                            p4
                        }
                    ),
                    Edge(EdgeLine{
                            p4,
                            p5
                        }
                    ),
                    Edge(EdgeLine{
                            p5,
                            p6
                        }
                    ),
                    Edge(EdgeLine{
                            p6,
                            p7
                        }
                    ),
                    Edge(EdgeLine{
                            p7,
                            p8
                        }
                    ),
                    Edge(EdgeLine{
                            p8,
                            p1
                        }
                    ),

                    // EEEEEEEEEEEE
                    // Edge(EdgeBezier{
                    //     p9,
                    //     p1,
                    //     p10
                    // }),
                    // Edge(EdgeBezier{
                    //     p1,
                    //     p3,
                    //     p2
                    // }),
                    // Edge(EdgeBezier{
                    //     p3,
                    //     p5,
                    //     p4
                    // }),
                    // Edge(EdgeBezier{
                    //     p5,
                    //     p6,
                    //     p5
                    // }),
                    // Edge(EdgeBezier{
                    //     p6,
                    //     p8,
                    //     p7
                    // }),
                    // Edge(EdgeBezier{
                    //     p8,
                    //     p9,
                    //     p8
                    // }),
                }
            }
        }
    };


    test.edgeColoring();

    #pragma omp parallel for
    for(int y = 0; y < IMAGE_HEIGHT; y++){
        for(int x = 0; x < IMAGE_WIDTH; x++){
            vec2 P = {((float)x+0.5f)/IMAGE_WIDTH, ((float)y+0.5f)/IMAGE_HEIGHT};
            pixels[(IMAGE_HEIGHT-1-y)*IMAGE_WIDTH+x] = generatePixel(test,P);
        }
    }

    // test.draw(0xFFFFFFFF, 3); // White color, thickness = 3

    if(!stbi_write_png("out.png", IMAGE_WIDTH, IMAGE_HEIGHT, 4, pixels, IMAGE_WIDTH * 4)){
        printf("Could not save");
    }

    return 0;
}
