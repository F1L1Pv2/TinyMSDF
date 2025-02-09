#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb_image_resize.h"

double cross(const vec2& a, const vec2& b){
    return a.x*b.y - a.y*b.x;
}

uint8_t extractChannel(uint32_t color, int shift) {
    return (color >> shift) & 0xFF;
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
};

bool AreSame(double a, double b, double epsilon = 1e-10)
{
    return fabs(a - b) < epsilon;
}

struct EdgeBezier {
    vec2 start;
    vec2 end;
    vec2 control;

    // A quadratic Bezier using the standard formulation:
    // B(t) = (1-t)^2*start + 2t(1-t)*control + t^2*end
    vec2 point(double t) const {
        double omt = 1.0 - t;
        return start * (omt * omt) + control * (2 * t * omt) + end * (t * t);
    }

    // Its derivative:
    // B'(t) = 2(1-t)(control - start) + 2t(end - control)
    vec2 derivative(double t) const {
        return (control - start) * (2.0 * (1.0 - t)) + (end - control) * (2.0 * t);
    }

    // Compute the unsigned distance from p to the curve by dense sampling.
    double distanceTo(const vec2& p) const {
        double minDistance = std::numeric_limits<double>::max();
        const int subdivisions = 100;
        for (int i = 0; i <= subdivisions; ++i) {
            double t = i / (double)subdivisions;
            vec2 curvePoint = point(t);
            double d = (p - curvePoint).mag();
            if (d < minDistance)
                minDistance = d;
        }
        return minDistance;
    }

    // Compute a measure of the signed orthogonality.
    // We sample the curve densely, pick the closest point and then use its tangent.
    double orthogonality(const vec2& p) const {
        if(
            (AreSame(this->control.x,this->start.x) && (AreSame(this->control.y,this->start.y)))
            ||
            (AreSame(this->control.x,this->end.x) && (AreSame(this->control.y,this->end.y)))
        ){
            return EdgeLine{this->start,this->end}.orthogonality(p);
        }
        double minDistance = std::numeric_limits<double>::max();
        double minT = 0.0;
        const int subdivisions = 100; // dense enough for a quadratic curve
        for (int i = 0; i <= subdivisions; ++i) {
            double t = i / (double)subdivisions;
            vec2 curvePoint = point(t);
            double d = (p - curvePoint).mag();
            if (d < minDistance) {
                minDistance = d;
                minT = t;
            }
        }
        vec2 closestPoint = point(minT);
        vec2 tangent = derivative(minT);
        double tangentMag = tangent.mag();
        if (tangentMag < 1e-10)
            return 0.0;
        vec2 tangentNorm = tangent / tangentMag;
        vec2 dir = p - closestPoint;
        double dirMag = dir.mag();
        if (dirMag < 1e-10)
            return 0.0;
        vec2 dirNorm = dir / dirMag;
        // The signed cross product between the tangent and the direction
        return cross(tangentNorm, dirNorm);
    }

    // Compute a signed distance:
    // We sample many points, pick the closest one, then determine the sign using the normal.
    double signedDistanceTo(const vec2& p) const {
        if(
            (AreSame(this->control.x,this->start.x) && (AreSame(this->control.y,this->start.y)))
            ||
            (AreSame(this->control.x,this->end.x) && (AreSame(this->control.y,this->end.y)))
        ){
            return EdgeLine{this->start,this->end}.signedDistanceTo(p);
        }
        double minDistance = std::numeric_limits<double>::max();
        double minT = 0.0;
        const int subdivisions = 200; // use more subdivisions for signed distance accuracy
        for (int i = 0; i <= subdivisions; ++i) {
            double t = i / (double)subdivisions;
            vec2 curvePoint = point(t);
            double d = (p - curvePoint).mag();
            if (d < minDistance) {
                minDistance = d;
                minT = t;
            }
        }
        vec2 closestPoint = point(minT);
        vec2 tangent = derivative(minT);
        double tangentMag = tangent.mag();
        if (tangentMag < 1e-10)
            return minDistance; // fallback if tangent is degenerate
        vec2 tangentNorm = tangent / tangentMag;
        // Compute a normal (by rotating the tangent 90° counter-clockwise)
        vec2 normal = { -tangentNorm.y, tangentNorm.x };
        // Determine sign: positive if the point lies along the normal direction, negative otherwise.
        double sign = (p - closestPoint).dot(normal) >= 0 ? 1.0 : -1.0;
        return sign * minDistance * -1.0f;
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
    bool isClockwise;
};

bool isCornerSharp(const Edge* a,const Edge* b, double epsilon=PI/10){
    return !(abs(
        cross(
            a->derivative(1)/abs(a->derivative(1).mag()),
            b->derivative(0)/abs(b->derivative(0).mag())
        )
        ) <= sin(epsilon));
}

bool isContourClockwise(const Contour& contour) {
    double area = 0.0;
    for (size_t i = 0; i < contour.edges.size(); ++i) {
        const Edge& edge = contour.edges[i];
        vec2 p1 = edge.point(0.0);
        vec2 p2 = edge.point(1.0);
        area += (p2.x - p1.x) * (p2.y + p1.y);
    }
    return area > 0.0;
}


struct Shape {
    vector<Contour> contours;
    bool initialized = false;

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
                if (fabs(d) < fabs(dMin) || (AreSame(fabs(d), fabs(dMin)) && (contour.isClockwise ? o > oMin : o < oMin))) {
                    eMin = &edge;
                    dMin = d;
                    oMin = o;
                }
            }
        }

        return eMin;
    }

    void prepare_shape(){
        for (auto& contour : contours) {
            contour.isClockwise = isContourClockwise(contour);
        }
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

float smoothStep(float edge0, float edge1, float x) {
    x = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return x * x * (3 - 2 * x);
}

struct GenParams{
    size_t width;
    size_t height;
    size_t innerSize = 256;
    float smooth_val = 0.15f;
};

float smoothDistance(GenParams* params, float distance) {
    return smoothStep(-params->smooth_val, params->smooth_val, distance);
}

uint32_t generatePixel(GenParams* params, Shape& shape, const vec2& P) {
    Edge* eR = shape.closestEdge(P, 0x000000FF);
    Edge* eG = shape.closestEdge(P, 0x0000FF00);
    Edge* eB = shape.closestEdge(P, 0x00FF0000);

    if (!eR || !eG || !eB) {
        return 0xFF000000;
    }

    float dR = smoothDistance(params, eR->signedDistanceTo(P));
    float dG = smoothDistance(params, eG->signedDistanceTo(P));
    float dB = smoothDistance(params, eB->signedDistanceTo(P));

    return rgb(dR, dG, dB, 1.0f);
}

float median(float a, float b, float c) {
    std::array<float, 3> values = {a, b, c};
    std::sort(values.begin(), values.end());
    return values[1];
}

void collisionCorrection(uint32_t* pixels, int width, int height) {
    const float threshold = 0.2f;

    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            int index = y * width + x;

            float dR = extractChannel(pixels[index], 0) / 255.0f;
            float dG = extractChannel(pixels[index], 8) / 255.0f;
            float dB = extractChannel(pixels[index], 16) / 255.0f;
            float dR_left = extractChannel(pixels[index - 1], 0) / 255.0f;
            float dG_left = extractChannel(pixels[index - 1], 8) / 255.0f;
            float dB_left = extractChannel(pixels[index - 1], 16) / 255.0f;
            float dR_right = extractChannel(pixels[index + 1], 0) / 255.0f;
            float dG_right = extractChannel(pixels[index + 1], 8) / 255.0f;
            float dB_right = extractChannel(pixels[index + 1], 16) / 255.0f;
            float dR_top = extractChannel(pixels[index - width], 0) / 255.0f;
            float dG_top = extractChannel(pixels[index - width], 8) / 255.0f;
            float dB_top = extractChannel(pixels[index - width], 16) / 255.0f;
            float dR_bottom = extractChannel(pixels[index + width], 0) / 255.0f;
            float dG_bottom = extractChannel(pixels[index + width], 8) / 255.0f;
            float dB_bottom = extractChannel(pixels[index + width], 16) / 255.0f;

            bool collisionR = (fabs(dR - dR_left) > threshold) || (fabs(dR - dR_right) > threshold) ||
                              (fabs(dR - dR_top) > threshold) || (fabs(dR - dR_bottom) > threshold);

            bool collisionG = (fabs(dG - dG_left) > threshold) || (fabs(dG - dG_right) > threshold) ||
                              (fabs(dG - dG_top) > threshold) || (fabs(dG - dG_bottom) > threshold);

            bool collisionB = (fabs(dB - dB_left) > threshold) || (fabs(dB - dB_right) > threshold) ||
                              (fabs(dB - dB_top) > threshold) || (fabs(dB - dB_bottom) > threshold);

            if ((collisionR && collisionG) || (collisionR && collisionB) || (collisionG && collisionB)) {
                float medianDistance = median(dR, dG, dB);
                uint32_t correctedColor = rgb(medianDistance, medianDistance, medianDistance, 1.0f);
                pixels[index] = correctedColor;
            }
        }
    }
}

uint32_t* genMsdf(Shape& shape, GenParams params){
    if(params.width > params.innerSize) params.innerSize = params.width;
    if(params.height > params.innerSize) params.innerSize = params.height;

    if(shape.initialized == false){
        shape.prepare_shape();
        shape.edgeColoring();
        shape.initialized = true;
    }

    
    uint32_t* pixels = (uint32_t*)malloc(params.innerSize * params.innerSize * 4);
    uint32_t* outPixels;

    if(shape.contours.size() == 0){
        memset(pixels,0xFF000000,sizeof(uint32_t));
        return pixels;
    }

    #pragma omp parallel for
    for(int i = 0; i < params.innerSize*params.innerSize; i++){
        int x = i % params.innerSize;
        int y = params.innerSize - i / params.innerSize;
        vec2 P = {((float)x+0.5f)/params.innerSize, ((float)y+0.5f)/params.innerSize};
        pixels[i] = generatePixel(&params,shape,P);
    }

    if(params.width == params.innerSize && params.height == params.innerSize){
        outPixels = pixels;
    }else{
        outPixels = (uint32_t*)malloc(params.width * params.height * 4);
        stbir_resize_uint8((const unsigned char*)pixels,params.innerSize,params.innerSize,params.innerSize*4,(unsigned char*)outPixels,params.width,params.height,params.width*4,4);
    }

    collisionCorrection(outPixels, params.width, params.height);

    if(pixels != outPixels) free(pixels);
    return outPixels;
}