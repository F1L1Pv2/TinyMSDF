#include <omp.h>
#include <stdio.h>
#include <malloc.h>
#include <vector>
#include <string>
#include <stdint.h>
#include <limits>
#include "3dmath.h"
#include <algorithm>
#include <array>

using namespace std;

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "GenMsdf.h"

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#include "filesWrap.h"

int main(){

    auto data = filesWrap::readFile("Courier New.ttf");


    stbtt_fontinfo font = {};
    if(!stbtt_InitFont(&font,data->data(),0)){
        printf("failed\n");
    }

    int glyphIndex = stbtt_FindGlyphIndex(&font, 'F'); // Get glyph index for a character
    stbtt_vertex* vertices;
    int numVertices = stbtt_GetGlyphShape(&font, glyphIndex, &vertices);

    bool first = true;

    Shape shape = {};

    Contour contour = {};

    Edge edge = {};

    vec2 cursor = {};

    for (int i = 0; i < numVertices; i++) {
        stbtt_vertex v = vertices[i];
        switch (v.type) {
            case STBTT_vmove:{
                if(!first){
                    shape.contours.push_back(contour);
                    contour = {};
                }else{
                    first = false;
                }

                cursor = {(float)v.x, (float)v.y};
                // // Start of a new contour
                printf("Move to (%d, %d)\n", v.x, v.y);
                break;
            }

            case STBTT_vline:{
                // Straight line to (v.x, v.y)
                printf("Line to (%d, %d)\n", v.x, v.y);


                vec2 newP = {(float)v.x,(float)v.y};
                contour.edges.push_back(Edge(EdgeLine{cursor,newP}));
                cursor = newP;

                break;
            }
            case STBTT_vcurve:{
                // Quadratic Bézier curve with control point (v.cx, v.cy) and endpoint (v.x, v.y)
                printf("Quadratic Bezier to (%d, %d) with control (%d, %d)\n", v.x, v.y, v.cx, v.cy);

                vec2 newP = {(float)v.x,(float)v.y};
                contour.edges.push_back(Edge(EdgeBezier{cursor,newP, {(float)v.cx,(float)v.cy}}));
                cursor = newP;

                break;
            }
            case STBTT_vcubic:{
                // Cubic Bézier curve with control points (v.cx, v.cy), (v.cx1, v.cy1), and endpoint (v.x, v.y)
                printf("Cubic Bezier to (%d, %d) with controls (%d, %d) and (%d, %d)\n", v.x, v.y, v.cx, v.cy, v.cx1, v.cy1);

                vec2 newP = {(float)v.x,(float)v.y};
                contour.edges.push_back(Edge(EdgeCubicBezier{cursor,newP, {(float)v.cx,(float)v.cy}, {(float)v.cx1, (float)v.cy1}}));
                cursor = newP;

                break;
            }
        }
    }

    shape.contours.push_back(contour);
    uint32_t* outPixels = genMsdf(shape,{
        .width = 64,
        .height = 64,
        .scale = 1300,
        .smooth_val = 0.15f
    });

    if(!stbi_write_png("out.png", 64, 64, 4, outPixels, 64 * 4)){
        printf("Could not save");
    }

    free(outPixels);

    return 0;
}