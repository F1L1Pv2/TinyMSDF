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
    // Load the font file
    auto data = filesWrap::readFile("Courier New.ttf");

    stbtt_fontinfo font = {};
    if(!stbtt_InitFont(&font, data->data(), 0)){
        printf("failed\n");
        return 1;
    }

    // Get font metrics
    int ascent, descent, lineGap;
    stbtt_GetFontVMetrics(&font, &ascent, &descent, &lineGap);

    int x0, y0, x1, y1;
    stbtt_GetFontBoundingBox(&font, &x0, &y0, &x1, &y1);

    // Calculate the bounding box dimensions.
    float fontWidth = (float)(x1 - x0);
    float fontHeight = (float)(y1 - y0);

    // Compute a uniform scale factor so that the largest dimension maps to 1.0.
    float scale = 1.0f / max(fontWidth, fontHeight);

    // Compute offsets to center the glyph in the [0,1] square.
    float offsetX = 0.0f, offsetY = 0.0f;
    if(fontWidth < fontHeight) {
        offsetX = (1.0f - fontWidth * scale) / 2.0f;
    } else if(fontHeight < fontWidth) {
        offsetY = (1.0f - fontHeight * scale) / 2.0f;
    }

    // Get glyph shape for character 'F'
    int glyphIndex = stbtt_FindGlyphIndex(&font, 'a');
    stbtt_vertex* vertices;
    int numVertices = stbtt_GetGlyphShape(&font, glyphIndex, &vertices);

    bool first = true;
    Shape shape = {};
    Contour contour = {};
    vec2 cursor = {};

    for (int i = 0; i < numVertices; i++) {
        stbtt_vertex v = vertices[i];
        switch (v.type) {
            case STBTT_vmove:{
                if(!first){
                    shape.contours.push_back(contour);
                    contour = {};
                } else {
                    first = false;
                }
                // Apply uniform scaling and offset to the move-to point.
                cursor = { (float)(v.x - x0) * scale + offsetX, (float)(v.y - y0) * scale + offsetY };
                printf("Move to (%f, %f)\n", cursor.x, cursor.y);
                break;
            }
            case STBTT_vline:{
                vec2 newP = { (float)(v.x - x0) * scale + offsetX, (float)(v.y - y0) * scale + offsetY };
                printf("Line to (%f, %f)\n", newP.x, newP.y);
                contour.edges.push_back(Edge(EdgeLine{cursor, newP}));
                cursor = newP;
                break;
            }
            case STBTT_vcurve:{
                vec2 newP = { (float)(v.x - x0) * scale + offsetX, (float)(v.y - y0) * scale + offsetY };
                vec2 control = { (float)(v.cx - x0) * scale + offsetX, (float)(v.cy - y0) * scale + offsetY };
                printf("Quadratic Bezier to (%f, %f) with control (%f, %f)\n", newP.x, newP.y, control.x, control.y);
                contour.edges.push_back(Edge(EdgeBezier{cursor, newP, control}));
                cursor = newP;
                break;
            }
            case STBTT_vcubic:{
                vec2 newP = { (float)(v.x - x0) * scale + offsetX, (float)(v.y - y0) * scale + offsetY };
                vec2 control1 = { (float)(v.cx - x0) * scale + offsetX, (float)(v.cy - y0) * scale + offsetY };
                vec2 control2 = { (float)(v.cx1 - x0) * scale + offsetX, (float)(v.cy1 - y0) * scale + offsetY };
                printf("Cubic Bezier to (%f, %f) with controls (%f, %f) and (%f, %f)\n", newP.x, newP.y, control1.x, control1.y, control2.x, control2.y);
                contour.edges.push_back(Edge(EdgeCubicBezier{cursor, newP, control1, control2}));
                cursor = newP;
                break;
            }
        }
    }
    // Push the last contour.
    shape.contours.push_back(contour);

    // Generate MSDF with normalized coordinates.
    uint32_t* outPixels = genMsdf(shape, {
        .width = 64,
        .height = 64,
        .smooth_val = 0.15f
    });

    if(!stbi_write_png("out.png", 64, 64, 4, outPixels, 64 * 4)){
        printf("Could not save\n");
    }

    free(outPixels);
    return 0;
}
