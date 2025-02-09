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

int main() {

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

    // vec2 p1 = {0.1,0.1};
    // vec2 p2 = {0.3,0.9};
    // vec2 p3 = {0.7,0.9};
    // vec2 p4 = {0.9,0.1};
    // vec2 p5 = {0.7,0.1};
    // vec2 p6 = {0.6,0.4};
    // vec2 p7 = {0.4,0.4};
    // vec2 p8 = {0.3,0.1};

    // EEEEEEEEE

    vec2 p1 = {0.106,0.486};
    vec2 p2 = {0.1,0.9};
    vec2 p3 = {0.5,0.9};
    vec2 p4 = {0.8,0.9};
    vec2 p5 = {0.8,0.5};
    vec2 p6 = {0.3,0.5};
    vec2 p7 = {0.295,0.24};
    vec2 p8 = {0.8,0.3};
    vec2 p9 = {0.8,0.15};
    vec2 p10 = {0.1,0.05};

    // reverse triangle
    // vec2 tp1 = {0.3,0.5};
    // vec2 tp2 = {0.7,0.5};
    // vec2 tp3 = {0.5,0.85};

    // reverse triangle-e
    vec2 tp1 = {0.3,0.55};
    vec2 tp2 = {0.7,0.55};
    vec2 tp3 = {0.5,0.8};


    Shape test{
        .contours = {
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
                    // Edge(EdgeLine{
                    //         p1,
                    //         p2
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p2,
                    //         p3
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p3,
                    //         p4
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p4,
                    //         p5
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p5,
                    //         p6
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p6,
                    //         p7
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p7,
                    //         p8
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p8,
                    //         p1
                    //     }
                    // ),

                    // AAAAAAAAAAAAA - reverse winding
                    // Edge(EdgeLine{
                    //         p1,
                    //         p8
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p8,
                    //         p7
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p7,
                    //         p6
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p6,
                    //         p5
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p5,
                    //         p4
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p4,
                    //         p3
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p3,
                    //         p2
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p2,
                    //         p1
                    //     }
                    // ),

                    // EEEEEEEEEEEE
                    Edge(EdgeBezier{
                        p9,
                        p1,
                        p10
                    }),
                    Edge(EdgeBezier{
                        p1,
                        p3,
                        p2
                    }),
                    Edge(EdgeBezier{
                        p3,
                        p5,
                        p4
                    }),
                    Edge(EdgeBezier{
                        p5,
                        p6,
                        p5
                    }),
                    Edge(EdgeBezier{
                        p6,
                        p8,
                        p7
                    }),
                    Edge(EdgeBezier{
                        p8,
                        p9,
                        p8
                    }),
                }
            },

            Contour {
                {
                    // AAAAAAAAAAAAA - reverse winding
                    // Edge(EdgeLine{
                    //         p1,
                    //         p8
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p8,
                    //         p7
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p7,
                    //         p6
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p6,
                    //         p5
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p5,
                    //         p4
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p4,
                    //         p3
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p3,
                    //         p2
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         p2,
                    //         p1
                    //     }
                    // ),
                }
            },

            Contour{
                {
                    // triangle
                    // Edge(EdgeLine{
                    //         tp1,
                    //         tp3
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         tp3,
                    //         tp2
                    //     }
                    // ),
                    // Edge(EdgeLine{
                    //         tp2,
                    //         tp1
                    //     }
                    // ),
                    // triangle reversed
                    Edge(EdgeLine{
                            tp1,
                            tp2
                        }
                    ),
                    Edge(EdgeLine{
                            tp2,
                            tp3
                        }
                    ),
                    Edge(EdgeLine{
                            tp3,
                            tp1
                        }
                    ),
                }
            }
        }
    };

    uint32_t* outPixels = genMsdf(test,{
        .width = 32,
        .height = 32
    });

    if(!stbi_write_png("out.png", 32, 32, 4, outPixels, 32 * 4)){
        printf("Could not save");
    }

    free(outPixels);

    return 0;
}
