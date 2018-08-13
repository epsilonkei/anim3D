#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>

static void create_ppm(char *prefix, int frame_id, unsigned int width, unsigned int height,
        unsigned int color_max, unsigned int pixel_nbytes, GLubyte *pixels) {
    size_t i, j, k, cur;
    enum Constants { max_filename = 256 };
    char filename[max_filename];
    snprintf(filename, max_filename, "%s%d.ppm", prefix, frame_id);
    FILE *f = fopen(filename, "w");
    fprintf(f, "P3\n%d %d\n%d\n", width, height, 255);
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            cur = pixel_nbytes * ((height - i - 1) * width + j);
            fprintf(f, "%3d %3d %3d ", pixels[cur], pixels[cur + 1], pixels[cur + 2]);
        }
        fprintf(f, "\n");
    }
    fclose(f);
}
