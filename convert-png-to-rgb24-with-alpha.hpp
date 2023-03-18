#include <iostream>
#include <string>
#include <png.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace std;

struct Image {
    int width;
    int height;
    std::vector<unsigned char> data;
};

void readPNG(const string& filename, unsigned char* data, int width, int height) {
    FILE* fp = fopen(filename.c_str(), "rb");
    if (!fp) {
        cerr << "Error: failed to open file " << filename << endl;
        return;
    }
    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png_ptr) {
        cerr << "Error: failed to create PNG read structure" << endl;
        fclose(fp);
        return;
    }
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        cerr << "Error: failed to create PNG info structure" << endl;
        png_destroy_read_struct(&png_ptr, NULL, NULL);
        fclose(fp);
        return;
    }
    if (setjmp(png_jmpbuf(png_ptr))) {
        cerr << "Error: failed to read PNG file" << endl;
        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        fclose(fp);
        return;
    }
    png_init_io(png_ptr, fp);
    png_read_info(png_ptr, info_ptr);
    width = png_get_image_width(png_ptr, info_ptr);
    height = png_get_image_height(png_ptr, info_ptr);
    int color_type = png_get_color_type(png_ptr, info_ptr);
    int bit_depth = png_get_bit_depth(png_ptr, info_ptr);
    // Convert paletted images to RGB
    if (color_type == PNG_COLOR_TYPE_PALETTE) {
        png_set_palette_to_rgb(png_ptr);
    }
    // Convert grayscale images to RGB
    if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8) {
        png_set_expand_gray_1_2_4_to_8(png_ptr);
    }
    // Convert transparency chunks to full alpha channel
    if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS)) {
        png_set_tRNS_to_alpha(png_ptr);
    }
    // Convert 16-bit images to 8-bit
    if (bit_depth == 16) {
        png_set_strip_16(png_ptr);
    }
    // Allocate memory for the pixel data
    int row_bytes = png_get_rowbytes(png_ptr, info_ptr);
    data = new unsigned char[row_bytes * height];
    // Read the pixel data
    png_bytepp row_pointers = new png_bytep[height];
    for (int y = 0; y < height; y++) {
        row_pointers[y] = data + y * row_bytes;
    }
    png_read_image(png_ptr, row_pointers);
    // Clean up
    delete[] row_pointers;
    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
    fclose(fp);
}

void writePNG(const std::string& filename, unsigned char* data, int width, int height) {
    FILE* fp = fopen(filename.c_str(), "wb");
    if (!fp) {
        std::cerr << "Error: Could not open file " << filename << " for writing" << std::endl;
        return;
    }
    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr) {
        std::cerr << "Error: Could not create PNG write struct" << std::endl;
        fclose(fp);
        return;
    }
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        std::cerr << "Error: Could not create PNG info struct" << std::endl;
        png_destroy_write_struct(&png_ptr, nullptr);
        fclose(fp);
        return;
    }
    if (setjmp(png_jmpbuf(png_ptr))) {
        std::cerr << "Error: An error occurred while writing the PNG file" << std::endl;
        png_destroy_write_struct(&png_ptr, &info_ptr);
        fclose(fp);
        return;
    }
    png_init_io(png_ptr, fp);
    png_set_IHDR(png_ptr, info_ptr, width, height, 8, PNG_COLOR_TYPE_RGB_ALPHA, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png_ptr, info_ptr);
    png_bytep row_pointers[height];
    for (int i = 0; i < height; i++) {
        row_pointers[i] = data + i * width * 4;
    }
    png_write_image(png_ptr, row_pointers);
    png_write_end(png_ptr, nullptr);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    fclose(fp);
}

Image read_png_file(const char* filename) {
    Image image;
    FILE* fp = fopen(filename, "rb");
    if (!fp) {
        fprintf(stderr, "Failed to open file: %s\n", filename);
        exit(EXIT_FAILURE);
    }
    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr) {
        fclose(fp);
        fprintf(stderr, "Failed to create png read struct.\n");
        exit(EXIT_FAILURE);
    }
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_read_struct(&png_ptr, nullptr, nullptr);
        fclose(fp);
        fprintf(stderr, "Failed to create png info struct.\n");
        exit(EXIT_FAILURE);
    }
    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
        fclose(fp);
        fprintf(stderr, "Failed to read PNG file.\n");
        exit(EXIT_FAILURE);
    }
    png_init_io(png_ptr, fp);
    png_read_info(png_ptr, info_ptr);
    int color_type = png_get_color_type(png_ptr, info_ptr);
    int bit_depth = png_get_bit_depth(png_ptr, info_ptr);
    if (bit_depth == 16) {
        png_set_strip_16(png_ptr);
    }
    if (color_type == PNG_COLOR_TYPE_PALETTE) {
        png_set_palette_to_rgb(png_ptr);
    }
    if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8) {
        png_set_expand_gray_1_2_4_to_8(png_ptr);
    }
    if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS)) {
        png_set_tRNS_to_alpha(png_ptr);
    }
    if (color_type == PNG_COLOR_TYPE_RGB || color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_PALETTE) {
        png_set_filler(png_ptr, 0xFF, PNG_FILLER_AFTER);
    }
    if (color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_GRAY_ALPHA) {
        png_set_gray_to_rgb(png_ptr);
    }
    png_read_update_info(png_ptr, info_ptr);
    int row_bytes = png_get_rowbytes(png_ptr, info_ptr);
    image.width = png_get_image_width(png_ptr, info_ptr);
    image.height = png_get_image_height(png_ptr, info_ptr);
    std::vector<unsigned char> data(row_bytes * image.height);
    png_bytep* row_pointers = new png_bytep[image.height];
    for (int y = 0; y < image.height; y++) {
        row_pointers[y] = &data[y * row_bytes];
    }
    png_read_image(png_ptr, row_pointers);
    png_read_end(png_ptr, nullptr);
    delete[] row_pointers;
    png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
    fclose(fp);
    image.data = std::move(data);
    return image;
}
