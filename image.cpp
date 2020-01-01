#include "image.h"

#define STB_IMAGE_IMPLEMENTATION
#include "tools/stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "tools/stb_image_write.h"
	
void image_t::write(const char* filename)
{
	stbi_write_png(filename, w, h, c, buf, 0);
	std::printf("Save image %s: %dx%dx%d\n", filename, w, h, c);
}

image_t::image_t(const char* filename)
{
	uint8_t *stbi_buf = stbi_load(filename, &w, &h, &c, 0);
	buf = new uint8_t[w * h * c];
	std::memcpy(buf, stbi_buf, w * h * c);
	stbi_image_free(stbi_buf);
	std::printf("Load image %s: %dx%dx%d\n", filename, w, h, c);
}

