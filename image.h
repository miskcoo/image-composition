#ifndef __IMAGE_H__
#define __IMAGE_H__

#define STB_IMAGE_IMPLEMENTATION
#include "tools/stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "tools/stb_image_write.h"

#include <cstdio>
#include <cstring>
#include <cstdint>
using std::uint8_t;

class image_t
{
public:
	int w, h, c;
	uint8_t *buf;
public:
	int locate(int x, int y)
	{
		x = std::max(0, std::min(x, h - 1));
		y = std::max(0, std::min(y, w - 1));
		return (w * x + y) * c;
	}

	uint8_t* get_ptr(int x, int y) { return buf + locate(x, y); }
	uint8_t get(int x, int y, int c) { return get_ptr(x, y)[c]; }
	void set_rgb(int x, int y, int rgb)
	{
		uint8_t *ptr = get_ptr(x, y);
		ptr[0] = rgb;
		ptr[1] = rgb >> 8;
		ptr[2] = rgb >> 16;
	}

	double interp(double x, double y, int c)
	{
		int fx = (int)x, fy = (int)y;
		int cx = std::ceil(x), cy = std::ceil(y);
		double dx = x - fx, dy = y - fy;

		double l = get(fx, fy, c) * (1.0 - dx) + get(cx, fy, c) * dx;
		double r = get(fx, cy, c) * (1.0 - dx) + get(cx, cy, c) * dx;
		return l * (1.0 - dy) + r * dy;
	}
	
	void write(const char* filename)
	{
		stbi_write_png(filename, w, h, c, buf, 0);
	}

public:
	image_t(const char* filename)
	{
		uint8_t *stbi_buf = stbi_load(filename, &w, &h, &c, 0);
		buf = new uint8_t[w * h * c];
		std::memcpy(buf, stbi_buf, w * h * c);
		stbi_image_free(stbi_buf);
		std::printf("Load image %s: %dx%dx%d\n", filename, w, h, c);
	}

	image_t(int w, int h, int c = 3)
	{
		this->w = w;
		this->h = h;
		this->c = c;
		buf = new uint8_t[w * h * c];
	}

	~image_t()
	{
		delete[] buf;
	}
};

#endif
