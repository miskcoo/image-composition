#ifndef __LAYER_H__
#define __LAYER_H__

#include "image.h"
#include <algorithm>
#include <memory>

class layer_t
{
	std::shared_ptr<image_t> image;
	int offset_x, offset_y;
	uint8_t *mask;
public:
	layer_t() : mask(nullptr) {}
	layer_t(const layer_t&)  = delete;
	~layer_t() { if(mask) delete[] mask; }

	void set_offset(int ox, int oy)
	{
		offset_x = ox;
		offset_y = oy;
	}

	void load(const char *image_path, const char *mask_path)
	{
		image = std::make_shared<image_t>(image_path);
		mask = new uint8_t[image->w * image->h];
		if(mask_path)
		{
			image_t mask_image(mask_path);
			for(int i = 0; i < image->h; ++i)
				for(int j = 0; j < image->w; ++j)
					mask[i * image->w + j] = mask_image.get(i, j, 0) > 128;
		} else {
			std::memset(mask, 255, image->w * image->h);
		}
	}

	template<typename Callback>
	void traverse(int xl, int yl, int xr, int yr, Callback callback)
	{
		xl = std::max(xl, offset_x);
		yl = std::max(yl, offset_y);
		xr = std::min(xr, offset_x + image->h);
		yr = std::min(yr, offset_y + image->w);
		for(int i = xl; i < xr; ++i)
			for(int j = yl; j < yr; ++j)
				if(get_mask(i, j))
					callback(i, j, image->get_ptr(i - offset_x, j - offset_y));
	}

	bool get_mask(int x, int y)
	{
		x -= offset_x;
		y -= offset_y;
		if(x < 0 || y < 0 || x >= image->h || y >= image->w)
			return false;
		return mask[x * image->w + y];
	}

	uint8_t* get_ptr(int x, int y)
	{
		x -= offset_x;
		y -= offset_y;
		if(x < 0 || y < 0 || x >= image->h || y >= image->w)
			return nullptr;
		return image->get_ptr(x, y);
	}

	uint8_t get_color(int x, int y, int c)
	{
		uint8_t *ptr = get_ptr(x, y);
		if(ptr == nullptr)
			return 255;
		return ptr[c];
	}
};

#endif
