#include <cstdlib>
#include <cstdio>
#include <algorithm>
#include "quadtree.h"
#include "image.h"

quadtree_t::quadtree_t(int xl, int xr, int yl, int yr)
	: xl(xl), xr(xr), yl(yl), yr(yr)
{
	range = xr - xl;
	s_ll = s_lr = s_rl = s_rr = nullptr;
}

quadtree_t::~quadtree_t()
{
	if(!is_leaf())
	{
		delete s_ll;
		delete s_lr;
		delete s_rl;
		delete s_rr;
	}
}

bool quadtree_t::is_leaf()
{
    return s_ll == nullptr;
}

bool quadtree_t::in_range(int x, int y)
{
	return x >= xl && y >= yl && x < xr && y < yr;
}

quadtree_t *quadtree_t::find(int x, int y)
{
	if(!in_range(x, y))
		return nullptr;
	if(is_leaf()) return this;
	return _find_child(x, y)->find(x, y);
}

bool quadtree_t::is_keypoint(int x, int y)
{
	if(x == 0 || y == 0)
		return true;
	quadtree_t *node = find(x, y);
	if(node->xl == x && node->yl == y)
	{
		quadtree_t *outer = find_outer(x, y);
		if(outer == nullptr) return false;
		else return outer->xr == x && outer->yr == y;
	} else return false;
}

quadtree_t* quadtree_t::find_outer(int x, int y)
{
	if(is_leaf())
		return this;

	if(xl <= x && x <= xr && yl <= y && y <= yr)
	{
		int xm = (xl + xr) >> 1, ym = (yl + yr) >> 1;
		quadtree_t *s;
		if(x <= xm) s = y <= ym ? s_ll : s_lr;
		else s = y <= ym ? s_rl : s_rr;
		return s->find_outer(x, y);
	} else return nullptr;
}

quadtree_t *quadtree_t::_find_child(int x, int y)
{
	int xm = (xl + xr) >> 1, ym = (yl + yr) >> 1;

	quadtree_t *s;
	if(x < xm) s = y < ym ? s_ll : s_lr;
	else s = y < ym ? s_rl : s_rr;
	return s;
}

void quadtree_t::_split()
{
	int xm = (xl + xr) >> 1, ym = (yl + yr) >> 1;
	s_ll = new quadtree_t(xl, xm, yl, ym);
	s_lr = new quadtree_t(xl, xm, ym, yr);
	s_rl = new quadtree_t(xm, xr, yl, ym);
	s_rr = new quadtree_t(xm, xr, ym, yr);
}

void quadtree_t::_split_tree(quadtree_t *root, int x, int y, int range)
{
	auto sub_split = [=](int x, int y, int range) {
		quadtree_t *n = root->find(x, y);
		if(n && n->range > range)
			_split_tree(root, x, y, range);
	};
	quadtree_t *now = root;
	while(now->range > range)
	{
		if(now->is_leaf())
		{
			now->_split();
			sub_split(now->xl - 1, now->yl, now->range);
			sub_split(now->xl, now->yl - 1, now->range);
			sub_split(now->xr, now->yl, now->range);
			sub_split(now->xl, now->yr, now->range);
		}

		now = now->_find_child(x, y);
	}
}

void quadtree_t::split(int x, int y, int range)
{
	_split_tree(this, x, y, range);
}

void quadtree_t::dump_to(const char *filename, int width, int height)
{
	if(width == 0) width = range;
	if(height == 0) height = range;
	image_t img(width, height);
	traverse([&](int xl, int xr, int yl, int yr) {
		xr = std::min(xr, height - 1);
		yr = std::min(yr, width - 1);
		int color = std::rand();
		for(int i = xl; i < xr; ++i)
			for(int j = yl; j < yr; ++j)
				img.set_rgb(i, j, color);
	} );

//	for(int i = 0; i < height; ++i)
//		for(int j = 0; j < width; ++j)
//			if(is_keypoint(i, j))
//				for(int s = -1; s <= 50; ++s)
//					for(int t = -1; t <= 50; ++t)
//						img.set_rgb(i + s, j + t, 0);

	img.write(filename);
}

quadtree_t::quadtree_t(const char* boundary_filename)
{
	auto get_2pow = [](int x) {
		int p = 1;
		while(p < x) p <<= 1;
		return p;
	};

	// initialize
	image_t img(boundary_filename);
	int w = img.w, h = img.h;
	xl = yl = 0;
	range = xr = yr = std::max(get_2pow(h), get_2pow(w));
	s_ll = s_lr = s_rl = s_rr = nullptr;

	// split
	for(int i = 0; i < w; ++i)
		split(h - 1, i, 1);
	for(int i = 0; i < h; ++i)
		split(i, w - 1, 1);

	for(int i = 0; i < h; ++i)
		for(int j = 0; j < w; ++j)
		{
			if(img.get(i, j, 0) < 128)
				split(i, j, 1);
		}
}

void quadtree_t::fill_boundary(int x[4], int y[4])
{
	x[0] = xl, y[0] = yl;
	x[1] = xl, y[1] = yr;
	x[2] = xr, y[2] = yl;
	x[3] = xr, y[3] = yr;
}
