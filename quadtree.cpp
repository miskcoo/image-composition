#include <cstdlib>
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

bool quadtree_t::is_leaf() const
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
	image_t img(width, height);
	traverse([&](int xl, int xr, int yl, int yr) {
		xr = std::min(xr, height - 1);
		yr = std::min(yr, width - 1);
		int color = std::rand();
		for(int i = xl; i < xr; ++i)
			for(int j = yl; j < yr; ++j)
				img.set_rgb(i, j, color);
	} );

	img.write(filename);
}
