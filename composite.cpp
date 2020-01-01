#include "composite.h"
#include "layer.h"
#include "quadtree.h"
#include "image.h"
#include <cassert>
#include <eigen3/Eigen/src/IterativeLinearSolvers/ConjugateGradient.h>
#include <memory>
#include <cmath>
#include <unordered_map>

void image_compositor::build_mixed_image()
{
	img_delta = std::make_shared<image_t>(width, height, 3);
	img_mixed = std::make_shared<image_t>(width, height, 3);
	z_index = std::make_shared<image_t>(width, height, 1);
	for(int i = 0; i < (int)layers.size(); ++i)
	{
		layers[i]->traverse(0, 0, height, width,
			[&](int x, int y, uint8_t *ptr) {
				std::memcpy(img_mixed->get_ptr(x, y), ptr, 3);
				z_index->get_ptr(x, y)[0] = i + 1;
			}
		);
	}
}

void image_compositor::build_boundary()
{
	/* (1) build quadtree */
	int range = 1, boundary_cnt = 0;
	for(int t = std::max(width, height); range < t; range <<= 1);
	qtree = std::make_shared<quadtree_t>(0, range, 0, range);
	static int dir[][2] = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } };
	for(int i = 0; i < width; ++i)
		qtree->split(height - 1, i, 1);
	for(int i = 0; i < height; ++i)
		qtree->split(i, width - 1, 1);
	for(int i = 0; i < height; ++i)
	{
		for(int j = 0; j < width; ++j)
		{
			int z = z_index->get(i, j, 0);
			for(int k = 0; k < 4; ++k)
			{
				int *t = dir[k];
				int ti = i + t[0], tj = j + t[1];
				if(0 <= ti && ti < height && 0 <= tj && tj < width)
				{
					if(z_index->get(ti, tj, 0) != z)
					{
						qtree->split(i, j, 1);
						++boundary_cnt;
						break;
					}
				}
			}
		}
	}

	std::printf("Found boundary points %d\n", boundary_cnt);

	/* (2) load keypoints */
	int keypoint_count = 0;
	for(int i = 0; i < height; ++i)
	{
		for(int j = 0; j < width; ++j)
			if(qtree->is_keypoint(i, j))
				keypoints[std::make_pair(i, j)] = keypoint_count++;
	}

	std::printf("Found key points %d\n", keypoint_count);
}

uint8_t image_compositor::get_color(int x, int y, int ch, int ignore_z)
{
	for(int i = layers.size() - 1; i >= 0; --i)
	{
		if(i == ignore_z)
			continue;
		if(layers[i]->get_mask(x, y))
			return layers[i]->get_color(x, y, ch);
	}

	return 255;
}

void image_compositor::apply_gradient_matrix(const std::vector<interp_line_t>& S, const std::vector<double> B[3], int size)
{
	std::puts("  Computing sparse matrix StS...");
	std::map<std::pair<int, int>, double> M;
	for(auto &line : S)
	{
		for(auto mv1 : line)
			for(auto mv2 : line)
				M[ std::make_pair(mv1.first, mv2.first) ] += mv1.second * mv2.second;
	}

	std::vector<Eigen::Triplet<double>> items;
	for(auto it : M)
		items.push_back( { it.first.first, it.first.second, it.second } );

	StS = std::make_shared<Eigen::SparseMatrix<double>>(size, size);
	StS->setFromTriplets(items.begin(), items.end());
	StS->makeCompressed();

	std::puts("  Computing sparse vectors StB...");
	for(int ch = 0; ch < 3; ++ch)
	{
		StB[ch] = std::make_shared<Eigen::SparseVector<double>>(size);
		StB[ch]->setZero();
		for(int i = 0; i < (int)S.size(); ++i)
			for(auto mv : S[i])
				StB[ch]->coeffRef(mv.first) += mv.second * B[ch][i];
	}
}

void image_compositor::build_full_matrices()
{
	std::puts("  Building matrix S and vector B...");
	std::vector<std::vector<mv_t>> S;
	std::vector<double> B[3];
	for(int i = 0; i < height; ++i)
	{
		for(int j = 0; j < width; ++j)
		{
			for(int axis = 0; axis < 2; ++axis)
			{
				int ti = i - axis, tj = j - (1 - axis);
				if(ti < 0 || tj < 0) continue;

				// interpolation matrix
				std::vector<mv_t> line;
				line.push_back( mv_t(i * width + j, 1.0) );
				line.push_back( mv_t(ti * width + tj, -1.0) );
				S.emplace_back(std::move(line));

				// B vector
				int z = z_index->get(i, j, 0);
				int z_t = z_index->get(ti, tj, 0);
				if(z != z_t)
				{
					int z_m = std::max(z, z_t) - 1;
					for(int ch = 0; ch < 3; ++ch)
					{
						int g0 = img_mixed->get(i, j, ch) - img_mixed->get(ti, tj, ch);
						int g1 = get_color(i, j, ch, z_m) - get_color(ti, tj, ch, z_m);
						B[ch].push_back(g1 - g0);
					}
				} else {
					for(int ch = 0; ch < 3; ++ch)
						B[ch].push_back(0.0);
				}
			}
		}
	}

	std::vector<mv_t> last_line;
	last_line.push_back( { width * height - 1, 1.0 } );
	S.push_back(last_line);
	for(int ch = 0; ch < 3; ++ch)
	{
		B[ch].push_back(0.0);
		assert(B[ch].size() == S.size());
	}

	apply_gradient_matrix(S, B, height * width);
}

void image_compositor::build_matrices()
{
	/* (1) build interpolation matrix */
	std::puts("  Building interpolation lines...");
	for(int i = 0; i < height; ++i)
		for(int j = 0; j < width; ++j)
			interp.emplace_back(build_interp_line(i, j));

	std::puts("  Building matrix S and vector B...");
	std::vector<std::vector<std::pair<int, double>>> S;
	std::vector<double> B[3];
	for(int i = 0; i < height; ++i)
	{
		for(int j = 0; j < width; ++j)
		{
			for(int axis = 0; axis < 2; ++axis)
			{
				int ti = i - axis, tj = j - (1 - axis);
				if(ti < 0 || tj < 0) continue;

				// interpolation matrix
				std::map<int, double> line;
				for(mv_t mv : interp[i * width + j])
					line[mv.first] += mv.second;
				for(mv_t mv : interp[ti * width + tj])
					line[mv.first] -= mv.second;
				std::vector<std::pair<int, double>> vec_line;
				for(mv_t mv : line) vec_line.push_back(mv);
				S.emplace_back(std::move(vec_line));

				// B vector
				int z = z_index->get(i, j, 0);
				int z_t = z_index->get(ti, tj, 0);
				if(z != z_t)
				{
					int z_m = std::max(z, z_t) - 1;
					for(int ch = 0; ch < 3; ++ch)
					{
						int g0 = img_mixed->get(i, j, ch) - img_mixed->get(ti, tj, ch);
	//					int g1 = layers[z_m]->get_color(i, j, ch) - layers[z_m]->get_color(ti, tj, ch);
						int g1 = get_color(i, j, ch, z_m) - get_color(ti, tj, ch, z_m);
						B[ch].push_back(g1 - g0);
					}
				} else {
					for(int ch = 0; ch < 3; ++ch)
						B[ch].push_back(0.0);
				}
			}
		}
	}

	S.push_back(interp[height * width - 1]);
	for(int ch = 0; ch < 3; ++ch)
	{
		B[ch].push_back(0.0);
		assert(B[ch].size() == S.size());
	}

	apply_gradient_matrix(S, B, keypoints.size());
}

image_compositor::interp_line_t image_compositor::build_interp_line(int x, int y)
{
	interp_line_t line;
	auto it = keypoints.find( { x, y } );
	if(it != keypoints.end())
	{
		line.push_back( { it->second, 1.0 } );
		return line;
	}

	std::unordered_map<int, double> weight;

	quadtree_t *node = qtree->find(x, y);

	int X[4], Y[4];
	double W[4];
	double area = node->get_range() * node->get_range();
	X[0] = node->xl; Y[0] = node->yl;
	W[0] = (node->xr - x) * (node->yr - y) / area;
	X[1] = node->xl; Y[1] = node->yr;
	W[1] = (node->xr - x) * (y - node->yl) / area;
	X[2] = node->xr; Y[2] = node->yl;
	W[2] = (x - node->xl) * (node->yr - y) / area;
	X[3] = node->xr; Y[3] = node->yr;
	W[3] = (x - node->xl) * (y - node->yl) / area;

	for(int i = 0; i < 4; ++i)
	{
		if(W[i] < 1.0e-5)
			continue;

		auto it = keypoints.find( { X[i], Y[i] } );
		if(it != keypoints.end())
		{
			weight[it->second] += W[i];
		} else {
			auto interp_edge = [&](quadtree_t *n) {
				if(X[i] == n->xl || X[i] == n->xr)
				{
					auto it_l = keypoints.find( { X[i], n->yl } );
					auto it_r = keypoints.find( { X[i], n->yr } );
					if(it_l != keypoints.end() && it_r != keypoints.end())
					{
						double len = n->yr - n->yl;
						weight[it_r->second] += W[i] * (Y[i] - n->yl) / len;
						weight[it_l->second] += W[i] * (n->yr - Y[i]) / len;
					}
				}

				if(Y[i] == n->yl || Y[i] == n->yr)
				{
					auto it_l = keypoints.find( { n->xl, Y[i] } );
					auto it_r = keypoints.find( { n->xr, Y[i] } );
					if(it_l != keypoints.end() && it_r != keypoints.end())
					{
						double len = n->xr - n->xl;
						weight[it_r->second] += W[i] * (X[i] - n->xl) / len;
						weight[it_l->second] += W[i] * (n->xr - X[i]) / len;
					}
				}
			};

			interp_edge(qtree->find(X[i], Y[i]));
			interp_edge(qtree->find_outer(X[i], Y[i]));
		}
	}

	for(auto it : weight)
		line.push_back(it);

	return line;
}

void image_compositor::run(bool full_keypoings)
{
	std::puts("Building mixed image...");
	build_mixed_image();
	if(!full_keypoings)
	{
		std::puts("Calculating boundary...");
		build_boundary();
	}

	std::puts("Calculating matrices...");
	if(full_keypoings) build_full_matrices();
	else build_matrices();

	std::puts("Initializing solver...");
	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> solver;
	solver.compute(*StS);
	img_result = std::make_shared<image_t>(width, height, 3);

	for(int ch = 0; ch < 3; ++ch)
	{
		std::printf("Calculating channel %d...\n", ch + 1);
		int size = full_keypoings ? height * width : keypoints.size();
		std::vector<double> x(size, 0.0);
		Eigen::SparseVector<double> ans = solver.solve(*StB[ch]);
		for(Eigen::SparseVector<double>::InnerIterator it(ans); it; ++it)
			x[it.index()] = it.value();

		double mean = 0.0, max = -1.0e4, min = 1.0e4;
		std::vector<double> delta(height * width, 0.0);
		for(int i = 0; i < height; ++i)
		{
			for(int j = 0; j < width; ++j)
			{
				double val = 0.0;
				if(full_keypoings)
				{
					val = x[i * width + j];
				} else {
					for(mv_t mv : interp[i * width + j])
						val += x[mv.first] * mv.second;
				}
				delta[i * width + j] = val;
				mean += val;
				max = std::max(max, val);
				min = std::min(min, val);
			}
		}

		mean /= height * width;
		std::printf("mean = %.5lf\n", mean);
		for(int i = 0; i < height; ++i)
		{
			for(int j = 0; j < width; ++j)
			{
				double d = delta[i * width + j];
				int val = std::round(img_mixed->get(i, j, ch) + d - mean);
				val = std::max(0, std::min(255, val));
				img_result->get_ptr(i, j)[ch] = val;
				img_delta->get_ptr(i, j)[ch] = (d - min) / (max - min) * 255;
			}
		}
	}
}

void image_compositor::save_quadtree(const char *path)
{
	qtree->dump_to(path, width, height);
}

void image_compositor::save_mixed_image(const char *path)
{
	img_mixed->write(path);
}

void image_compositor::save_image(const char *path)
{
	img_result->write(path);
}

void image_compositor::save_delta_image(const char *path)
{
	img_delta->write(path);
}

void image_compositor::set_image_size(int w, int h)
{
	width = w, height = h;
}

void image_compositor::auto_image_size()
{
	width = height = 0;
	for(auto layer : layers)
	{
		width = std::max(layer->get_right(), width);
		height = std::max(layer->get_bottom(), height);
	}

	std::printf("Adjust image to %dx%d\n", width, height);
}

void image_compositor::add_layer(const char *image, const char *mask, int offset_x, int offset_y)
{
	auto layer = std::make_shared<layer_t>();
	layer->load(image, mask);
	layer->set_offset(offset_x, offset_y);
	layers.push_back(layer);
}
