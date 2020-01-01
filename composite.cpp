#include "composite.h"
#include "layer.h"
#include "quadtree.h"
#include "image.h"
#include <eigen3/Eigen/src/IterativeLinearSolvers/BasicPreconditioners.h>
#include <eigen3/Eigen/src/IterativeLinearSolvers/BiCGSTAB.h>
#include <memory>
#include <cmath>

void image_compositor::build_mixed_image()
{
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

	for(int ch = 0; ch < 3; ++ch)
		B[ch].push_back(0.0);
	S.push_back(interp[height * width - 1]);

	/* (2) compute StS */
	std::puts("  Computing sparse matrix StS...");
	int size = keypoints.size();
	StS = std::make_shared<Eigen::SparseMatrix<double>>(size, size);
	StS->setZero();
	for(auto &line : S)
	{
		for(auto mv1 : line)
			for(auto mv2 : line)
				StS->coeffRef(mv1.first, mv2.first) += mv1.second * mv2.second;
	}
	StS->makeCompressed();

	/* (3) compute StB */
	std::puts("  Computing sparse vectors StB...");
	for(int ch = 0; ch < 3; ++ch)
	{
		StB[ch] = std::make_shared<Eigen::SparseVector<double>>(size);
		for(int i = 0; i < (int)S.size(); ++i)
			for(auto mv : S[i])
				StB[ch]->coeffRef(mv.first) += mv.second * B[ch][i];
	}
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

	int X[4], Y[4];
	std::unordered_map<int, double> weight;

	quadtree_t *node = qtree->find(x, y);
	node->fill_boundary(X, Y);
	double area = node->get_range() * node->get_range();
	for(int i = 0; i < 4; ++i)
	{
		if(X[i] == x || Y[i] == y)
			continue;

		double w = std::abs((X[i] - x) * (Y[i] - y)) / area;
		auto it = keypoints.find( { X[i], Y[i] } );
		if(it != keypoints.end())
		{
			weight[it->second] += w;
		} else {
			auto interp_edge = [&](quadtree_t *n) {
				if(X[i] == n->xl || X[i] == n->xr)
				{
					auto it_l = keypoints.find( { X[i], n->yl } );
					auto it_r = keypoints.find( { X[i], n->yr } );
					if(it_l != keypoints.end() && it_r != keypoints.end())
					{
						double len = n->yr - n->yl;
						weight[it_l->second] += w * (Y[i] - n->yl) / len;
						weight[it_r->second] += w * (n->yr - Y[i]) / len;
					}
				}

				if(Y[i] == n->yl || Y[i] == n->yr)
				{
					auto it_l = keypoints.find( { n->xl, Y[i] } );
					auto it_r = keypoints.find( { n->xr, Y[i] } );
					if(it_l != keypoints.end() && it_r != keypoints.end())
					{
						double len = n->xr - n->xl;
						weight[it_l->second] += w * (X[i] - n->xl) / len;
						weight[it_r->second] += w * (n->xr - X[i]) / len;
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

void image_compositor::run()
{
	std::puts("Building mixed image...");
	build_mixed_image();
	std::puts("Calculating boundary...");
	build_boundary();
	std::puts("Calculating matrices...");
	build_matrices();

	std::puts("Initializing solver...");
	Eigen::BiCGSTAB<Eigen::SparseMatrix<double>, Eigen::IdentityPreconditioner> solver;
	solver.compute(*StS);
	img_result = std::make_shared<image_t>(width, height, 3);

	for(int ch = 0; ch < 3; ++ch)
	{
		std::printf("Calculating channel %d...\n", ch + 1);
		std::vector<double> x(keypoints.size(), 0.0);
		Eigen::SparseVector<double> ans = solver.solve(*StB[ch]);
		for(Eigen::SparseVector<double>::InnerIterator it(ans); it; ++it)
			x[it.index()] = it.value();

		double mean = 0.0;
		std::vector<double> delta(height * width, 0.0);
		for(int i = 0; i < height; ++i)
		{
			for(int j = 0; j < width; ++j)
			{
				double val = 0.0;
				for(mv_t mv : interp[i * width + j])
					val += x[mv.first] * mv.second;
				delta[i * width + j] = val;
				mean += val;
			}
		}

		mean /= height * width;
		std::printf("mean = %.5lf\n", mean);
		for(int i = 0; i < height; ++i)
		{
			for(int j = 0; j < width; ++j)
			{
				int val = std::round(img_mixed->get(i, j, ch) + delta[i * width + j] - mean);
				val = std::max(0, std::min(255, val));
				img_result->get_ptr(i, j)[ch] = val;
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

void image_compositor::set_image_size(int w, int h)
{
	width = w, height = h;
}

void image_compositor::add_layer(const char *image, const char *mask, int offset_x, int offset_y)
{
	auto layer = std::make_shared<layer_t>();
	layer->load(image, mask);
	layer->set_offset(offset_x, offset_y);
	layers.push_back(layer);
}
