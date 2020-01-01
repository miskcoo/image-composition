#ifndef __COMPOSITE_H__
#define __COMPOSITE_H__

#include <vector>
#include <memory>
#include <utility>
#include <map>
#include <unordered_map>
#include <eigen3/Eigen/Sparse>
#include "quadtree.h"
#include "image.h"
#include "layer.h"

class image_compositor
{
private:
	using point_t = std::pair<int, int>;
	using mv_t = std::pair<int, double>;
	using interp_line_t = std::vector<mv_t>;

	int width, height;
	std::shared_ptr<quadtree_t> qtree;
	std::shared_ptr<image_t> img_mixed, z_index, img_result;
	std::vector<interp_line_t> interp;
	std::map<point_t, int> keypoints;
	std::shared_ptr<Eigen::SparseMatrix<double>> StS;
	std::shared_ptr<Eigen::SparseVector<double>> StB[3];

	void apply_gradient_matrix(const std::vector<interp_line_t>& S, const std::vector<double> B[3], int size);
	void build_mixed_image();
	void build_boundary();
	void build_matrices();
	void build_full_matrices();
	interp_line_t build_interp_line(int x, int y);
	uint8_t get_color(int x, int y, int ch, int ignore_z);

private:
	std::vector<std::shared_ptr<layer_t>> layers;

public:
	void run(bool full_keypoings = false);
	void save_quadtree(const char *path);
	void save_image(const char *path);
	void save_mixed_image(const char *path);
	void save_delta_image(const char *path);
	void set_image_size(int w, int h);
	void add_layer(const char *image, const char *mask, int offset_x = 0, int offset_y = 0);
};

#endif
