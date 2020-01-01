#include "composite.h"
#include <cstdlib>
#include <memory>

int main(int argc, char *argv[])
{
	// auto compositor = std::make_shared<image_compositor>();
	// compositor->add_layer("images/rainier/image_1.jpg", "images/rainier/mask_1.jpg");
	// compositor->add_layer("images/rainier/image_2.jpg", "images/rainier/mask_2.jpg");
	// compositor->add_layer("images/rainier/image_3.jpg", "images/rainier/mask_3.jpg");
	// compositor->set_image_size(9316, 1782);
	// compositor->run();
	// compositor->save_mixed_image("images/rainier/mixed.png");
	// compositor->save_quadtree("images/rainier/quadtree.png");
	// compositor->save_image("images/rainier/result.png");
	auto compositor = std::make_shared<image_compositor>();
	compositor->add_layer("images/hand-eye/test2_target.png", nullptr);
	compositor->add_layer("images/hand-eye/test2_src.png", nullptr, // "images/hand-eye/test2_mask.png",
			std::atoi(argv[1]), std::atoi(argv[2]));
	compositor->set_image_size(418, 356);
	compositor->run();
	compositor->save_mixed_image("images/hand-eye/mixed.png");
	compositor->save_delta_image("images/hand-eye/delta.png");
	compositor->save_quadtree("images/hand-eye/quadtree.png");
	compositor->save_image("images/hand-eye/result.png");
	return 0;
}
