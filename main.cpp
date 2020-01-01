#include "composite.h"
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <memory>
#include <string>

int main(int argc, char *argv[])
{
	std::string prefix = argv[1];
	prefix += "/";
	auto compositor = std::make_shared<image_compositor>();

	std::ifstream ifs(prefix + "layers.conf");
	int offset_x, offset_y;
	std::string image_name, mask_name;
	while(ifs >> image_name >> mask_name >> offset_x >> offset_y)
	{
		auto image_path = prefix + image_name;
		auto mask_path = prefix + mask_name;
		compositor->add_layer(
			image_path.c_str(),
			mask_name == "NULL" ? nullptr : mask_path.c_str(),
			offset_x, offset_y
		);
	}

	compositor->auto_image_size();

	bool use_full_matrix = false;
	if(argc == 3) use_full_matrix = true;

	auto t1 = std::clock();
	compositor->run(use_full_matrix);
	auto t2 = std::clock();

	compositor->save_delta_image((prefix + "delta.png").c_str());
	compositor->save_mixed_image((prefix + "mixed.png").c_str());
	compositor->save_quadtree((prefix + "quadtree.png").c_str());
	compositor->save_image((prefix + "result.png").c_str());

	std::printf("Elasped time: %.3lfs\n", (t2 - t1) / double(CLOCKS_PER_SEC));
	return 0;
}
