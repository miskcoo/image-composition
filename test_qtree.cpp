#include "quadtree.h"

int main()
{
	quadtree_t qtree(0, 512, 0, 512);
	for(int i = 0; i < 512; ++i)
		qtree.split(256, i);
	for(int i = 0; i < 256; ++i)
		qtree.split(i, 511);
	qtree.dump_to("qtree.png", 512, 512);
	return 0;
}
