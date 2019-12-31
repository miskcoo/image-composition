#ifndef __QUADTREE_H__
#define __QUADTREE_H__

class quadtree_t
{
    int range;
    int xl, xr, yl, yr;
    quadtree_t *s_ll, *s_lr, *s_rl, *s_rr;

    void _split();
    static void _split_tree(quadtree_t *root, int x, int y, int range);
    quadtree_t *_find_child(int x, int y);
public:
    quadtree_t(int xl, int xr, int yl, int yr);
    ~quadtree_t();

    void split(int x, int y, int range = 1);
    quadtree_t *find(int x, int y);
	bool in_range(int x, int y);
    bool is_leaf() const;

	void dump_to(const char* filename, int width, int height);

    template<typename Callback>
    void traverse(const Callback &callback)
    {
        if(is_leaf())
        {
            callback(xl, xr, yl, yr);
        } else {
            s_ll->traverse(callback);
            s_lr->traverse(callback);
            s_rl->traverse(callback);
            s_rr->traverse(callback);
        }
    }
};

#endif
