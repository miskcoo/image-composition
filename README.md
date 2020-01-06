# Image Composition

The project of "Media Computing" of Tsinghua University, Fall 2019.

A C++ implementation of the following paper:

Agarwala, Aseem. "Efficient gradient-domain compositing using quadtrees." _ACM Transactions on Graphics (TOG)._ Vol. 26. No. 3. ACM, 2007.

## Usage

You can simply use the following command to compile this project
```bash
g++ *.cpp -o composite -O2
```

And then run by
```bash
./composite <directory>
```
where `<directory>` is the directory of your images and masks. You need to create a configuration file `layers.conf` on `<directory>`. This configuration file contains multiple lines, each of which has 4 components separated by whitespace describing an image and its mask:
```
<image path> <mask path> <offset_x> <offset_y>
```

You can find an example in `images/hand-eye`.

## Example

For example, if you want to composite the following two images:

![](images/hand-eye/test2_src.png)![](images/hand-eye/test2_mask.png)

![](images/hand-eye/test2_target.png)

The configuration `layers.conf` is:

```plain
test2_target.png NULL 0 0
test2_src.png test2_mask.png 160 140
```

The result image is

![](images/hand-eye/result.png)

and the corresponding quadtree and delta map are

![](images/hand-eye/quadtree.png)![](images/hand-eye/delta.png)
