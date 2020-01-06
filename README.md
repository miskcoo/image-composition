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

You can find an example in `examples`.
