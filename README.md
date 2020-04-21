# c3d-to-bvh

Converts c3d files into bvh.

Right now, only lower-body Vicon marker sets are supported. (Motion Analysis marker set: TBD).

The c3d file should have these market sets:
```
SACR, LASI, RASI, LTHI, RTHI, LKNE, RKNE, LTIB, RTIB, LANK, RANK, LTOE, RTOE
```

## To run

IMPORTANT: this project uses submodules! Make sure to include the `--recursive` flag.

You should also use a C++17 compiler with the std::filesystem library included.

```
git clone https://github.com/snumrl/c3d-to-bvh --recursive
cd c3d-to-bvh
mkdir build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

Now you will have the three executables inside the `build/src` folder: bvh_converter, bvh_converter_test, bvh_viewer.

## bvh_converter

Converts a folder with bvh files to c3d.

```
bvh_converter <input_bvh_folder> <output_c3d_folder>
```

## bvh_converter_test

Test program to see if the algorithm for bvh conversion works.

```
bvh_converter_test vicon <c3d to convert> <reference pose c3d>
```

## bvh_viewer

Views bvh and c3d files.

```
bvh_viewer <filename>
```

## License (MIT)

Copyright 2020 Philsik Chang

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
