#Makula#

Makula (hopefully) will be a library for recognizing and learning complex shapes in "real time".
It is mostly based on the Point Cloud Library (http://www.pointclouds.org) and the MRPT (http://www.mrpt.org). 
Currently I am using the version 1.7.0 (trunk), because of its support for CUDA and GPUs. 

##Roadmap##

- [ ] **Give me eyes:** Buffered streaming of point clouds. The current OpenNI grabber takes me a bit to much CPU.
- [ ] **Divide and Conquer:** Segmentation of a point cloud and not losing sight of "real time".
- [ ] **Knowledgebase:** Store segmented objects and its characteristics(local features, keypoints) in a database.
- [ ] **Learn to forget:** Keep memory clean. Delete bad records, when they seem useless.
- [ ] **I spy with my little eye...:** Recognize already learned objects.

## Copyright ##

Copyright (c) 2013 Sebastian Krieger "sebastian-krieger@online.de"

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
