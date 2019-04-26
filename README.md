0. Check to see if the src/vendor folder exists. It is excluded by gitignore because it is large. src/vendor is the same src/vendor in Monocle. 

1. Make sure SWIG is installed on the system:
sudo apt install swig

2. Navigate to the include directory and run SWIG
cd include
swig -c++ -python depth_image_annotator.i

This should generate two files: depth_image_annotator.py and depth_image_annotator_wrap.cxx

3. Use g++ to compile wrapper into a shared library
Issue the following command:
g++ -fPIC -shared depth_image_annotator_wrap.cxx Reinitializer.cpp \
		-I/home/eric/.conda/envs/py37/include/python3.7m (python include directory \ python-config --cflags) \
		-I/home/eric/anaconda3/lib/python3.7/site-packages/numpy/core/include (numpy include directory \ np.get_include())
		-I/home/eric/Dev/DepthImageAnnotator/src (project source) \
		-I/home/eric/Dev/DepthImageAnnotator/include (project include) \
		-I/home/eric/Dev/DepthImageAnnotator/src/vendor (project vendor) \
		-I/usr/local/include/torch/csrc/api/include (Torch headers) \ 
		-I/usr/local/include/opencv4 (OpenCV headers) \
		-o _depth_image_annotator.so \
		-L/usr/local/lib \ 
		-L/usr/lib64 \ 
		-lGLEW -lGLU -lm -lGL -lm -lpthread -pthread -ldl -ldrm -lXdamage -lXfixes -lX11-xcb -lxcb-glx -lxcb-dri2 -lXxf86vm -lXext -lX11 -lpthread -lxcb -lXau -lXdmcp (for GLEW) \
		-ltorch (for Torch) \
		-lopencv_core -lopencv_highgui -lopencv_imgcodecs (for OpenCV) \
		-lglfw3 -lrt -lm -ldl -lXrandr -lXinerama -lXxf86vm -lXext -lXcursor -lXrender -lXfixes -lX11 -lpthread -lxcb -lXau -lXdmcp (for GLFW) \ 
		-lassimp -lstdc++ -lz (for ASSIMP)
		-lopencv_dnn -lopencv_ml -lopencv_objdetect -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_imgproc -lopencv_flann -lopencv_core -ldl -lm -lpthread -lrt -ltbb

On one line: 
g++ -fPIC -shared depth_image_annotator_wrap.cxx Reinitializer.cpp -I/home/eric/.conda/envs/py37/include/python3.6m -I/home/eric/.local/lib/python3.6/site-packages/numpy/core/include -I/home/eric/Dev/DepthImageAnnotator/src -I/home/eric/Dev/DepthImageAnnotator/include -I/home/eric/Dev/DepthImageAnnotator/src/vendor -I/usr/local/include/torch/csrc/api/include -I/usr/local/include/opencv4 -o _depth_image_annotator.so -L/usr/lib64 -L/usr/local/lib -lGLEW -lGLU -lm -lGL -lm -lpthread -pthread -ldl -ldrm -lXdamage -lXfixes -lX11-xcb -lxcb-glx -lxcb-dri2 -lXxf86vm -lXext -lX11 -lpthread -lxcb -lXau -lXdmcp -ltorch -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lglfw3 -lrt -lm -ldl -lXrandr -lXinerama -lXxf86vm -lXext -lXcursor -lXrender -lXfixes -lX11 -lpthread -lxcb -lXau -lXdmcp -lassimp -lstdc++ -lz -lopencv_dnn -lopencv_ml -lopencv_objdetect -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_imgproc -lopencv_flann -lopencv_core -ldl -lm -lpthread -lrt -ltbb

The file _depth_image_annotator.so should be created.

4. Run python
The test file should now work.
python3 dia_test.py

5. Possible issues
If certain library files aren't found, try explicitly adding them to LD_LIBRARY_PATH, i.e.
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

For Andrew:
g++ -fPIC -shared depth_image_annotator_wrap.cxx Reinitializer.cpp -I/home/monocle/anaconda3/envs/py37/include/python3.7m -I/home/monocle/anaconda3/lib/python3.7/site-packages/numpy/core/include -I/home/monocle/Dev/DepthImageAnnotator/src -I/home/monocle/Dev/DepthImageAnnotator/include -I/home/monocle/Dev/DepthImageAnnotator/src/vendor -I/usr/local/include/torch/csrc/api/include -I/usr/local/include/opencv4 -o _depth_image_annotator.so -L/usr/lib64 -L/usr/local/lib -lGLEW -lGLU -lm -lGL -lm -lpthread -pthread -ldl -ldrm -lXdamage -lXfixes -lX11-xcb -lxcb-glx -lxcb-dri2 -lXxf86vm -lXext -lX11 -lpthread -lxcb -lXau -lXdmcp -ltorch -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lglfw3 -lrt -lm -ldl -lXrandr -lXinerama -lXxf86vm -lXext -lXcursor -lXrender -lXfixes -lX11 -lpthread -lxcb -lXau -lXdmcp -lassimp -lstdc++ -lz -L/usr/local/cuda-10.0/lib64 -lnvrtc -L/usr/local/cuda-10.0/lib64 -lcuda -L/usr/local/cuda-10.0/lib64 -lcudart -lopencv_dnn -lopencv_ml -lopencv_objdetect -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_imgproc -lopencv_flann -lopencv_core -ldl -lm -lpthread -lrt -ltbb


