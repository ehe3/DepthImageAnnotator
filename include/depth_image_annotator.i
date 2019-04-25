%module depth_image_annotator
%{
  #include "depth_image_annotator.h"
%}

%{
#define SWIG_FILE_WITH_INIT
%}

%include "numpy.i"

%init %{
  import_array();
%}

%apply (float* ARGOUT_ARRAY1, int DIM1) {(float* currentdt, int n)}
%apply (float* IN_ARRAY1, int DIM1) {(float* depth_data, int n)}

%include "depth_image_annotator.h"
