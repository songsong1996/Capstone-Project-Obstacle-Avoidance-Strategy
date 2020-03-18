

//-----------------------------------------------------------------------------------------------------------------------------------------------
//descriptor
/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Andreas Geiger

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

// NOTE: This descripter is a sparse approximation to the 50-dimensional
// descriptor described in the paper. It produces similar results, but
// is faster to compute.

#ifndef __DESCRIPTOR_H__
#define __DESCRIPTOR_H__

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// Define fixed-width datatypes for Visual Studio projects
#ifndef _MSC_VER
  #include <stdint.h>
#else
  typedef __int8            int8_t;
  typedef __int16           int16_t;
  typedef __int32           int32_t;
  typedef __int64           int64_t;
  typedef unsigned __int8   uint8_t;
  typedef unsigned __int16  uint16_t;
  typedef unsigned __int32  uint32_t;
  typedef unsigned __int64  uint64_t;
#endif

class Descriptor {
  
public:
  
  // constructor creates filters
  Descriptor(uint8_t* I,int32_t width,int32_t height,int32_t bpl,bool half_resolution);
  
  // deconstructor releases memory
  ~Descriptor();
  
  // descriptors accessible from outside
  uint8_t* I_desc;
  
private:

  // build descriptor I_desc from I_du and I_dv
  void createDescriptor(uint8_t* I_du,uint8_t* I_dv,int32_t width,int32_t height,int32_t bpl,bool half_resolution);

};

#endif






//-----------------------------------------------------------------------------------------------------------------------------------------------
//ELAS

/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Andreas Geiger

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

// Main header file. Include this to use libelas in your code.

#ifndef __ELAS_H__
#define __ELAS_H__

#include <vector>
#include <emmintrin.h>

// define fixed-width datatypes for Visual Studio projects
#ifndef _MSC_VER
#include <stdint.h>
#else
typedef __int8            int8_t;
  typedef __int16           int16_t;
  typedef __int32           int32_t;
  typedef __int64           int64_t;
  typedef unsigned __int8   uint8_t;
  typedef unsigned __int16  uint16_t;
  typedef unsigned __int32  uint32_t;
  typedef unsigned __int64  uint64_t;
#endif

#ifdef PROFILE

#endif

class Elas {

public:

    enum setting {ROBOTICS,MIDDLEBURY};

    // parameter settings
    struct parameters {
        int32_t disp_min;               // min disparity
        int32_t disp_max;               // max disparity
        float   support_threshold;      // max. uniqueness ratio (best vs. second best support match)
        int32_t support_texture;        // min texture for support points
        int32_t candidate_stepsize;     // step size of regular grid on which support points are matched
        int32_t incon_window_size;      // window size of inconsistent support point check
        int32_t incon_threshold;        // disparity similarity threshold for support point to be considered consistent
        int32_t incon_min_support;      // minimum number of consistent support points
        bool    add_corners;            // add support points at image corners with nearest neighbor disparities
        int32_t grid_size;              // size of neighborhood for additional support point extrapolation
        float   beta;                   // image likelihood parameter
        float   gamma;                  // prior constant
        float   sigma;                  // prior sigma
        float   sradius;                // prior sigma radius
        int32_t match_texture;          // min texture for dense matching
        int32_t lr_threshold;           // disparity threshold for left/right consistency check
        float   speckle_sim_threshold;  // similarity threshold for speckle segmentation
        int32_t speckle_size;           // maximal size of a speckle (small speckles get removed)
        int32_t ipol_gap_width;         // interpolate small gaps (left<->right, top<->bottom)
        bool    filter_median;          // optional median filter (approximated)
        bool    filter_adaptive_mean;   // optional adaptive mean filter (approximated)
        bool    postprocess_only_left;  // saves time by not postprocessing the right image
        bool    subsampling;            // saves time by only computing disparities for each 2nd pixel
        // note: for this option D1 and D2 must be passed with size
        //       width/2 x height/2 (rounded towards zero)

        // constructor
        parameters (setting s=ROBOTICS) {

          // default settings in a robotics environment
          // (do not produce results in half-occluded areas
          //  and are a bit more robust towards lighting etc.)
          if (s==ROBOTICS) {
            disp_min              = 0;
            disp_max              = 255;
            support_threshold     = 0.85;
            support_texture       = 10;
            candidate_stepsize    = 5;
            incon_window_size     = 5;
            incon_threshold       = 5;
            incon_min_support     = 5;
            add_corners           = 0;
            grid_size             = 20;
            beta                  = 0.02;
            gamma                 = 3;
            sigma                 = 1;
            sradius               = 2;
            match_texture         = 1;
            lr_threshold          = 2;
            speckle_sim_threshold = 1;
            speckle_size          = 200;
            ipol_gap_width        = 3;
            filter_median         = 0;
            filter_adaptive_mean  = 1;
            postprocess_only_left = 1;
            subsampling           = 0;

            // default settings for middlebury benchmark
            // (interpolate all missing disparities)
          } else {
            disp_min              = 0;
            disp_max              = 255;
            support_threshold     = 0.95;
            support_texture       = 10;
            candidate_stepsize    = 5;
            incon_window_size     = 5;
            incon_threshold       = 5;
            incon_min_support     = 5;
            add_corners           = 1;
            grid_size             = 20;
            beta                  = 0.02;
            gamma                 = 5;
            sigma                 = 1;
            sradius               = 3;
            match_texture         = 0;
            lr_threshold          = 2;
            speckle_sim_threshold = 1;
            speckle_size          = 200;
            ipol_gap_width        = 5000;
            filter_median         = 1;
            filter_adaptive_mean  = 0;
            postprocess_only_left = 0;
            subsampling           = 0;
          }
        }
    };

    // constructor, input: parameters
    Elas (parameters param) : param(param) {}

    // deconstructor
    ~Elas () {}

    // matching function
    // inputs: pointers to left (I1) and right (I2) intensity image (uint8, input)
    //         pointers to left (D1) and right (D2) disparity image (float, output)
    //         dims[0] = width of I1 and I2
    //         dims[1] = height of I1 and I2
    //         dims[2] = bytes per line (often equal to width, but allowed to differ)
    //         note: D1 and D2 must be allocated before (bytes per line = width)
    //               if subsampling is not active their size is width x height,
    //               otherwise width/2 x height/2 (rounded towards zero)
    void process (uint8_t* I1,uint8_t* I2,float* D1,float* D2,const int32_t* dims);

private:

    struct support_pt {
        int32_t u;
        int32_t v;
        int32_t d;
        support_pt(int32_t u,int32_t v,int32_t d):u(u),v(v),d(d){}
    };

    struct triangle {
        int32_t c1,c2,c3;
        float   t1a,t1b,t1c;
        float   t2a,t2b,t2c;
        triangle(int32_t c1,int32_t c2,int32_t c3):c1(c1),c2(c2),c3(c3){}
    };

    inline uint32_t getAddressOffsetImage (const int32_t& u,const int32_t& v,const int32_t& width) {
      return v*width+u;
    }

    inline uint32_t getAddressOffsetGrid (const int32_t& x,const int32_t& y,const int32_t& d,const int32_t& width,const int32_t& disp_num) {
      return (y*width+x)*disp_num+d;
    }

    // support point functions
    void removeInconsistentSupportPoints (int16_t* D_can,int32_t D_can_width,int32_t D_can_height);
    void removeRedundantSupportPoints (int16_t* D_can,int32_t D_can_width,int32_t D_can_height,
                                       int32_t redun_max_dist, int32_t redun_threshold, bool vertical);
    void addCornerSupportPoints (std::vector<support_pt> &p_support);
    inline int16_t computeMatchingDisparity (const int32_t &u,const int32_t &v,uint8_t* I1_desc,uint8_t* I2_desc,const bool &right_image);
    std::vector<support_pt> computeSupportMatches (uint8_t* I1_desc,uint8_t* I2_desc);

    // triangulation & grid
    std::vector<triangle> computeDelaunayTriangulation (std::vector<support_pt> p_support,int32_t right_image);
    void computeDisparityPlanes (std::vector<support_pt> p_support,std::vector<triangle> &tri,int32_t right_image);
    void createGrid (std::vector<support_pt> p_support,int32_t* disparity_grid,int32_t* grid_dims,bool right_image);

    // matching
    inline void updatePosteriorMinimum (__m128i* I2_block_addr,const int32_t &d,const int32_t &w,
                                        const __m128i &xmm1,__m128i &xmm2,int32_t &val,int32_t &min_val,int32_t &min_d);
    inline void updatePosteriorMinimum (__m128i* I2_block_addr,const int32_t &d,
                                        const __m128i &xmm1,__m128i &xmm2,int32_t &val,int32_t &min_val,int32_t &min_d);
    inline void findMatch (int32_t &u,int32_t &v,float &plane_a,float &plane_b,float &plane_c,
                           int32_t* disparity_grid,int32_t *grid_dims,uint8_t* I1_desc,uint8_t* I2_desc,
                           int32_t *P,int32_t &plane_radius,bool &valid,bool &right_image,float* D);
    void computeDisparity (std::vector<support_pt> p_support,std::vector<triangle> tri,int32_t* disparity_grid,int32_t* grid_dims,
                           uint8_t* I1_desc,uint8_t* I2_desc,bool right_image,float* D);

    // L/R consistency check
    void leftRightConsistencyCheck (float* D1,float* D2);

    // postprocessing
    void removeSmallSegments (float* D);
    void gapInterpolation (float* D);

    // optional postprocessing
    void adaptiveMean (float* D);
    void median (float* D);

    // parameter set
    parameters param;

    // memory aligned input images + dimensions
    uint8_t *I1,*I2;
    int32_t width,height,bpl;

    // profiling timer
#ifdef PROFILE
    Timer timer;
#endif
};

#endif



//-----------------------------------------------------------------------------------------------------------------------------------------------
//filter
/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Julius Ziegler, Andreas Geiger

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

#ifndef __FILTER_H__
#define __FILTER_H__

#include <pmmintrin.h>

// define fixed-width datatypes for Visual Studio projects
#ifndef _MSC_VER
#include <stdint.h>
#else
typedef __int8            int8_t;
  typedef __int16           int16_t;
  typedef __int32           int32_t;
  typedef __int64           int64_t;
  typedef unsigned __int8   uint8_t;
  typedef unsigned __int16  uint16_t;
  typedef unsigned __int32  uint32_t;
  typedef unsigned __int64  uint64_t;
#endif

// fast filters: implements 3x3 and 5x5 sobel filters and
//               5x5 blob and corner filters based on SSE2/3 instructions
namespace filter {

    // private namespace, public user functions at the bottom of this file
    namespace detail {
        void integral_image( const uint8_t* in, int32_t* out, int w, int h );
        void unpack_8bit_to_16bit( const __m128i a, __m128i& b0, __m128i& b1 );
        void pack_16bit_to_8bit_saturate( const __m128i a0, const __m128i a1, __m128i& b );

        // convolve image with a (1,4,6,4,1) row vector. Result is accumulated into output.
        // output is scaled by 1/128, then clamped to [-128,128], and finally shifted to [0,255].
        void convolve_14641_row_5x5_16bit( const int16_t* in, uint8_t* out, int w, int h );

        // convolve image with a (1,2,0,-2,-1) row vector. Result is accumulated into output.
        // This one works on 16bit input and 8bit output.
        // output is scaled by 1/128, then clamped to [-128,128], and finally shifted to [0,255].
        void convolve_12021_row_5x5_16bit( const int16_t* in, uint8_t* out, int w, int h );

        // convolve image with a (1,2,1) row vector. Result is accumulated into output.
        // This one works on 16bit input and 8bit output.
        // output is scaled by 1/4, then clamped to [-128,128], and finally shifted to [0,255].
        void convolve_121_row_3x3_16bit( const int16_t* in, uint8_t* out, int w, int h );

        // convolve image with a (1,0,-1) row vector. Result is accumulated into output.
        // This one works on 16bit input and 8bit output.
        // output is scaled by 1/4, then clamped to [-128,128], and finally shifted to [0,255].
        void convolve_101_row_3x3_16bit( const int16_t* in, uint8_t* out, int w, int h );

        void convolve_cols_5x5( const unsigned char* in, int16_t* out_v, int16_t* out_h, int w, int h );

        void convolve_col_p1p1p0m1m1_5x5( const unsigned char* in, int16_t* out, int w, int h );

        void convolve_row_p1p1p0m1m1_5x5( const int16_t* in, int16_t* out, int w, int h );

        void convolve_cols_3x3( const unsigned char* in, int16_t* out_v, int16_t* out_h, int w, int h );
    }

    void sobel3x3( const uint8_t* in, uint8_t* out_v, uint8_t* out_h, int w, int h );

    void sobel5x5( const uint8_t* in, uint8_t* out_v, uint8_t* out_h, int w, int h );

    // -1 -1  0  1  1
    // -1 -1  0  1  1
    //  0  0  0  0  0
    //  1  1  0 -1 -1
    //  1  1  0 -1 -1
    void checkerboard5x5( const uint8_t* in, int16_t* out, int w, int h );

    // -1 -1 -1 -1 -1
    // -1  1  1  1 -1
    // -1  1  8  1 -1
    // -1  1  1  1 -1
    // -1 -1 -1 -1 -1
    void blob5x5( const uint8_t* in, int16_t* out, int w, int h );
};

#endif



//-----------------------------------------------------------------------------------------------------------------------------------------------
//image
/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Andreas Geiger

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

// basic image I/O, based on Pedro Felzenszwalb's code

#ifndef IMAGE_H
#define IMAGE_H

#include <cstdlib>
#include <climits>
#include <cstring>
#include <fstream>

// use imRef to access image data.
#define imRef(im, x, y) (im->access[y][x])

// use imPtr to get pointer to image data.
#define imPtr(im, x, y) &(im->access[y][x])

#define BUF_SIZE 256

typedef unsigned char uchar;
typedef struct { uchar r, g, b; } rgb;

inline bool operator==(const rgb &a, const rgb &b) {
  return ((a.r == b.r) && (a.g == b.g) && (a.b == b.b));
}

// image class
template <class T> class image {
public:

    // create image
    image(const int width, const int height, const bool init = false);

    // delete image
    ~image();

    // init image
    void init(const T &val);

    // deep copy
    image<T> *copy() const;

    // get image width/height
    int width() const { return w; }
    int height() const { return h; }

    // image data
    T *data;

    // row pointers
    T **access;

private:
    int w, h;
};

template <class T> image<T>::image(const int width, const int height, const bool init) {
  w = width;
  h = height;
  data = new T[w * h];  // allocate space for image data
  access = new T*[h];   // allocate space for row pointers

  // initialize row pointers
  for (int i = 0; i < h; i++)
    access[i] = data + (i * w);

  // init to zero
  if (init)
    memset(data, 0, w * h * sizeof(T));
}

template <class T> image<T>::~image() {
  delete [] data;
  delete [] access;
}

template <class T> void image<T>::init(const T &val) {
  T *ptr = imPtr(this, 0, 0);
  T *end = imPtr(this, w-1, h-1);
  while (ptr <= end)
    *ptr++ = val;
}


template <class T> image<T> *image<T>::copy() const {
  image<T> *im = new image<T>(w, h, false);
  memcpy(im->data, data, w * h * sizeof(T));
  return im;
}

class pnm_error {};

void pnm_read(std::ifstream &file, char *buf) {
  char doc[BUF_SIZE];
  char c;

  file >> c;
  while (c == '#') {
    file.getline(doc, BUF_SIZE);
    file >> c;
  }
  file.putback(c);

  file.width(BUF_SIZE);
  file >> buf;
  file.ignore();
}

image<uchar> *loadPGM(const char *name) {
  char buf[BUF_SIZE];

  // read header
  std::ifstream file(name, std::ios::in | std::ios::binary);
  pnm_read(file, buf);
  if (strncmp(buf, "P5", 2)) {
    std::cout << "ERROR: Could not read file " << name << std::endl;
    throw pnm_error();
  }

  pnm_read(file, buf);
  int width = atoi(buf);
  pnm_read(file, buf);
  int height = atoi(buf);

  pnm_read(file, buf);
  if (atoi(buf) > UCHAR_MAX) {
    std::cout << "ERROR: Could not read file " << name << std::endl;
    throw pnm_error();
  }

  // read data
  image<uchar> *im = new image<uchar>(width, height);
  file.read((char *)imPtr(im, 0, 0), width * height * sizeof(uchar));

  return im;
}

void savePGM(image<uchar> *im, const char *name) {
  int width = im->width();
  int height = im->height();
  std::ofstream file(name, std::ios::out | std::ios::binary);

  file << "P5\n" << width << " " << height << "\n" << UCHAR_MAX << "\n";
  file.write((char *)imPtr(im, 0, 0), width * height * sizeof(uchar));
}

#endif





//-----------------------------------------------------------------------------------------------------------------------------------------------
//matrix
/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

#ifndef MATRIX_H
#define MATRIX_H


#ifndef _MSC_VER
#include <stdint.h>
#else
typedef __int8            int8_t;
  typedef __int16           int16_t;
  typedef __int32           int32_t;
  typedef __int64           int64_t;
  typedef unsigned __int8   uint8_t;
  typedef unsigned __int16  uint16_t;
  typedef unsigned __int32  uint32_t;
  typedef unsigned __int64  uint64_t;
#endif

#define endll endl << endl // double end line definition

typedef double FLOAT;      // double precision
//typedef float  FLOAT;    // single precision

class Matrix {

public:

    // constructor / deconstructor
    Matrix ();                                                  // init empty 0x0 matrix
    Matrix (const int32_t m,const int32_t n);                   // init empty mxn matrix
    Matrix (const int32_t m,const int32_t n,const FLOAT* val_); // init mxn matrix with values from array 'val'
    Matrix (const Matrix &M);                                   // creates deepcopy of M
    ~Matrix ();

    // assignment operator, copies contents of M
    Matrix& operator= (const Matrix &M);

    // copies submatrix of M into array 'val', default values copy whole row/column/matrix
    void getData(FLOAT* val_,int32_t i1=0,int32_t j1=0,int32_t i2=-1,int32_t j2=-1);

    // set or get submatrices of current matrix
    Matrix getMat(int32_t i1,int32_t j1,int32_t i2=-1,int32_t j2=-1);
    void   setMat(const Matrix &M,const int32_t i,const int32_t j);

    // set sub-matrix to scalar (default 0), -1 as end replaces whole row/column/matrix
    void setVal(FLOAT s,int32_t i1=0,int32_t j1=0,int32_t i2=-1,int32_t j2=-1);

    // set (part of) diagonal to scalar, -1 as end replaces whole diagonal
    void setDiag(FLOAT s,int32_t i1=0,int32_t i2=-1);

    // clear matrix
    void zero();

    // extract columns with given index
    Matrix extractCols (std::vector<int> idx);

    // create identity matrix
    static Matrix eye (const int32_t m);
    void          eye ();

    // create diagonal matrix with nx1 or 1xn matrix M as elements
    static Matrix diag(const Matrix &M);

    // returns the m-by-n matrix whose elements are taken column-wise from M
    static Matrix reshape(const Matrix &M,int32_t m,int32_t n);

    // create 3x3 rotation matrices (convention: http://en.wikipedia.org/wiki/Rotation_matrix)
    static Matrix rotMatX(const FLOAT &angle);
    static Matrix rotMatY(const FLOAT &angle);
    static Matrix rotMatZ(const FLOAT &angle);

    // simple arithmetic operations
    Matrix  operator+ (const Matrix &M); // add matrix
    Matrix  operator- (const Matrix &M); // subtract matrix
    Matrix  operator* (const Matrix &M); // multiply with matrix
    Matrix  operator* (const FLOAT &s);  // multiply with scalar
    Matrix  operator/ (const Matrix &M); // divide elementwise by matrix (or vector)
    Matrix  operator/ (const FLOAT &s);  // divide by scalar
    Matrix  operator- ();                // negative matrix
    Matrix  operator~ ();                // transpose
    FLOAT   l2norm ();                   // euclidean norm (vectors) / frobenius norm (matrices)
    FLOAT   mean ();                     // mean of all elements in matrix

    // complex arithmetic operations
    static Matrix cross (const Matrix &a, const Matrix &b);    // cross product of two vectors
    static Matrix inv (const Matrix &M);                       // invert matrix M
    bool   inv ();                                             // invert this matrix
    FLOAT  det ();                                             // returns determinant of matrix
    bool   solve (const Matrix &M,FLOAT eps=1e-20);            // solve linear system M*x=B, replaces *this and M
    bool   lu(int32_t *idx, FLOAT &d, FLOAT eps=1e-20);        // replace *this by lower upper decomposition
    void   svd(Matrix &U,Matrix &W,Matrix &V);                 // singular value decomposition *this = U*diag(W)*V^T

    // print matrix to stream
    friend std::ostream& operator<< (std::ostream& out,const Matrix& M);

    // direct data access
    FLOAT   **val;
    int32_t   m,n;

private:

    void allocateMemory (const int32_t m_,const int32_t n_);
    void releaseMemory ();
    inline FLOAT pythag(FLOAT a,FLOAT b);

};

#endif // MATRIX_H





//-----------------------------------------------------------------------------------------------------------------------------------------------
//timer
/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Andreas Geiger

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

#ifndef __TIMER_H__
#define __TIMER_H__

#include <iomanip>
#include <string>
#include <sys/time.h>

// Define fixed-width datatypes for Visual Studio projects
#ifndef _MSC_VER
#include <stdint.h>
#else
typedef __int8            int8_t;
  typedef __int16           int16_t;
  typedef __int32           int32_t;
  typedef __int64           int64_t;
  typedef unsigned __int8   uint8_t;
  typedef unsigned __int16  uint16_t;
  typedef unsigned __int32  uint32_t;
  typedef unsigned __int64  uint64_t;
#endif

class Timer {

public:

    Timer() {}

    ~Timer() {}

    void start (std::string title) {
      desc.push_back(title);
      push_back_time();
    }

    void stop () {
      if (time.size()<=desc.size())
        push_back_time();
    }

    void plot () {
      stop();
      float total_time = 0;
      for (int32_t i=0; i<desc.size(); i++) {
        float curr_time = getTimeDifferenceMilliseconds(time[i],time[i+1]);
        total_time += curr_time;
        std::cout.width(30);
        std::cout << desc[i] << " ";
        std::cout << std::fixed << std::setprecision(1) << std::setw(6);
        std::cout << curr_time;
        std::cout << " ms" << std::endl;
      }
      std::cout << "========================================" << std::endl;
      std::cout << "                    Total time ";
      std::cout << std::fixed << std::setprecision(1) << std::setw(6);
      std::cout << total_time;
      std::cout << " ms" << std::endl << std::endl;
    }

    void reset () {
      desc.clear();
      time.clear();
    }

private:

    std::vector<std::string>  desc;
    std::vector<timeval>      time;

    void push_back_time () {
      timeval curr_time;
      gettimeofday(&curr_time,0);
      time.push_back(curr_time);
    }

    float getTimeDifferenceMilliseconds(timeval a,timeval b) {
      return ((float)(b.tv_sec -a.tv_sec ))*1e+3 +
             ((float)(b.tv_usec-a.tv_usec))*1e-3;
    }
};

#endif



//-----------------------------------------------------------------------------------------------------------------------------------------------
//triangle
struct triangulateio {
    float *pointlist;                                               /* In / out */
    float *pointattributelist;                                      /* In / out */
    int *pointmarkerlist;                                          /* In / out */
    int numberofpoints;                                            /* In / out */
    int numberofpointattributes;                                   /* In / out */

    int *trianglelist;                                             /* In / out */
    float *triangleattributelist;                                   /* In / out */
    float *trianglearealist;                                         /* In only */
    int *neighborlist;                                             /* Out only */
    int numberoftriangles;                                         /* In / out */
    int numberofcorners;                                           /* In / out */
    int numberoftriangleattributes;                                /* In / out */

    int *segmentlist;                                              /* In / out */
    int *segmentmarkerlist;                                        /* In / out */
    int numberofsegments;                                          /* In / out */

    float *holelist;                        /* In / pointer to array copied out */
    int numberofholes;                                      /* In / copied out */

    float *regionlist;                      /* In / pointer to array copied out */
    int numberofregions;                                    /* In / copied out */

    int *edgelist;                                                 /* Out only */
    int *edgemarkerlist;            /* Not used with Voronoi diagram; out only */
    float *normlist;                /* Used only with Voronoi diagram; out only */
    int numberofedges;                                             /* Out only */
};

void triangulate(char *,triangulateio *,triangulateio *,triangulateio *);
void trifree(int *memptr);

/*****************************************************************************/
/*                                                                           */

/*                                                                           */
/*  Include file for programs that call Triangle.                            */
/*                                                                           */
/*  Accompanies Triangle Version 1.6                                         */
/*  July 28, 2005                                                            */
/*                                                                           */
/*  Copyright 1996, 2005                                                     */
/*  Jonathan Richard Shewchuk                                                */
/*  2360 Woolsey #H                                                          */
/*  Berkeley, California  94705-1927                                         */
/*  jrs@cs.berkeley.edu                                                      */
/*                                                                           */
/*  Modified by Andreas Geiger, 2011                                         */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  How to call Triangle from another program                                */
/*                                                                           */
/*                                                                           */
/*  If you haven't read Triangle's instructions (run "triangle -h" to read   */
/*  them), you won't understand what follows.                                */
/*                                                                           */
/*  Triangle must be compiled into an object file (triangle.o) with the      */
/*  TRILIBRARY symbol defined (generally by using the -DTRILIBRARY compiler  */
/*  switch).  The makefile included with Triangle will do this for you if    */
/*  you run "make trilibrary".  The resulting object file can be called via  */
/*  the procedure triangulate().                                             */
/*                                                                           */
/*  If the size of the object file is important to you, you may wish to      */
/*  generate a reduced version of triangle.o.  The REDUCED symbol gets rid   */
/*  of all features that are primarily of research interest.  Specifically,  */
/*  the -DREDUCED switch eliminates Triangle's -i, -F, -s, and -C switches.  */
/*  The CDT_ONLY symbol gets rid of all meshing algorithms above and beyond  */
/*  constrained Delaunay triangulation.  Specifically, the -DCDT_ONLY switch */
/*  eliminates Triangle's -r, -q, -a, -u, -D, -Y, -S, and -s switches.       */
/*                                                                           */
/*  IMPORTANT:  These definitions (TRILIBRARY, REDUCED, CDT_ONLY) must be    */
/*  made in the makefile or in triangle.c itself.  Putting these definitions */

/*                                                                           */
/*                                                                           */
/*  The calling convention for triangulate() follows.                        */
/*                                                                           */
/*      void triangulate(triswitches, in, out, vorout)                       */
/*      char *triswitches;                                                   */
///*      struct triangulateio *in;                                            */
///*      struct triangulateio *out;                                           */
///*      struct triangulateio *vorout;                                        */
/*                                                                           */
/*  `triswitches' is a string containing the command line switches you wish  */
/*  to invoke.  No initial dash is required.  Some suggestions:              */
/*                                                                           */
/*  - You'll probably find it convenient to use the `z' switch so that       */
/*    points (and other items) are numbered from zero.  This simplifies      */
/*    indexing, because the first item of any type always starts at index    */
/*    [0] of the corresponding array, whether that item's number is zero or  */
/*    one.                                                                   */
/*  - You'll probably want to use the `Q' (quiet) switch in your final code, */
/*    but you can take advantage of Triangle's printed output (including the */
/*    `V' switch) while debugging.                                           */
/*  - If you are not using the `q', `a', `u', `D', `j', or `s' switches,     */
/*    then the output points will be identical to the input points, except   */
/*    possibly for the boundary markers.  If you don't need the boundary     */
/*    markers, you should use the `N' (no nodes output) switch to save       */
/*    memory.  (If you do need boundary markers, but need to save memory, a  */
/*    good nasty trick is to set out->pointlist equal to in->pointlist       */
/*    before calling triangulate(), so that Triangle overwrites the input    */
/*    points with identical copies.)                                         */
/*  - The `I' (no iteration numbers) and `g' (.off file output) switches     */
/*    have no effect when Triangle is compiled with TRILIBRARY defined.      */
/*                                                                           */
/*  `in', `out', and `vorout' are descriptions of the input, the output,     */
/*  and the Voronoi output.  If the `v' (Voronoi output) switch is not used, */
/*  `vorout' may be NULL.  `in' and `out' may never be NULL.                 */
/*                                                                           */
/*  Certain fields of the input and output structures must be initialized,   */
/*  as described below.                                                      */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  The `triangulateio' structure.                                           */
/*                                                                           */
/*  Used to pass data into and out of the triangulate() procedure.           */
/*                                                                           */
/*                                                                           */
/*  Arrays are used to store points, triangles, markers, and so forth.  In   */
/*  all cases, the first item in any array is stored starting at index [0].  */
/*  However, that item is item number `1' unless the `z' switch is used, in  */
/*  which case it is item number `0'.  Hence, you may find it easier to      */
/*  index points (and triangles in the neighbor list) if you use the `z'     */
/*  switch.  Unless, of course, you're calling Triangle from a Fortran       */
/*  program.                                                                 */
/*                                                                           */
/*  Description of fields (except the `numberof' fields, which are obvious): */
/*                                                                           */
/*  `pointlist':  An array of point coordinates.  The first point's x        */
/*    coordinate is at index [0] and its y coordinate at index [1], followed */
/*    by the coordinates of the remaining points.  Each point occupies two   */
/*    REALs.                                                                 */
/*  `pointattributelist':  An array of point attributes.  Each point's       */
/*    attributes occupy `numberofpointattributes' REALs.                     */
/*  `pointmarkerlist':  An array of point markers; one int per point.        */
/*                                                                           */
/*  `trianglelist':  An array of triangle corners.  The first triangle's     */
/*    first corner is at index [0], followed by its other two corners in     */
/*    counterclockwise order, followed by any other nodes if the triangle    */
/*    represents a nonlinear element.  Each triangle occupies                */
/*    `numberofcorners' ints.                                                */
/*  `triangleattributelist':  An array of triangle attributes.  Each         */
/*    triangle's attributes occupy `numberoftriangleattributes' REALs.       */
/*  `trianglearealist':  An array of triangle area constraints; one REAL per */
/*    triangle.  Input only.                                                 */
/*  `neighborlist':  An array of triangle neighbors; three ints per          */
/*    triangle.  Output only.                                                */
/*                                                                           */
/*  `segmentlist':  An array of segment endpoints.  The first segment's      */
/*    endpoints are at indices [0] and [1], followed by the remaining        */
/*    segments.  Two ints per segment.                                       */
/*  `segmentmarkerlist':  An array of segment markers; one int per segment.  */
/*                                                                           */
/*  `holelist':  An array of holes.  The first hole's x and y coordinates    */
/*    are at indices [0] and [1], followed by the remaining holes.  Two      */
/*    REALs per hole.  Input only, although the pointer is copied to the     */
/*    output structure for your convenience.                                 */
/*                                                                           */
/*  `regionlist':  An array of regional attributes and area constraints.     */
/*    The first constraint's x and y coordinates are at indices [0] and [1], */
/*    followed by the regional attribute at index [2], followed by the       */
/*    maximum area at index [3], followed by the remaining area constraints. */
/*    Four REALs per area constraint.  Note that each regional attribute is  */
/*    used only if you select the `A' switch, and each area constraint is    */
/*    used only if you select the `a' switch (with no number following), but */
/*    omitting one of these switches does not change the memory layout.      */
/*    Input only, although the pointer is copied to the output structure for */
/*    your convenience.                                                      */
/*                                                                           */
/*  `edgelist':  An array of edge endpoints.  The first edge's endpoints are */
/*    at indices [0] and [1], followed by the remaining edges.  Two ints per */
/*    edge.  Output only.                                                    */
/*  `edgemarkerlist':  An array of edge markers; one int per edge.  Output   */
/*    only.                                                                  */
/*  `normlist':  An array of normal vectors, used for infinite rays in       */
/*    Voronoi diagrams.  The first normal vector's x and y magnitudes are    */
/*    at indices [0] and [1], followed by the remaining vectors.  For each   */
/*    finite edge in a Voronoi diagram, the normal vector written is the     */
/*    zero vector.  Two REALs per edge.  Output only.                        */
/*                                                                           */
/*                                                                           */
/*  Any input fields that Triangle will examine must be initialized.         */
/*  Furthermore, for each output array that Triangle will write to, you      */
/*  must either provide space by setting the appropriate pointer to point    */
/*  to the space you want the data written to, or you must initialize the    */
/*  pointer to NULL, which tells Triangle to allocate space for the results. */
/*  The latter option is preferable, because Triangle always knows exactly   */
/*  how much space to allocate.  The former option is provided mainly for    */
/*  people who need to call Triangle from Fortran code, though it also makes */
/*  possible some nasty space-saving tricks, like writing the output to the  */
/*  same arrays as the input.                                                */
/*                                                                           */
/*  Triangle will not free() any input or output arrays, including those it  */
/*  allocates itself; that's up to you.  You should free arrays allocated by */
/*  Triangle by calling the trifree() procedure defined below.  (By default, */
/*  trifree() just calls the standard free() library procedure, but          */
/*  applications that call triangulate() may replace trimalloc() and         */
/*  trifree() in triangle.c to use specialized memory allocators.)           */
/*                                                                           */
/*  Here's a guide to help you decide which fields you must initialize       */
/*  before you call triangulate().                                           */
/*                                                                           */
/*  `in':                                                                    */
/*                                                                           */
/*    - `pointlist' must always point to a list of points; `numberofpoints'  */
/*      and `numberofpointattributes' must be properly set.                  */
/*      `pointmarkerlist' must either be set to NULL (in which case all      */
/*      markers default to zero), or must point to a list of markers.  If    */
/*      `numberofpointattributes' is not zero, `pointattributelist' must     */
/*      point to a list of point attributes.                                 */
/*    - If the `r' switch is used, `trianglelist' must point to a list of    */
/*      triangles, and `numberoftriangles', `numberofcorners', and           */
/*      `numberoftriangleattributes' must be properly set.  If               */
/*      `numberoftriangleattributes' is not zero, `triangleattributelist'    */
/*      must point to a list of triangle attributes.  If the `a' switch is   */
/*      used (with no number following), `trianglearealist' must point to a  */
/*      list of triangle area constraints.  `neighborlist' may be ignored.   */
/*    - If the `p' switch is used, `segmentlist' must point to a list of     */
/*      segments, `numberofsegments' must be properly set, and               */
/*      `segmentmarkerlist' must either be set to NULL (in which case all    */
/*      markers default to zero), or must point to a list of markers.        */
/*    - If the `p' switch is used without the `r' switch, then               */
/*      `numberofholes' and `numberofregions' must be properly set.  If      */
/*      `numberofholes' is not zero, `holelist' must point to a list of      */
/*      holes.  If `numberofregions' is not zero, `regionlist' must point to */
/*      a list of region constraints.                                        */
/*    - If the `p' switch is used, `holelist', `numberofholes',              */
/*      `regionlist', and `numberofregions' is copied to `out'.  (You can    */
/*      nonetheless get away with not initializing them if the `r' switch is */
/*      used.)                                                               */
/*    - `edgelist', `edgemarkerlist', `normlist', and `numberofedges' may be */
/*      ignored.                                                             */
/*                                                                           */
/*  `out':                                                                   */
/*                                                                           */
/*    - `pointlist' must be initialized (NULL or pointing to memory) unless  */
/*      the `N' switch is used.  `pointmarkerlist' must be initialized       */
/*      unless the `N' or `B' switch is used.  If `N' is not used and        */
/*      `in->numberofpointattributes' is not zero, `pointattributelist' must */
/*      be initialized.                                                      */
/*    - `trianglelist' must be initialized unless the `E' switch is used.    */
/*      `neighborlist' must be initialized if the `n' switch is used.  If    */
/*      the `E' switch is not used and (`in->numberofelementattributes' is   */
/*      not zero or the `A' switch is used), `elementattributelist' must be  */
/*      initialized.  `trianglearealist' may be ignored.                     */
/*    - `segmentlist' must be initialized if the `p' or `c' switch is used,  */
/*      and the `P' switch is not used.  `segmentmarkerlist' must also be    */
/*      initialized under these circumstances unless the `B' switch is used. */
/*    - `edgelist' must be initialized if the `e' switch is used.            */
/*      `edgemarkerlist' must be initialized if the `e' switch is used and   */
/*      the `B' switch is not.                                               */
/*    - `holelist', `regionlist', `normlist', and all scalars may be ignored.*/
/*                                                                           */
/*  `vorout' (only needed if `v' switch is used):                            */
/*                                                                           */
/*    - `pointlist' must be initialized.  If `in->numberofpointattributes'   */
/*      is not zero, `pointattributelist' must be initialized.               */
/*      `pointmarkerlist' may be ignored.                                    */
/*    - `edgelist' and `normlist' must both be initialized.                  */
/*      `edgemarkerlist' may be ignored.                                     */
/*    - Everything else may be ignored.                                      */
/*                                                                           */
/*  After a call to triangulate(), the valid fields of `out' and `vorout'    */
/*  will depend, in an obvious way, on the choice of switches used.  Note    */
/*  that when the `p' switch is used, the pointers `holelist' and            */
/*  `regionlist' are copied from `in' to `out', but no new space is          */
/*  allocated; be careful that you don't free() the same array twice.  On    */
/*  the other hand, Triangle will never copy the `pointlist' pointer (or any */
/*  others); new space is allocated for `out->pointlist', or if the `N'      */
/*  switch is used, `out->pointlist' remains uninitialized.                  */
/*                                                                           */
/*  All of the meaningful `numberof' fields will be properly set; for        */
/*  instance, `numberofedges' will represent the number of edges in the      */
/*  triangulation whether or not the edges were written.  If segments are    */
/*  not used, `numberofsegments' will indicate the number of boundary edges. */
/*                                                                           */
/*****************************************************************************/




