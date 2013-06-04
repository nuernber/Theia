// Copyright (C) 2013 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#include <agast/agast5_8.h>
#include <agast/cvWrapper.h>
#include <agast/oast9_16.h>

#include "image/keypoint_detector/brisk_helper.h"

namespace theia {
// construct a layer
BriskLayer::BriskLayer(const GrayImage& img, float scale, float offset) {
  img_ = img;
  scores_ = GrayImage(img.Rows(), img.Cols());
  scale_ = scale;
  offset_ = offset;
  oastDetector_ = new agast::OastDetector9_16(img.cols, img.rows, 0);
  agastDetector_5_8_ = new agast::AgastDetector5_8(img.cols, img.rows, 0);
}
// derive a layer
BriskLayer::BriskLayer(const BriskLayer& layer, int mode){
  if(mode==CommonParams::HALFSAMPLE){
    img_.create(layer.img().rows/2, layer.img().cols/2,CV_8U);
    halfsample(layer.img(), img_);
    scale_= layer.scale()*2;
    offset_=0.5*scale_-0.5;
  }
  else {
    img_.create(2*(layer.img().rows/3), 2*(layer.img().cols/3),CV_8U);
    twothirdsample(layer.img(), img_);
    scale_= layer.scale()*1.5;
    offset_=0.5*scale_-0.5;
  }
  scores_=GrayImage::zeros(img_.rows,img_.cols,CV_8U);
  oastDetector_ = new agast::OastDetector9_16(img_.cols, img_.rows, 0);
  agastDetector_5_8_ = new agast::AgastDetector5_8(img_.cols, img_.rows, 0);
}

// Fast/Agast
// wraps the agast class
void BriskLayer::getAgastPoints(uint8_t threshold, std::vector<CvPoint>& keypoints){
  oastDetector_->set_threshold(threshold);
  oastDetector_->detect(img_.data,keypoints);

  // also write scores
  const int num=keypoints.size();
  const int imcols=img_.cols;

  for(int i=0; i<num; i++){
    const int offs=keypoints[i].x+keypoints[i].y*imcols;
    *(scores_.data+offs)=oastDetector_->cornerScore(img_.data+offs);
  }
}
inline uint8_t BriskLayer::getAgastScore(int x, int y, uint8_t threshold){
  if(x<3||y<3) return 0;
  if(x>=img_.cols-3||y>=img_.rows-3) return 0;
  uint8_t& score=*(scores_.data+x+y*scores_.cols);
  if(score>2) { return score; }
  oastDetector_->set_threshold(threshold-1);
  score = oastDetector_->cornerScore(img_.data+x+y*img_.cols);
  if (score<threshold) score = 0;
  return score;
}

inline uint8_t BriskLayer::getAgastScore_5_8(int x, int y, uint8_t threshold){
  if(x<2||y<2) return 0;
  if(x>=img_.cols-2||y>=img_.rows-2) return 0;
  agastDetector_5_8_->set_threshold(threshold-1);
  uint8_t score = agastDetector_5_8_->cornerScore(img_.data+x+y*img_.cols);
  if (score<threshold) score = 0;
  return score;
}

inline uint8_t BriskLayer::getAgastScore(float xf, float yf, uint8_t threshold, float scale){
  if(scale<=1.0f){
    // just do an interpolation inside the layer
    const int x=int(xf);
    const float rx1=xf-float(x);
    const float rx=1.0f-rx1;
    const int y=int(yf);
    const float ry1=yf-float(y);
    const float ry=1.0f-ry1;

    return rx*ry*getAgastScore(x, y, threshold)+
        rx1*ry*getAgastScore(x+1, y, threshold)+
        rx*ry1*getAgastScore(x, y+1, threshold)+
        rx1*ry1*getAgastScore(x+1, y+1, threshold);
  }
  else{
    // this means we overlap area smoothing
    const float halfscale = scale/2.0f;
    // get the scores first:
    for(int x=int(xf-halfscale); x<=int(xf+halfscale+1.0f); x++){
      for(int y=int(yf-halfscale); y<=int(yf+halfscale+1.0f); y++){
        getAgastScore(x, y, threshold);
      }
    }
    // get the smoothed value
    return value(scores_,xf,yf,scale);
  }
}

// access gray values (smoothed/interpolated)
__inline__ uint8_t BriskLayer::value(const GrayImage& mat, float xf, float yf, float scale){
  assert(!mat.empty());
  // get the position
  const int x = floor(xf);
  const int y = floor(yf);
  const GrayImage& image=mat;
  const int& imagecols=image.cols;

  // get the sigma_half:
  const float sigma_half=scale/2;
  const float area=4.0*sigma_half*sigma_half;
  // calculate output:
  int ret_val;
  if(sigma_half<0.5){
    //interpolation multipliers:
    const int r_x=(xf-x)*1024;
    const int r_y=(yf-y)*1024;
    const int r_x_1=(1024-r_x);
    const int r_y_1=(1024-r_y);
    uchar* ptr=image.data+x+y*imagecols;
    // just interpolate:
    ret_val=(r_x_1*r_y_1*int(*ptr));
    ptr++;
    ret_val+=(r_x*r_y_1*int(*ptr));
    ptr+=imagecols;
    ret_val+=(r_x*r_y*int(*ptr));
    ptr--;
    ret_val+=(r_x_1*r_y*int(*ptr));
    return 0xFF&((ret_val+512)/1024/1024);
  }

  // this is the standard case (simple, not speed optimized yet):

  // scaling:
  const int scaling = 4194304.0/area;
  const int scaling2=float(scaling)*area/1024.0;

  // calculate borders
  const float x_1=xf-sigma_half;
  const float x1=xf+sigma_half;
  const float y_1=yf-sigma_half;
  const float y1=yf+sigma_half;

  const int x_left=int(x_1+0.5);
  const int y_top=int(y_1+0.5);
  const int x_right=int(x1+0.5);
  const int y_bottom=int(y1+0.5);

  // overlap area - multiplication factors:
  const float r_x_1=float(x_left)-x_1+0.5;
  const float r_y_1=float(y_top)-y_1+0.5;
  const float r_x1=x1-float(x_right)+0.5;
  const float r_y1=y1-float(y_bottom)+0.5;
  const int dx=x_right-x_left-1;
  const int dy=y_bottom-y_top-1;
  const int A=(r_x_1*r_y_1)*scaling;
  const int B=(r_x1*r_y_1)*scaling;
  const int C=(r_x1*r_y1)*scaling;
  const int D=(r_x_1*r_y1)*scaling;
  const int r_x_1_i=r_x_1*scaling;
  const int r_y_1_i=r_y_1*scaling;
  const int r_x1_i=r_x1*scaling;
  const int r_y1_i=r_y1*scaling;

  // now the calculation:
  uchar* ptr=image.data+x_left+imagecols*y_top;
  // first row:
  ret_val=A*int(*ptr);
  ptr++;
  const uchar* end1 = ptr+dx;
  for(; ptr<end1; ptr++){
    ret_val+=r_y_1_i*int(*ptr);
  }
  ret_val+=B*int(*ptr);
  // middle ones:
  ptr+=imagecols-dx-1;
  uchar* end_j=ptr+dy*imagecols;
  for(; ptr<end_j; ptr+=imagecols-dx-1){
    ret_val+=r_x_1_i*int(*ptr);
    ptr++;
    const uchar* end2 = ptr+dx;
    for(; ptr<end2; ptr++){
      ret_val+=int(*ptr)*scaling;
    }
    ret_val+=r_x1_i*int(*ptr);
  }
  // last row:
  ret_val+=D*int(*ptr);
  ptr++;
  const uchar* end3 = ptr+dx;
  for(; ptr<end3; ptr++){
    ret_val+=r_y1_i*int(*ptr);
  }
  ret_val+=C*int(*ptr);

  return 0xFF&((ret_val+scaling2/2)/scaling2/1024);
}

// half sampling
inline void BriskLayer::halfsample(const GrayImage& srcimg, GrayImage& dstimg){
  const unsigned short leftoverCols = ((srcimg.cols%16)/2);// take care with border...
  const bool noleftover = (srcimg.cols%16)==0; // note: leftoverCols can be zero but this still false...

  // make sure the destination image is of the right size:
  assert(srcimg.cols/2==dstimg.cols);
  assert(srcimg.rows/2==dstimg.rows);

  // mask needed later:
  register __m128i mask = _mm_set_epi32 (0x00FF00FF, 0x00FF00FF, 0x00FF00FF, 0x00FF00FF);
  // to be added in order to make successive averaging correct:
  register __m128i ones = _mm_set_epi32 (0x11111111, 0x11111111, 0x11111111, 0x11111111);

  // data pointers:
  __m128i* p1=(__m128i*)srcimg.data;
  __m128i* p2=(__m128i*)(srcimg.data+srcimg.cols);
  __m128i* p_dest=(__m128i*)dstimg.data;
  unsigned char* p_dest_char;//=(unsigned char*)p_dest;

  // size:
  const unsigned int size = (srcimg.cols*srcimg.rows)/16;
  const unsigned int hsize = srcimg.cols/16;
  __m128i* p_end=p1+size;
  unsigned int row=0;
  const unsigned int end=hsize/2;
  bool half_end;
  if(hsize%2==0)
    half_end=false;
  else
    half_end=true;
  while(p2<p_end){
    for(unsigned int i=0; i<end;i++){
      // load the two blocks of memory:
      __m128i upper;
      __m128i lower;
      if(noleftover){
        upper=_mm_load_si128(p1);
        lower=_mm_load_si128(p2);
      }
      else{
        upper=_mm_loadu_si128(p1);
        lower=_mm_loadu_si128(p2);
      }

      __m128i result1=_mm_adds_epu8 (upper, ones);
      result1=_mm_avg_epu8 (upper, lower);

      // increment the pointers:
      p1++;
      p2++;

      // load the two blocks of memory:
      upper=_mm_loadu_si128(p1);
      lower=_mm_loadu_si128(p2);
      __m128i result2=_mm_adds_epu8 (upper, ones);
      result2=_mm_avg_epu8 (upper, lower);
      // calculate the shifted versions:
      __m128i result1_shifted = _mm_srli_si128 (result1, 1);
      __m128i result2_shifted = _mm_srli_si128 (result2, 1);
      // pack:
      __m128i result=_mm_packus_epi16 (_mm_and_si128 (result1, mask),
                                       _mm_and_si128 (result2, mask));
      __m128i result_shifted = _mm_packus_epi16 (_mm_and_si128 (result1_shifted, mask),
                                                 _mm_and_si128 (result2_shifted, mask));
      // average for the second time:
      result=_mm_avg_epu8(result,result_shifted);

      // store to memory
      _mm_storeu_si128 (p_dest, result);

      // increment the pointers:
      p1++;
      p2++;
      p_dest++;
      //p_dest_char=(unsigned char*)p_dest;
    }
    // if we are not at the end of the row, do the rest:
    if(half_end){
      // load the two blocks of memory:
      __m128i upper;
      __m128i lower;
      if(noleftover){
        upper=_mm_load_si128(p1);
        lower=_mm_load_si128(p2);
      }
      else{
        upper=_mm_loadu_si128(p1);
        lower=_mm_loadu_si128(p2);
      }

      __m128i result1=_mm_adds_epu8 (upper, ones);
      result1=_mm_avg_epu8 (upper, lower);

      // increment the pointers:
      p1++;
      p2++;

      // compute horizontal pairwise average and store
      p_dest_char=(unsigned char*)p_dest;
      const UCHAR_ALIAS* result=(UCHAR_ALIAS*)&result1;
      for(unsigned int j=0; j<8; j++){
        *(p_dest_char++)=(*(result+2*j)+*(result+2*j+1))/2;
      }
      //p_dest_char=(unsigned char*)p_dest;
    }
    else{
      p_dest_char=(unsigned char*)p_dest;
    }

    if(noleftover){
      row++;
      p_dest=(__m128i*)(dstimg.data+row*dstimg.cols);
      p1=(__m128i*)(srcimg.data+2*row*srcimg.cols);
      //p2=(__m128i*)(srcimg.data+(2*row+1)*srcimg.cols);
      //p1+=hsize;
      p2=p1+hsize;
    }
    else{
      const unsigned char* p1_src_char=(unsigned char*)(p1);
      const unsigned char* p2_src_char=(unsigned char*)(p2);
      for(unsigned int k=0; k<leftoverCols; k++){
        unsigned short tmp = p1_src_char[k]+p1_src_char[k+1]+
                             p2_src_char[k]+p2_src_char[k+1];
        *(p_dest_char++)=(unsigned char)(tmp/4);
      }
      // done with the two rows:
      row++;
      p_dest=(__m128i*)(dstimg.data+row*dstimg.cols);
      p1=(__m128i*)(srcimg.data+2*row*srcimg.cols);
      p2=(__m128i*)(srcimg.data+(2*row+1)*srcimg.cols);
    }
  }
}

inline void BriskLayer::twothirdsample(const GrayImage& srcimg, GrayImage& dstimg){
  const unsigned short leftoverCols = ((srcimg.cols/3)*3)%15;// take care with border...

  // make sure the destination image is of the right size:
  assert((srcimg.cols/3)*2==dstimg.cols);
  assert((srcimg.rows/3)*2==dstimg.rows);

  // masks:
  register __m128i mask1 = _mm_set_epi8 (0x80,0x80,0x80,0x80,0x80,0x80,0x80,12,0x80,10,0x80,7,0x80,4,0x80,1);
  register __m128i mask2 = _mm_set_epi8 (0x80,0x80,0x80,0x80,0x80,0x80,12,0x80,10,0x80,7,0x80,4,0x80,1,0x80);
  register __m128i mask = _mm_set_epi8 (0x80,0x80,0x80,0x80,0x80,0x80,14,12,11,9,8,6,5,3,2,0);
  register __m128i store_mask = _mm_set_epi8 (0,0,0,0,0,0,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80);

  // data pointers:
  unsigned char* p1=srcimg.data;
  unsigned char* p2=p1+srcimg.cols;
  unsigned char* p3=p2+srcimg.cols;
  unsigned char* p_dest1 = dstimg.data;
  unsigned char* p_dest2 = p_dest1+dstimg.cols;
  unsigned char* p_end=p1+(srcimg.cols*srcimg.rows);

  unsigned int row=0;
  unsigned int row_dest=0;
  int hsize = srcimg.cols/15;
  while(p3<p_end){
    for(int i=0; i<hsize; i++){
      // load three rows
      __m128i first = _mm_loadu_si128((__m128i*)p1);
      __m128i second = _mm_loadu_si128((__m128i*)p2);
      __m128i third = _mm_loadu_si128((__m128i*)p3);

      // upper row:
      __m128i upper = _mm_avg_epu8(_mm_avg_epu8(first,second),first);
      __m128i temp1_upper = _mm_or_si128(_mm_shuffle_epi8(upper,mask1),_mm_shuffle_epi8(upper,mask2));
      __m128i temp2_upper=_mm_shuffle_epi8(upper,mask);
      __m128i result_upper = _mm_avg_epu8(_mm_avg_epu8(temp2_upper,temp1_upper),temp2_upper);

      // lower row:
      __m128i lower = _mm_avg_epu8(_mm_avg_epu8(third,second),third);
      __m128i temp1_lower = _mm_or_si128(_mm_shuffle_epi8(lower,mask1),_mm_shuffle_epi8(lower,mask2));
      __m128i temp2_lower=_mm_shuffle_epi8(lower,mask);
      __m128i result_lower = _mm_avg_epu8(_mm_avg_epu8(temp2_lower,temp1_lower),temp2_lower);

      // store:
      if(i*10+16>dstimg.cols){
        _mm_maskmoveu_si128(result_upper, store_mask, (char*)p_dest1);
        _mm_maskmoveu_si128(result_lower, store_mask, (char*)p_dest2);
      }
      else{
        _mm_storeu_si128 ((__m128i*)p_dest1, result_upper);
        _mm_storeu_si128 ((__m128i*)p_dest2, result_lower);
      }

      // shift pointers:
      p1+=15;
      p2+=15;
      p3+=15;
      p_dest1+=10;
      p_dest2+=10;
    }

    // fill the remainder:
    for(unsigned int j = 0; j<leftoverCols;j+=3){
      const unsigned short A1=*(p1++);
      const unsigned short A2=*(p1++);
      const unsigned short A3=*(p1++);
      const unsigned short B1=*(p2++);
      const unsigned short B2=*(p2++);
      const unsigned short B3=*(p2++);
      const unsigned short C1=*(p3++);
      const unsigned short C2=*(p3++);
      const unsigned short C3=*(p3++);

      *(p_dest1++)=(unsigned char)(((4*A1+2*(A2+B1)+B2)/9)&0x00FF);
      *(p_dest1++)=(unsigned char)(((4*A3+2*(A2+B3)+B2)/9)&0x00FF);
      *(p_dest2++)=(unsigned char)(((4*C1+2*(C2+B1)+B2)/9)&0x00FF);
      *(p_dest2++)=(unsigned char)(((4*C3+2*(C2+B3)+B2)/9)&0x00FF);
    }


    // increment row counter:
    row+=3;
    row_dest+=2;

    // reset pointers
    p1=srcimg.data+row*srcimg.cols;
    p2=p1+srcimg.cols;
    p3=p2+srcimg.cols;
    p_dest1 = dstimg.data+row_dest*dstimg.cols;
    p_dest2 = p_dest1+dstimg.cols;
  }
}
}  // namespace
