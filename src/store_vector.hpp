#ifndef __STORE_VECTOR_HPP__
#define __STORE_VECTOR_HPP__ 

#include <opencv2/opencv.hpp>

/******
simple templace functions for storing/loading stereo vectors. supports the class version as well as the POD version and stream version of target template storage, e.g. if target template is a basic type, use POD, if target template is a class which has the storeKeyPoint and loadKeyPoint functions implemented use class
******/
template<typename T>
void storeKeyPoint(const T& kp, std::ostream& os)
{
  os << kp.angle << "\n";
  os << kp.class_id << "\n";
  os << kp.octave << "\n";
  os << kp.pt.x << " " << kp.pt.y << "\n";
  os << kp.response << "\n";
  os << kp.size << "\n";
};

template<typename T>
void loadKeyPoint(T& kp, std::istream& is)
{
  is >> kp.angle;
  is.ignore(10, '\n');

  is >> kp.class_id;
  is.ignore(10, '\n');

  is >> kp.octave;
  is.ignore(10, '\n');

  float d1, d2;
  is >> d1;
  is >> d2;
  is.ignore(10, '\n');
  kp.pt = cv::Point2f(d1, d2);

  is >> kp.response;
  is.ignore(10, '\n');

  is >> kp.size;
  is.ignore(10, '\n');
};


template<typename T>
void StoreClassVector(const std::vector<T>& Tvector, std::ostream& os)
{ 
  typename std::vector<T>::size_type size(Tvector.size());
  os.write(reinterpret_cast<const char*>(&size), sizeof(typename std::vector<T>::size_type));
  for(typename std::vector<T>::size_type i =0; i < size; ++i)
  {
    storeKeyPoint(Tvector[i], os);
  }
};
  
template<typename T>
void LoadClassVector(std::vector<T>& Tvector, std::istream& is, int max_load_indices = -1)
{ 
  typename std::vector<T>::size_type to_load_size(0);
  is.read(reinterpret_cast<char*>(&to_load_size), sizeof(typename std::vector<T>::size_type));
  // check, if the whole vector should be loaded or only the indices until max_load_indices is reached
  if(max_load_indices > -1)
    to_load_size = ((int)to_load_size < max_load_indices)?to_load_size:max_load_indices;
  Tvector.resize(to_load_size);
  for(typename std::vector<T>::size_type i =0; i < to_load_size; ++i)
  {
    loadKeyPoint(Tvector[i], is);
  }
};
  
template<typename T>
void StorePODVector(const std::vector<T>& Tvector, std::ostream& os)
{ 
  typename std::vector<T>::size_type size(Tvector.size());
  os.write(reinterpret_cast<const char*>(&size), sizeof(typename std::vector<T>::size_type));
  for(typename std::vector<T>::size_type i = 0; i < size; i++)
    os << Tvector[i] << " ";
};
  
template<typename T>
void LoadPODVector(std::vector<T>& Tvector, std::istream& is)
{
  typename std::vector<T>::size_type to_load_size(0);
  is.read(reinterpret_cast<char*>(&to_load_size), sizeof(typename std::vector<T>::size_type));
  T temp;
  for(typename std::vector<T>::size_type i = 0; i < to_load_size; i++)
  {
    is >> temp;
    Tvector.push_back(temp);
  }
};  

template<typename T>
void StoreEigenMatrix3d(const T &matrix, std::ostream& os)
{
  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      os << matrix(i, j) << " ";
    }
  }
  os << "\n";
};

template<typename T>
void LoadEigenMatrix3d(T &matrix, std::istream& is)
{
  double a;
  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      is >> a;
      matrix(i, j) = a;
    }
  }
  is.ignore(10, '\n');
};

template<typename T>
void StoreEigenMatrix4d(const T &matrix, std::ostream& os)
{
  for(int i = 0; i < 4; ++i)
  {
    for(int j = 0; j < 4; ++j)
    {
      os << matrix(i, j) << " ";
    }
  }
  os << "\n";
};

template<typename T>
void LoadEigenMatrix4d(T &matrix, std::istream& is)
{
  double a;
  for(int i = 0; i < 4; ++i)
  {
    for(int j = 0; j < 4; ++j)
    {
      is >> a;
      matrix(i, j) = a;
    }
  }
  is.ignore(10, '\n');
};

#endif
