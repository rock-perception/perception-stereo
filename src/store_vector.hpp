#ifndef __STORE_VECTOR_HPP__
#define __STORE_VECTOR_HPP__ 

/******
simple templace functions for storing/loading entire vectors. supports the class version as well as the POD version and stream version of target template storage, e.g. if target template is a basic type, use POD, if target template is a class which has the .store and .read functions implemented use class and if the target template is a class with the stream operators defined use stream.
******/

template<typename T>
void StoreClassVector(const std::vector<T>& Tvector, std::ostream& os)
{ 
  typename std::vector<T>::size_type size(Tvector.size());
  os.write(reinterpret_cast<const char*>(&size), sizeof(typename std::vector<T>::size_type));
  for(typename std::vector<T>::size_type i =0; i < size; ++i)
  {
    Tvector[i].store(os);
  }
};
  
template<typename T>
void LoadClassVector(std::vector<T>& Tvector, std::istream& is)
{ 
  typename std::vector<T>::size_type to_load_size(0);
  is.read(reinterpret_cast<char*>(&to_load_size), sizeof(typename std::vector<T>::size_type));
  Tvector.resize(to_load_size);
  for(typename std::vector<T>::size_type i =0; i < to_load_size; ++i)
  {
    Tvector[i].load(is);
  }
};
  
template<typename T>
void StoreStreamVector(const std::vector<T>& Tvector, std::ostream& os)
{ 
  typename std::vector<T>::size_type size(Tvector.size());
  os.write(reinterpret_cast<const char*>(&size), sizeof(typename std::vector<T>::size_type));
  for(typename std::vector<T>::size_type i =0; i < size; ++i)
  {
    os << Tvector[i];
  }
};
  
template<typename T>
void LoadStreamVector(std::vector<T>& Tvector, std::istream& is)
{ 
  typename std::vector<T>::size_type to_load_size(0);
  is.read(reinterpret_cast<char*>(&to_load_size), sizeof(typename std::vector<T>::size_type));
  Tvector.resize(to_load_size);
  for(typename std::vector<T>::size_type i =0; i < to_load_size; ++i)
  {
    is >> Tvector[i];
  }
};
  
template<typename T>
void StorePODVector(const std::vector<T>& Tvector, std::ostream& os)
{ 
  typename std::vector<T>::size_type size(Tvector.size());
  os.write(reinterpret_cast<const char*>(&size), sizeof(typename std::vector<T>::size_type));
  if(0 != size)
    os.write(reinterpret_cast<const char*>(&Tvector), size * sizeof(T));
};
  
template<typename T>
void LoadPODVector(std::vector<T>& Tvector, std::istream& is)
{
  typename std::vector<T>::size_type to_load_size(0);
  is.read(reinterpret_cast<char*>(&to_load_size), sizeof(typename std::vector<T>::size_type));
  if(0 != to_load_size)
  { 
    Tvector.resize(to_load_size);
    is.read(reinterpret_cast<char*>(&Tvector[0]), to_load_size * sizeof(T));
  } 
};  

#endif
