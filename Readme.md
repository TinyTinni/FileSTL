# STL File Reader and Writer
[![CMake](https://github.com/TinyTinni/FileSTL/actions/workflows/cmake-build.yml/badge.svg)](https://github.com/TinyTinni/FileSTL/actions/workflows/cmake-build.yml)

Reads and writes STL files for 3D Geometry.

## Install
Header only

## Dependencies:
  - [Boost.Spirit](http://www.boost.org/)
  - [Boost.Test (for tests)](http://www.boost.org/)

## Small and Fast Reference
Use one of the read functions in namespace `tyti::stl`.

Possible scalar values for the template `float, double`, default is `float`.

```c++
//////////////////////////////////////////////////////////////
// Reader Functions:

//for super simple usage
inline std::pair<solid, bool> read(const std::string& fileName);

template<typename T>
inline bool read(basic_solid<T>& out, const std::string& fileName);
template<typename T>
inline std::pair<basic_solid<T>, bool> read(const std::string& fileName);

template<typename T, typename IterT>
inline bool read(basic_solid<T>& out, IterT begin, IterT end);
template<typename T, typename IterT>
inline std::pair< basic_solid<T>, bool > read(IterT begin, IterT end);
template<typename IterT>
inline std::pair< solid, bool > read(IterT begin, IterT end);


//////////////////////////////////////////////////////////////
// Writer Functions: (streamT is an output stream type)
template<typename T, typename streamT>
void write(streamT& out, const basic_solid<T>& s, bool binary);
template<typename T, typename streamT>
void write_binary(streamT& out, const basic_solid<T>& s);
template<typename T, typename streamT>
void write_ascii(streamT& out, const basic_solid<T>&s);

// Remark: #vertices should be 3*#normals, otherwise some data gets ignored
// You may want to rewrite those functions corresponding on your error-checking presumptions

//////////////////////////////////////////////////////////////
// Output DataType:
template<typename T>
struct basic_solid
{
    using vec3 = basic_vec3<T>;
    using scalar_type = T;
    std::string header; //solid name or header
    std::vector<vec3> normals;
    std::vector<vec3> vertices;
    std::vector<std::uint16_t> attributes; //only available in binary files
};

typedef basic_solid<float> solid;
typedef basic_solid<double> dsolid;

template<typename T>
struct basic_vec3
{
    using scalar_type = T;
    T data[3];
};

typedef basic_vec3<float> vec3;
typedef basic_vec3<double> dvec3;
```


## License

[MIT License](./LICENSE) © Matthias Möller. Made with ♥ in Germany.
