# STL File Reader and Writer
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
template<typename T, typename IterT>
inline bool read(basic_solid<T>& out, IterT begin, IterT end)
template<typename T, typename IterT>
inline std::pair< basic_solid<T>, bool > read(IterT begin, IterT end)
template<typename IterT>
inline std::pair< solid, bool > read(IterT begin, IterT end)

//////////////////////////////////////////////////////////////
// Writer Functions: (streamT is an output stream type)
template<typename T, typename streamT>
void write(streamT& out, basic_solid<T>& s, bool binary)
template<typename T, typename streamT>
void write_binary(streamT& out, basic_solid<T>&s)
template<typename T, typename streamT>
void write_ascii(streamT& out, basic_solid<T>&s)

// Remark: the solid parameter is not constant to ensure the correct size of the containers.
// You may want to rewrite those functions corresponding on your error-checking presumptions

// The write functions were just written as a proof of concept and you may want to avoid them

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
typedef basic_solid<float> dsolid;

template<typename T>
struct basic_vec3
{
    using scalar_type = T;
    T data[3];
};

typedef basic_vec3<float> vec3;
typedef basic_vec3<float> dvec3;
```


## License

[MIT License](./LICENSE) © Matthias Möller. Made with ♥ in Germany.
