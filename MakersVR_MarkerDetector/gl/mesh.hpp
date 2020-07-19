#ifndef DEF_MESH
#define DEF_MESH

#include <GLES2/gl2.h>

#include <vector>

/* Structs and Definitions */

// Type of packed data for vertex attributes
typedef enum VertexType
{
	NONE = 0,
	POS = 1,
	COL = 2,
	TEX = 4,
	NRM = 8,
} VertexType;
inline VertexType operator | (VertexType a, VertexType b)
{
	return static_cast<VertexType>(static_cast<int>(a) | static_cast<int>(b));
}
inline unsigned int PackedFloats (VertexType type)
{
	unsigned int val = 0;
	if ((type & POS) != 0) val += 3;
	if ((type & COL) != 0) val += 3;
	if ((type & TEX) != 0) val += 2;
	if ((type & NRM) != 0) val += 3;
	return val;
}

/*
 * Generic Mesh structure allowing for arbitrary vertex data packing
 */
class Mesh
{
	public:
	GLuint VBO_ID, EBO_ID;
	VertexType type;
	GLenum mode;
	std::vector<VertexType> packing;
	unsigned int FpV, vertexCount, elementCount;

	Mesh(const std::vector<VertexType> Packing, const std::vector<float> Vertices, const std::vector<unsigned int> Elements);
	~Mesh(void);
	void draw(void);
	void setMode(GLenum mode);
};


#endif
