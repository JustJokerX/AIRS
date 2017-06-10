#include "LoadMeshFromObj.h"
#include "tiny_obj_loader.h"
#include "GLInstanceGraphicsShape.h"
#include <stdio.h> //fopen
#include <bullet/Bullet3Common/b3AlignedObjectArray.h>
#include <string>
#include <vector>
#include "Wavefront2GLInstanceGraphicsShape.h"

GLInstanceGraphicsShape* LoadMeshFromObj(const char* relativeFileName, const char* materialPrefixPath)
{
	std::vector<tinyobj::shape_t> shapes;
	std::string err = tinyobj::LoadObj(shapes, relativeFileName, materialPrefixPath);
		
	GLInstanceGraphicsShape* gfxShape = btgCreateGraphicsShapeFromWavefrontObj(shapes);
	return gfxShape;
}
