/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "mesh.hpp"

#include "GL/glew.h"

#include <math.h>
#include <iostream>

static const GLint vPosAdr = 0, vColAdr = 1, vUVAdr = 2, vNrmAdr = 3;

Mesh::Mesh(const std::vector<VertexType> Packing, const std::vector<float> Vertices, const std::vector<unsigned int> Elements)
{
	// Calculate Floats per vertex as specified by packing
	packing = Packing;
	type = NONE;
	for (int i = 0; i < packing.size(); i++)
		type = type | packing[i];
	FpV = PackedFloats(type);
	vertexCount = (unsigned int)std::floor((double)Vertices.size()/FpV);
	elementCount = (unsigned int)Elements.size();
//	std::cout << "Mesh (type " << type << ", FpV " << FpV << "): " << vertexCount << " vertices (" << Vertices.size() << " floats) / " << elementCount << " elements\n";

	// Setup vertex buffer
	glGenBuffers(1, &VBO_ID);
	glBindBuffer(GL_ARRAY_BUFFER, VBO_ID);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * Vertices.size(), &Vertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// Setup elements buffer
	EBO_ID = 0;
	if (elementCount > 0)
	{
		glGenBuffers(1, &EBO_ID);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_ID);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * elementCount, &Elements[0], GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	mode = GL_TRIANGLES;
}

Mesh::~Mesh(void)
{
	glDeleteBuffers(1, &VBO_ID);
	glDeleteBuffers(1, &EBO_ID);
}

void Mesh::draw(void)
{
	glBindBuffer(GL_ARRAY_BUFFER, VBO_ID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_ID);

	// Setup vertex attributes
	unsigned int offset = 0;
	for (int i = 0; i < packing.size(); i++)
	{
		unsigned int packFloats = PackedFloats(packing[i]);
		GLint adr = 0;
		if (packing[i] == POS) adr = vPosAdr;
		else if (packing[i] == COL) adr = vColAdr;
		else if (packing[i] == TEX) adr = vUVAdr;
		else if (packing[i] == NRM) adr = vNrmAdr;
		else {
			std::cout << "Unknown packing " << packing[i] << "! \n";
			continue;
		}
		glVertexAttribPointer(adr, packFloats, GL_FLOAT, GL_FALSE, sizeof(float) * FpV, (void *)(sizeof(float) * offset));
		glEnableVertexAttribArray(adr);
		offset += packFloats;
	}

	if (EBO_ID == 0)
		glDrawArrays(mode, 0, vertexCount);
	else
		glDrawElements(mode, elementCount, GL_UNSIGNED_INT, 0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Mesh::setMode (GLenum Mode)
{
	mode = Mode;
}
